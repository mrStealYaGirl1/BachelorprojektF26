#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

#include "bmi2.h"
#include "bmi270.h"
#include "swing_manager.h"

static const char *TAG = "IMU";

/* =====================================================
   RINGBUFFER
===================================================== */

static imu_ringbuffer_t imu_rb;

/* =====================================================
   IMPACT DETECTION
===================================================== */

#define IMPACT_ENERGY_WINDOW      8
#define IMPACT_THRESHOLD          20.0f
#define IMPACT_COOLDOWN_SAMPLES   200   // 1 sekund ved 200 Hz

static float energy_buffer[IMPACT_ENERGY_WINDOW] = {0};
static uint8_t energy_index = 0;
static float energy_sum = 0;
static uint32_t cooldown_counter = 0;

//gyro gate
#define GYRO_GATE_THRESHOLD_DPS   250.0f   // startværdi, skal tunes
static float last_gyro_mag_dps = 0.0f;
//spike detection
#define IMPACT_RISE_THRESHOLD  27.0f   // startværdi, tune via log
static float prev_energy_sum = 0.0f;

/* =====================================================
   BIAS
===================================================== */

static float gyro_bias_x = 0;
static float gyro_bias_y = 0;
static float gyro_bias_z = 0;

static float acc_bias_x = 0;
static float acc_bias_y = 0;
static float acc_bias_z = 0;

static void imu_calibrate(void);
static uint8_t detect_impact(float acc_dynamic);

/* =====================================================
   SPI CONFIG
===================================================== */

#define PIN_MOSI  35
#define PIN_MISO  37
#define PIN_SCLK  36
#define PIN_CS    10

static spi_device_handle_t s_spi;
static struct bmi2_dev s_bmi;

/* =====================================================
   SPI WRITE
===================================================== */

static int8_t spi_write(uint8_t reg_addr,
                        const uint8_t *data,
                        uint32_t len,
                        void *intf_ptr)
{
    spi_device_handle_t dev = (spi_device_handle_t)intf_ptr;

    uint8_t tx[1 + len];
    tx[0] = reg_addr & 0x7F;
    memcpy(&tx[1], data, len);

    spi_transaction_t t = {0};
    t.length = (1 + len) * 8;
    t.tx_buffer = tx;

    if (spi_device_transmit(dev, &t) != ESP_OK)
        return BMI2_E_COM_FAIL;

    return BMI2_OK;
}

/* =====================================================
   SPI READ
===================================================== */

static int8_t spi_read(uint8_t reg_addr,
                       uint8_t *data,
                       uint32_t len,
                       void *intf_ptr)
{
    spi_device_handle_t dev = (spi_device_handle_t)intf_ptr;

    uint8_t tx[1 + len];
    uint8_t rx[1 + len];

    tx[0] = reg_addr | 0x80;
    memset(&tx[1], 0, len);

    spi_transaction_t t = {0};
    t.length = (1 + len) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    if (spi_device_transmit(dev, &t) != ESP_OK)
        return BMI2_E_COM_FAIL;

    memcpy(data, &rx[1], len);

    return BMI2_OK;
}

static void spi_delay(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period_us);
}

static void spi_init_bus(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi));
}

/* =====================================================
   INIT
===================================================== */

void imu_init(void)
{
    ESP_LOGI(TAG, "Initializing BMI270...");

    spi_init_bus();
    vTaskDelay(pdMS_TO_TICKS(100));

    s_bmi.read = spi_read;
    s_bmi.write = spi_write;
    s_bmi.delay_us = spi_delay;
    s_bmi.intf = BMI2_SPI_INTF;
    s_bmi.intf_ptr = s_spi;

    if (bmi270_init(&s_bmi) != BMI2_OK)
    {
        ESP_LOGE(TAG, "BMI270 init failed");
        return;
    }

    struct bmi2_sens_config sens_cfg[2];
    sens_cfg[0].type = BMI2_ACCEL;
    sens_cfg[1].type = BMI2_GYRO;

    bmi2_get_sensor_config(sens_cfg, 2, &s_bmi);

    sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
    sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
    sens_cfg[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
    sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;

    bmi2_set_sensor_config(sens_cfg, 2, &s_bmi);

    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    bmi2_sensor_enable(sens_list, 2, &s_bmi);

    imu_ringbuffer_init();
    imu_calibrate();

    ESP_LOGI(TAG, "IMU ready");
}

/* =====================================================
   CALIBRATION
===================================================== */

static void imu_calibrate(void)
{
    struct bmi2_sens_data sensor_data;
    const int samples = 500;

    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;

    ESP_LOGI(TAG, "Calibrating IMU... Keep it still!");

    for (int i = 0; i < samples; i++)
    {
        if (bmi2_get_sensor_data(&sensor_data, &s_bmi) == BMI2_OK)
        {
            float ax = (sensor_data.acc.x / 16384.0f) * 9.81f;
            float ay = (sensor_data.acc.y / 16384.0f) * 9.81f;
            float az = (sensor_data.acc.z / 16384.0f) * 9.81f;

            float gx = sensor_data.gyr.x * (2000.0f / 32768.0f);
            float gy = sensor_data.gyr.y * (2000.0f / 32768.0f);
            float gz = sensor_data.gyr.z * (2000.0f / 32768.0f);

            sum_ax += ax;
            sum_ay += ay;
            sum_az += az;

            sum_gx += gx;
            sum_gy += gy;
            sum_gz += gz;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    acc_bias_x = sum_ax / samples;
    acc_bias_y = sum_ay / samples;
    acc_bias_z = (sum_az / samples) - 9.81f;

    gyro_bias_x = sum_gx / samples;
    gyro_bias_y = sum_gy / samples;
    gyro_bias_z = sum_gz / samples;

    ESP_LOGI(TAG, "Calibration done");
}

/* =====================================================
   IMPACT DETECTION
===================================================== */

static float energy_sum_peak = 0.0f;
static uint32_t peak_print_counter = 0;
static float gyro_peak_1s = 0.0f;

static uint8_t detect_impact(float acc_dynamic)
{
    float energy = acc_dynamic * acc_dynamic;

    energy_sum -= energy_buffer[energy_index];
    energy_buffer[energy_index] = energy;
    energy_sum += energy;

    float dE = energy_sum - prev_energy_sum;    // delta Energy for spike detection - ændring i energi fra én sample til den næste. (ryk => dE moderat (energi bygger op over flere samples), slag => dE stor (energi ændrer sig meget på kort tid))
    prev_energy_sum = energy_sum;


    if (energy_sum > energy_sum_peak) energy_sum_peak = energy_sum;
    //if (last_gyro_mag_dps > gyro_peak_1s) gyro_peak_1s = last_gyro_mag_dps;


    peak_print_counter++;
    if (peak_print_counter >= 200) { // ca 1 sekund ved 200 Hz
        ESP_LOGI("IMPACT_DEBUG", "energy_sum=%.2f  peak_1s=%.2f", energy_sum, energy_sum_peak);
        energy_sum_peak = 0.0f;

        //ESP_LOGI("IMPACT_DEBUG", "E=%.2f peakE=%.2f  gyro=%.1f peakG=%.1f", energy_sum, energy_sum_peak, last_gyro_mag_dps, gyro_peak_1s);
        //gyro_peak_1s = 0.0f;

        peak_print_counter = 0;
    }



    energy_index = (energy_index + 1) % IMPACT_ENERGY_WINDOW;

    //ESP_LOGI("IMPACT_DEBUG", "energy_sum=%.2f", energy_sum);

    if (cooldown_counter > 0)
    {
        cooldown_counter--;
        return 0;
    }

    // if (energy_sum > IMPACT_THRESHOLD)
    // {
    //     ESP_LOGW("IMPACT_DEBUG", "IMPACT! energy_sum=%.2f", energy_sum);
    //     cooldown_counter = IMPACT_COOLDOWN_SAMPLES;
    //     return 1;
    // }

    // if (energy_sum > IMPACT_THRESHOLD && last_gyro_mag_dps > GYRO_GATE_THRESHOLD_DPS)
    // {
    //     ESP_LOGW("IMPACT_DEBUG", "IMPACT! E=%.2f gyro=%.1f dps", energy_sum, last_gyro_mag_dps);
    //     cooldown_counter = IMPACT_COOLDOWN_SAMPLES;
    //     return 1;
    // }
    if (energy_sum > IMPACT_THRESHOLD && dE > IMPACT_RISE_THRESHOLD)
{
    ESP_LOGW("IMPACT_DEBUG", "IMPACT! E=%.2f dE=%.2f", energy_sum, dE);
    cooldown_counter = IMPACT_COOLDOWN_SAMPLES;
    return 1;
}


    // log why it does not trigger impact
    if (energy_sum > IMPACT_THRESHOLD && dE <= IMPACT_RISE_THRESHOLD)
    {
        ESP_LOGI("IMPACT_DEBUG", "Blocked by rise threshold: E=%.2f dE=%.2f", energy_sum, dE);
    }


    return 0;
}

/* =====================================================
   TASK
===================================================== */

void imu_task(void *pvParameters)
{
    struct bmi2_sens_data sensor_data;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        if (bmi2_get_sensor_data(&sensor_data, &s_bmi) == BMI2_OK)
        {
            imu_sample_t sample;

            sample.ax = sensor_data.acc.x;
            sample.ay = sensor_data.acc.y;
            sample.az = sensor_data.acc.z;
            sample.gx = sensor_data.gyr.x;
            sample.gy = sensor_data.gyr.y;
            sample.gz = sensor_data.gyr.z;
            sample.timestamp_us = esp_timer_get_time();

            imu_ringbuffer_push(&sample);

            float ax = ((sensor_data.acc.x / 16384.0f) * 9.81f) - acc_bias_x;
            float ay = ((sensor_data.acc.y / 16384.0f) * 9.81f) - acc_bias_y;
            float az = ((sensor_data.acc.z / 16384.0f) * 9.81f) - acc_bias_z;

            float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
            float acc_dynamic = acc_mag - 9.81f;

            // Gyro raw -> dps (range = 2000 dps)
            float gx_dps = (sensor_data.gyr.x * (2000.0f / 32768.0f)) - gyro_bias_x;
            float gy_dps = (sensor_data.gyr.y * (2000.0f / 32768.0f)) - gyro_bias_y;
            float gz_dps = (sensor_data.gyr.z * (2000.0f / 32768.0f)) - gyro_bias_z;

            last_gyro_mag_dps = sqrtf(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);



            if (detect_impact(acc_dynamic))
            {
                uint32_t idx = (imu_rb.write_index == 0)
                               ? (IMU_BUFFER_SIZE - 1)
                               : (imu_rb.write_index - 1);

                swing_manager_notify_impact(idx);
            }
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5)); // 200 Hz --- FreeRTOS changed to 1000 Hz (100 Hz before)
    }
}

/* =====================================================
   RINGBUFFER IMPLEMENTATION
===================================================== */

void imu_ringbuffer_init(void)
{
    imu_rb.write_index = 0;
    imu_rb.wrapped = 0;
}

void imu_ringbuffer_push(const imu_sample_t *sample)
{
    imu_rb.buffer[imu_rb.write_index] = *sample;

    imu_rb.write_index++;

    if (imu_rb.write_index >= IMU_BUFFER_SIZE)
    {
        imu_rb.write_index = 0;
        imu_rb.wrapped = 1;
    }
}

imu_ringbuffer_t* imu_get_ringbuffer(void)
{
    return &imu_rb;
}
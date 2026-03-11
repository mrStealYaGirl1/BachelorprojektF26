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
#include "ble_manager.h"

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
   PUTT START DETECTION
===================================================== */

typedef enum {
    PUTT_STATE_IDLE = 0,
    PUTT_STATE_READY,
    PUTT_STATE_POSSIBLE_START,
    PUTT_STATE_IN_PUTT
} putt_state_t;

static putt_state_t s_putt_state = PUTT_STATE_IDLE;

/* thresholds - startværdier, skal tunes med logs */
#define PUTT_READY_GYRO_THRESHOLD_DPS      8.0f
#define PUTT_READY_MIN_SAMPLES             80    // 0.4 s ved 200 Hz

#define PUTT_START_GYRO_THRESHOLD_DPS      25.0f
#define PUTT_START_MIN_SAMPLES             8     // 40 ms ved 200 Hz

#define PUTT_CANCEL_GYRO_THRESHOLD_DPS     12.0f
#define PUTT_MAX_DURATION_SAMPLES          400   // 2.0 s ved 200 Hz

#define PUTT_START_PREBUFFER_SAMPLES       100   // 0.5 s før start

static uint32_t s_putt_still_counter = 0;
static uint32_t s_putt_start_counter = 0;
static uint32_t s_putt_duration_counter = 0;

static bool s_putt_start_valid = false;
static uint32_t s_putt_start_idx = 0;

/* =====================================================
   BIAS
===================================================== */

static float gyro_bias_x = 0;
static float gyro_bias_y = 0;
static float gyro_bias_z = 0;

static float acc_bias_x = 0;
static float acc_bias_y = 0;
static float acc_bias_z = 0;



/* =========================================================
   FORWARD DECLARATIONS
========================================================= */

static void imu_calibrate(void);
static uint8_t detect_impact(float acc_dynamic);

static uint32_t rb_index_back(uint32_t index, uint32_t back);
static uint8_t detect_putt_start(float gyro_mag_dps, uint32_t current_idx);
static void reset_putt_start_detector(void);

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
//static float gyro_peak_1s = 0.0f;

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
        //ESP_LOGI("IMPACT_DEBUG", "energy_sum=%.2f  peak_1s=%.2f", energy_sum, energy_sum_peak);
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
        //ESP_LOGI("IMPACT_DEBUG", "Blocked by rise threshold: E=%.2f dE=%.2f", energy_sum, dE);
    }


    return 0;
}

/* =====================================================
   RESET IMPACT DETECTOR
===================================================== */

static void reset_impact_detector(void)
{
    memset(energy_buffer, 0, sizeof(energy_buffer));
    energy_index = 0;
    energy_sum = 0.0f;
    prev_energy_sum = 0.0f;
    cooldown_counter = 0;
    energy_sum_peak = 0.0f;
    peak_print_counter = 0;
}

// helper for indexing back in ringbuffer with wraparound
static uint32_t rb_index_back(uint32_t index, uint32_t back)
{
    if (back <= index) {
        return index - back;
    }
    return IMU_BUFFER_SIZE + index - back;
}


/* =====================================================
   TASK
===================================================== */

void imu_task(void *pvParameters)
{
    struct bmi2_sens_data sensor_data;
    TickType_t last_wake_time = xTaskGetTickCount();

    // uint32_t counter = 0;
    // TickType_t last_send = xTaskGetTickCount();

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

            static bool ble_was_busy = false;


            // Check if BLE is busy with IMU TX - if so, ignore impacts and reset detector when BLE is free again
            bool ble_busy = ble_manager_is_imu_tx_busy();
            if (ble_busy)
            {
                if (!ble_was_busy) {
                    ESP_LOGI(TAG, "Ignoring impacts while BLE IMU TX is busy");
                    reset_impact_detector();
                    reset_putt_start_detector();
                }

                ble_was_busy = true;
            }
            else
            {
                if (ble_was_busy) {
                    ESP_LOGI(TAG, "BLE IMU TX finished, impact/start detection enabled again");
                    reset_impact_detector();
                    reset_putt_start_detector();
                    ble_was_busy = false;
                }

                uint32_t current_idx = (imu_rb.write_index == 0)
                                    ? (IMU_BUFFER_SIZE - 1)
                                    : (imu_rb.write_index - 1);

                /* forsøg at finde putt-start */
                (void)detect_putt_start(last_gyro_mag_dps, current_idx);

                /* impact bekræfter det rigtige event */
                if (detect_impact(acc_dynamic))
                {
                    swing_manager_notify_impact(current_idx);
                }
            }
            // else
            // {
            //     if (ble_was_busy) {
            //         ESP_LOGI(TAG, "BLE IMU TX finished, impact detection enabled again");
            //         reset_impact_detector();
            //         ble_was_busy = false;
            //     }

            //     if (detect_impact(acc_dynamic))
            //     {
            //         uint32_t idx = (imu_rb.write_index == 0)
            //                     ? (IMU_BUFFER_SIZE - 1)
            //                     : (imu_rb.write_index - 1);

            //         swing_manager_notify_impact(idx);
            //     }
            // }




            // if (detect_impact(acc_dynamic))
            // {
            //     uint32_t idx = (imu_rb.write_index == 0)
            //                    ? (IMU_BUFFER_SIZE - 1)
            //                    : (imu_rb.write_index - 1);

            //     swing_manager_notify_impact(idx);
            // }
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






static uint8_t detect_putt_start(float gyro_mag_dps, uint32_t current_idx)
{
    switch (s_putt_state)
    {
        case PUTT_STATE_IDLE:
        {
            if (gyro_mag_dps < PUTT_READY_GYRO_THRESHOLD_DPS) {
                s_putt_still_counter++;
            } else {
                s_putt_still_counter = 0;
            }

            if (s_putt_still_counter >= PUTT_READY_MIN_SAMPLES) {
                s_putt_state = PUTT_STATE_READY;
                s_putt_start_counter = 0;
                s_putt_duration_counter = 0;
                // ESP_LOGI(TAG, "PUTT_READY");
            }
            break;
        }

        case PUTT_STATE_READY:
        {
            /* hvis køllen ikke længere er stille, men heller ikke rigtig starter,
               så bliv i READY så længe det ikke er voldsom bevægelse */
            if (gyro_mag_dps > PUTT_START_GYRO_THRESHOLD_DPS) {
                s_putt_state = PUTT_STATE_POSSIBLE_START;
                s_putt_start_counter = 1;
            } else if (gyro_mag_dps < PUTT_READY_GYRO_THRESHOLD_DPS) {
                /* stadig rolig */
            } else {
                /* lidt småbevægelse, men ikke nok til start */
            }
            break;
        }

        case PUTT_STATE_POSSIBLE_START:
        {
            if (gyro_mag_dps > PUTT_START_GYRO_THRESHOLD_DPS) {
                s_putt_start_counter++;

                if (s_putt_start_counter >= PUTT_START_MIN_SAMPLES) {
                    s_putt_start_idx = rb_index_back(current_idx, PUTT_START_PREBUFFER_SAMPLES);
                    s_putt_start_valid = true;
                    s_putt_state = PUTT_STATE_IN_PUTT;
                    s_putt_duration_counter = 0;

                    ESP_LOGI(TAG, "PUTT START detected at idx=%lu (event start idx=%lu)",
                             (unsigned long)current_idx,
                             (unsigned long)s_putt_start_idx);
                    return 1;
                }
            } else if (gyro_mag_dps < PUTT_CANCEL_GYRO_THRESHOLD_DPS) {
                /* falsk alarm -> tilbage til READY */
                s_putt_state = PUTT_STATE_READY;
                s_putt_start_counter = 0;
            }
            break;
        }

        case PUTT_STATE_IN_PUTT:
        {
            s_putt_duration_counter++;

            /* timeout hvis vi aldrig får impact */
            if (s_putt_duration_counter > PUTT_MAX_DURATION_SAMPLES) {
                ESP_LOGI(TAG, "PUTT start timeout -> reset");
                reset_putt_start_detector();
            }
            break;
        }

        default:
            reset_putt_start_detector();
            break;
    }

    return 0;
}


static void reset_putt_start_detector(void)
{
    s_putt_state = PUTT_STATE_IDLE;
    s_putt_still_counter = 0;
    s_putt_start_counter = 0;
    s_putt_duration_counter = 0;
    s_putt_start_valid = false;
    s_putt_start_idx = 0;
}

bool imu_putt_start_is_valid(void)
{
    return s_putt_start_valid;
}

uint32_t imu_get_putt_start_idx(void)
{
    return s_putt_start_idx;
}

void imu_clear_putt_start(void)
{
    s_putt_start_valid = false;
    s_putt_start_idx = 0;
}

void imu_reset_putt_start_detector(void)
{
    reset_putt_start_detector();
}
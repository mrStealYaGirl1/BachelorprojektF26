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
#define IMPACT_THRESHOLD          25.0f
#define IMPACT_COOLDOWN_SAMPLES   200   // 1 sekund ved 200 Hz

typedef enum
{
    SWING_IDLE,
    SWING_ADDRESS,
    SWING_BACKSWING,
    SWING_FORWARD,
    SWING_FOLLOW
} swing_state_t;

static swing_state_t swing_state = SWING_IDLE;
static uint32_t still_counter = 0;
static uint32_t forward_counter = 0;

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
//current swing timing
static swing_timing_t current_swing = {0};
static uint32_t next_swing_id = 1;
static float forward_peak_energy = 0.0f;
static uint32_t forward_peak_idx = 0;

// swing detection state machine thesholds
#define GZ_BACKSWING_START_DPS   10.0f
#define GZ_FORWARD_START_DPS    -20.0f
#define GZ_IDLE_DPS              3.0f

#define SWING_CONFIRM_SAMPLES    2
#define ADDRESS_RESET_SAMPLES  5
#define ADDRESS_STILL_RESET_SAMPLES  400 // 2 sekunder ved 200 Hz

static uint8_t backswing_confirm_count = 0;
static uint8_t forward_confirm_count = 0;
static uint8_t address_reset_confirm_count = 0;
static uint16_t address_still_counter = 0;

static uint8_t backswing_reset_confirm_count = 0;
#define BACKSWING_RESET_SAMPLES  20

static float backswing_peak_gz = 0.0f;
static uint32_t backswing_peak_idx = 0;
static uint8_t forward_fall_confirm_count = 0;

#define GZ_BACKSWING_PEAK_DROP_DPS  15.0f
#define IMPACT_FORWARD_GZ_MIN_DPS   -15.0f

static int32_t backswing_zero_cross_idx = 0;
static uint8_t zero_cross_confirm_count = 0;
#define GZ_ZERO_CROSS_BAND_DPS      5.0f
#define ZERO_CROSS_CONFIRM_SAMPLES  2

#define ADDRESS_WRONG_DIR_DPS            -10.0f
#define ADDRESS_WRONG_ROT_GYRO_MAG_DPS    30.0f
#define ADDRESS_WRONG_ROT_GZ_MAX_DPS       8.0f
#define ADDRESS_OTHER_AXIS_DPS            60.0f
#define ADDRESS_WRONG_CONFIRM_SAMPLES      3

static uint8_t address_wrong_dir_count = 0;

/* ====================================================
   BIAS
===================================================== */

static float gyro_bias_x = 0;
static float gyro_bias_y = 0;
static float gyro_bias_z = 0;

static float acc_bias_x = 0;
static float acc_bias_y = 0;
static float acc_bias_z = 0;

static void imu_calibrate(void);
static void reset_current_swing(void);
static uint8_t detect_swing(float acc_dynamic,
                            float gyro_mag,
                            float gx_dps,
                            float gy_dps,
                            float gz_dps,
                            uint32_t sample_idx);

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
    //ESP_LOGI(TAG, "Initializing BMI270...");

    spi_init_bus();
    vTaskDelay(pdMS_TO_TICKS(100));

    s_bmi.read = spi_read;
    s_bmi.write = spi_write;
    s_bmi.delay_us = spi_delay;
    s_bmi.intf = BMI2_SPI_INTF;
    s_bmi.intf_ptr = s_spi;

    if (bmi270_init(&s_bmi) != BMI2_OK)
    {
        //ESP_LOGE(TAG, "BMI270 init failed");
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

    //ESP_LOGI(TAG, "IMU ready");
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

    //ESP_LOGI(TAG, "Calibrating IMU... Keep it still!");

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

    //ESP_LOGI(TAG, "Calibration done");
}



// static uint32_t find_backswing_start_idx(uint32_t current_idx)
// {
//     imu_ringbuffer_t *rb = imu_get_ringbuffer();

//     const uint32_t search_back = 40;   // 40 samples = 0.2 s ved 200 Hz
//     uint32_t best_idx = current_idx;

//     for (uint32_t k = 1; k <= search_back; k++)
//     {
//         uint32_t idx = (current_idx + IMU_BUFFER_SIZE - k) % IMU_BUFFER_SIZE;

//         float gz_dps = (rb->buffer[idx].gz * (2000.0f / 32768.0f)) - gyro_bias_z;

//         if (fabsf(gz_dps) < 5.0f)
//         {
//             best_idx = idx;
//             break;
//         }
//     }

//     return best_idx;
// }
static uint32_t find_backswing_start_idx(uint32_t trigger_idx)
{
    imu_ringbuffer_t *rb = imu_get_ringbuffer();

    const uint32_t search_back = 80;   // 0.4 s ved 200 Hz
    const float still_thresh = 3.0f;
    const uint8_t still_needed = 3;

    uint32_t best_idx = trigger_idx;
    uint8_t still_count = 0;
    bool found_still_region = false;

    for (uint32_t k = search_back; k > 0; k--)
    {
        uint32_t idx = (trigger_idx + IMU_BUFFER_SIZE - k) % IMU_BUFFER_SIZE;
        float gz_dps = (rb->buffer[idx].gz * (2000.0f / 32768.0f)) - gyro_bias_z;

        if (fabsf(gz_dps) < still_thresh)
        {
            still_count++;
            if (still_count >= still_needed)
            {
                found_still_region = true;
            }
        }
        else
        {
            if (found_still_region)
            {
                best_idx = idx;   // første sample hvor bevægelsen starter efter ro
                break;
            }
            still_count = 0;
        }
    }

    return best_idx;
}


/* =====================================================
   IMPACT DETECTION
===================================================== */

static float energy_sum_peak = 0.0f;
static uint32_t peak_print_counter = 0;
//static float gyro_peak_1s = 0.0f;

static uint8_t detect_swing(float acc_dynamic,
                            float gyro_mag,
                            float gx_dps,
                            float gy_dps,
                            float gz_dps,
                            uint32_t sample_idx)
{
    /*
       Denne funktion implementerer swing state machine.
       Den bruger både gyro (rotation) og impact-detektor (energy spike).
    */
    /*
       State machine:
       IDLE      -> køllen står stille
       ADDRESS   -> setup/adresse
       BACKSWING -> gz stiger / er høj positiv
       FORWARD   -> gz falder igen efter backswing
       FOLLOW    -> efter impact
    */

    switch (swing_state)
    {

        /* -----------------------------------------
           SWING_IDLE
           Køllen er i ro
        ----------------------------------------- */

        case SWING_IDLE:
        {
            /* Hvis rotation næsten er nul → køllen står stille */
            if (gyro_mag < 5.0f)
                still_counter++;
            else
                still_counter = 0;

            /* Hvis køllen har været stille i 0.5 sek → ADDRESS */
            if (still_counter > 100)
            {
                swing_state = SWING_ADDRESS;
                still_counter = 0;
                forward_counter = 0;
                address_still_counter = 0;
                backswing_confirm_count = 0;
                zero_cross_confirm_count = 0;
                backswing_reset_confirm_count = 0;
                address_wrong_dir_count = 0;

                reset_current_swing();
                current_swing.swing_id = next_swing_id++;
                current_swing.address_start_us = imu_rb.buffer[sample_idx].timestamp_us;
                current_swing.address_start_idx = sample_idx;  
                
                ESP_LOGI("SWING", "ADDRESS detected");
            }

        break;
        }

        /* -----------------------------------------
           SWING_ADDRESS
           Spilleren står klar (vent på backswing)
        ----------------------------------------- */

        case SWING_ADDRESS:
        {
            /* backswing starter når gz bliver tydeligt positiv */
            if (gz_dps > GZ_BACKSWING_START_DPS)
            {
                backswing_confirm_count++;
            }
            else
            {
                backswing_confirm_count = 0;
            }

            if (backswing_confirm_count >= SWING_CONFIRM_SAMPLES)
            {
                swing_state = SWING_BACKSWING;
                backswing_confirm_count = 0;
                forward_confirm_count = 0;
                forward_fall_confirm_count = 0;
                address_wrong_dir_count = 0;

                uint32_t trigger_idx =
                    (sample_idx + IMU_BUFFER_SIZE - (SWING_CONFIRM_SAMPLES - 1)) % IMU_BUFFER_SIZE;

                uint32_t start_idx = find_backswing_start_idx(trigger_idx);

                current_swing.backswing_start_idx = start_idx;
                current_swing.backswing_start_us = imu_rb.buffer[start_idx].timestamp_us;

                backswing_peak_gz = gz_dps;
                backswing_peak_idx = sample_idx;

                ESP_LOGI("SWING", "BACKSWING (gz=%.1f)", gz_dps);
                break;
            }

            /* ugyldige bevægelser i ADDRESS -> reset til IDLE */
            bool wrong_direction =
                (gz_dps < ADDRESS_WRONG_DIR_DPS);   // bevæges forward

            bool wrong_rotation_pattern =
                (gyro_mag > ADDRESS_WRONG_ROT_GYRO_MAG_DPS &&   // for kraftig rotation
                gz_dps < ADDRESS_WRONG_ROT_GZ_MAX_DPS);         // rotation i forkert retning (fx negativ gz)

            bool too_much_other_axis =
                (fabsf(gx_dps) > ADDRESS_OTHER_AXIS_DPS ||      // for kraftig rotation i x aksen
                fabsf(gy_dps) > ADDRESS_OTHER_AXIS_DPS);        // for kraftig rotation i y aksen

            if (wrong_direction || wrong_rotation_pattern || too_much_other_axis)
            {
                address_wrong_dir_count++;
            }
            else
            {
                address_wrong_dir_count = 0;
            }

            if (address_wrong_dir_count >= ADDRESS_WRONG_CONFIRM_SAMPLES)
            {
                swing_state = SWING_IDLE;
                backswing_confirm_count = 0;
                forward_confirm_count = 0;
                forward_fall_confirm_count = 0;
                address_wrong_dir_count = 0;
                address_still_counter = 0;

                ESP_LOGI("SWING",
                        "RESET from ADDRESS to IDLE (wrong move: gz=%.1f gx=%.1f gy=%.1f gyro_mag=%.1f)",
                        gz_dps, gx_dps, gy_dps, gyro_mag);
            }

            break;
        }

        // case SWING_ADDRESS:

        //     /*
        //         Brug gz direkte:
        //         backswing starter når gz bliver tydeligt positiv
        //         TUNE DENNE TÆRSKEL
        //     */
        //      /* backswing starter når gz bliver tydeligt positiv */
        //     if (gz_dps > GZ_BACKSWING_START_DPS)
        //     {
        //         backswing_confirm_count++;
        //     }
        //     else
        //     {
        //         backswing_confirm_count = 0;
        //     }

        //     if (backswing_confirm_count >= SWING_CONFIRM_SAMPLES)
        //     {
        //         swing_state = SWING_BACKSWING;
        //         backswing_confirm_count = 0;
        //         forward_confirm_count = 0;
        //         forward_fall_confirm_count = 0;

        //         uint32_t trigger_idx =
        //             (sample_idx + IMU_BUFFER_SIZE - (SWING_CONFIRM_SAMPLES - 1)) % IMU_BUFFER_SIZE;

        //         uint32_t start_idx = find_backswing_start_idx(trigger_idx);

        //         current_swing.backswing_start_idx = start_idx;
        //         current_swing.backswing_start_us = imu_rb.buffer[start_idx].timestamp_us;

        //         backswing_peak_gz = gz_dps;
        //         backswing_peak_idx = sample_idx;

        //         ESP_LOGI("SWING", "BACKSWING (gz=%.1f)", gz_dps);
        //         break;
        //     }

        //     /* hvis brugeren bliver stille igen -> tilbage til IDLE */
        //     if (gyro_mag < GZ_IDLE_DPS)
        //     {
        //         address_still_counter++;
        //     }
        //     else
        //     {
        //         address_still_counter = 0;
        //     }

        //     if (address_still_counter >= ADDRESS_STILL_RESET_SAMPLES)
        //     {
        //         swing_state = SWING_IDLE;
        //         backswing_confirm_count = 0;
        //         forward_confirm_count = 0;
        //         address_still_counter = 0;

        //         ESP_LOGI("SWING", "RESET from ADDRESS to IDLE");
        //     }

        //     break;

        //     /* Hvis rotation bliver større → BACKSWING */
        //     if (gyro_mag > 20.0f)
        //     {
        //         swing_state = SWING_BACKSWING;

        //         current_swing.backswing_start_us = imu_rb.buffer[sample_idx].timestamp_us;
        //         current_swing.backswing_start_idx = sample_idx;

        //         ESP_LOGI("SWING", "BACKSWING");
        //     }

        // break;


        /* -----------------------------------------
           SWING_BACKSWING
           Køllen trækkes bagud (gz er høj / positiv og derefter begynder den at falde)
        ----------------------------------------- */

        case SWING_BACKSWING:

            /*
            FORWARD starter når gz er faldet igen.
            Hvis jeres forward faktisk går negativ, så kan denne
            tærskel senere ændres til fx gz_dps < -40.
            */

            /* opdater peak */
            if (gz_dps > backswing_peak_gz)
            {
                backswing_peak_gz = gz_dps;
                backswing_peak_idx = sample_idx;
            }

            /*
            Forward skal starte ved toppen af vinkelkurven.
            Det svarer til at gz går fra positiv til omkring 0 / negativ.
            Derfor kigger vi efter zero crossing efter et gyldigt backswing-peak.
            */
            if (backswing_peak_gz > GZ_BACKSWING_START_DPS)
            {
                if (gz_dps < 0.0f)
                {
                    zero_cross_confirm_count++;
                }
                else
                {
                    zero_cross_confirm_count = 0;
                }

                if (zero_cross_confirm_count >= ZERO_CROSS_CONFIRM_SAMPLES)
                {
                    backswing_zero_cross_idx = sample_idx;

                    swing_state = SWING_FORWARD;
                    zero_cross_confirm_count = 0;
                    backswing_reset_confirm_count = 0;
                    forward_counter = 0;

                    /* forward starter ved zero crossing = top af vinkelkurven */
                    current_swing.forward_start_us = imu_rb.buffer[backswing_zero_cross_idx].timestamp_us;
                    current_swing.forward_start_idx = backswing_zero_cross_idx;

                    forward_peak_energy = 0.0f;
                    forward_peak_idx = sample_idx;

                    ESP_LOGI("SWING",
                            "FORWARD at zero-cross (peak_gz=%.1f gz_now=%.1f)",
                            backswing_peak_gz, gz_dps);
                    break;
                }
            }

            /* reset hvis backswing dør ud */
            if (fabsf(gz_dps) < GZ_IDLE_DPS && gyro_mag < 6.0f)
            {
                backswing_reset_confirm_count++;
            }
            else
            {
                backswing_reset_confirm_count = 0;
            }

            if (backswing_reset_confirm_count >= BACKSWING_RESET_SAMPLES)
            {
                swing_state = SWING_IDLE;
                backswing_confirm_count = 0;
                forward_confirm_count = 0;
                forward_fall_confirm_count = 0;
                zero_cross_confirm_count = 0;
                backswing_reset_confirm_count = 0;

                ESP_LOGI("SWING", "RESET from BACKSWING");
            }

            break;
            

//ESP_LOGI("SWING_DBG", "state=%d gz=%.1f gyro_mag=%.1f", swing_state, gz_dps, gyro_mag);

        //     /* Når rotation bliver endnu større → FORWARD */
        //     if (gyro_mag > 40.0f)
        //     {
        //         swing_state = SWING_FORWARD;
        //         forward_counter = 0;

        //         current_swing.forward_start_us = imu_rb.buffer[sample_idx].timestamp_us;
        //         current_swing.forward_start_idx = sample_idx;

        //         forward_peak_energy = 0.0f;
        //         forward_peak_idx = sample_idx;

        //         ESP_LOGI("SWING", "FORWARD");
        //     }

        //     /* Hvis bevægelsen stopper → reset */
        //     if (gyro_mag < 5.0f)
        //     {
        //         swing_state = SWING_IDLE;
        //         ESP_LOGI("SWING", "RESET from BACKSWING");
        //     }
        // break;


        /* -----------------------------------------
           SWING_FORWARD
           Køllen accelererer frem mod bolden (impact detekteres her)
        ----------------------------------------- */

        case SWING_FORWARD:
        {

            forward_counter++;

            /*
               Her bruger vi den eksisterende impact-detektor
               baseret på energy + dE spike
            */

            float energy = acc_dynamic * acc_dynamic;

            energy_sum -= energy_buffer[energy_index];
            energy_buffer[energy_index] = energy;
            energy_sum += energy;

            float dE = energy_sum - prev_energy_sum;
            prev_energy_sum = energy_sum;

            energy_index = (energy_index + 1) % IMPACT_ENERGY_WINDOW;

            if (energy_sum > forward_peak_energy)
            {
                forward_peak_energy = energy_sum;
                forward_peak_idx = sample_idx;
            }

            if (cooldown_counter > 0)
            {
                cooldown_counter--;
                break;
            }

            if (energy_sum > IMPACT_THRESHOLD && dE > IMPACT_RISE_THRESHOLD && gz_dps < IMPACT_FORWARD_GZ_MIN_DPS)
            {
                current_swing.impact_idx = forward_peak_idx;
                current_swing.impact_us = imu_rb.buffer[forward_peak_idx].timestamp_us;
                
                ESP_LOGW("IMPACT_DEBUG", "IMPACT! E=%.2f dE=%.2f", energy_sum, dE);

                cooldown_counter = IMPACT_COOLDOWN_SAMPLES;
                swing_state = SWING_FOLLOW;

                current_swing.follow_start_us = imu_rb.buffer[sample_idx].timestamp_us;
                current_swing.follow_start_idx = sample_idx;

                return 1;
            }

            // if (energy_sum > IMPACT_THRESHOLD && dE > IMPACT_RISE_THRESHOLD)
            // {
            //     current_swing.impact_idx = forward_peak_idx;
            //     current_swing.impact_us = imu_rb.buffer[forward_peak_idx].timestamp_us;
                
            //     ESP_LOGW("IMPACT_DEBUG", "IMPACT! E=%.2f dE=%.2f", energy_sum, dE);

            //     cooldown_counter = IMPACT_COOLDOWN_SAMPLES;
            //     swing_state = SWING_FOLLOW;

            //     current_swing.follow_start_us = imu_rb.buffer[sample_idx].timestamp_us;
            //     current_swing.follow_start_idx = sample_idx;

            //     return 1;
            // }

            if (forward_counter > 200)
            {
                swing_state = SWING_IDLE;
                ESP_LOGI("SWING", "RESET from FORWARD timeout");
            }
            break;
        }

        /* -----------------------------------------
           SWING_FOLLOW
           Efter slaget
        ----------------------------------------- */

        case SWING_FOLLOW:
        {
            if (gyro_mag < 5.0f)
            {
                current_swing.end_us = imu_rb.buffer[sample_idx].timestamp_us;
                current_swing.end_idx = sample_idx;

                swing_state = SWING_IDLE;
                swing_manager_add_swing(current_swing);
                ESP_LOGI("SWING", "SWING END");
            }

            break;
        }
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

/* =====================================================
   IMU CSV LOGGER (til analyse af drift, ikke real-time) - print til UART i CSV format, kan fanges af en PC og gemmes i en fil
===================================================== */
// Hvis denne skal bruges - husk at fjerne ESP_LOGI'er i imu_init og imu_calibrate for at undgå at forstyrre CSV output

#define IMU_LOG_DURATION_MINUTES   4
#define IMU_LOG_SAMPLE_PERIOD_MS   5   // 200 Hz

void imu_csv_logger_task(void *pvParameters)
{
    struct bmi2_sens_data sensor_data;
    TickType_t last_wake = xTaskGetTickCount();

    vTaskDelay(pdMS_TO_TICKS(500));

    int64_t start_us = esp_timer_get_time();
    int64_t duration_us = (int64_t)IMU_LOG_DURATION_MINUTES * 60LL * 1000000LL;

    printf("t_us,gx_raw,gy_raw,gz_raw,gx_corr,gy_corr,gz_corr\n");

    while ((esp_timer_get_time() - start_us) < duration_us)
    {
        if (bmi2_get_sensor_data(&sensor_data, &s_bmi) == BMI2_OK)
        {
            int64_t t_us = esp_timer_get_time();

            int16_t gx_raw = sensor_data.gyr.x;
            int16_t gy_raw = sensor_data.gyr.y;
            int16_t gz_raw = sensor_data.gyr.z;

            float gx_corr = (gx_raw * (2000.0f / 32768.0f)) - gyro_bias_x;
            float gy_corr = (gy_raw * (2000.0f / 32768.0f)) - gyro_bias_y;
            float gz_corr = (gz_raw * (2000.0f / 32768.0f)) - gyro_bias_z;

            // logger både raw og korrigerede gyro-værdier for at kunne analysere drift og bias
            printf("%lld,%d,%d,%d,%.6f,%.6f,%.6f\n",
                   t_us,
                   gx_raw, gy_raw, gz_raw,
                   gx_corr, gy_corr, gz_corr);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_LOG_SAMPLE_PERIOD_MS));
    }

    vTaskDelete(NULL);
}

/* =====================================================
   TASK
===================================================== */

void imu_task(void *pvParameters)
{
    struct bmi2_sens_data sensor_data;
    TickType_t last_wake_time = xTaskGetTickCount();

    uint32_t counter = 0;
    TickType_t last_send = xTaskGetTickCount();

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
            
            uint32_t sample_idx = (imu_rb.write_index == 0)
                                ? (IMU_BUFFER_SIZE - 1)
                                : (imu_rb.write_index - 1);

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
                }

                ble_was_busy = true;
            }
            else
            {
                if (ble_was_busy) {
                    ESP_LOGI(TAG, "BLE IMU TX finished, impact detection enabled again");
                    reset_impact_detector();
                    ble_was_busy = false;
                }

                if (detect_swing(acc_dynamic, last_gyro_mag_dps, gx_dps, gy_dps, gz_dps, sample_idx))
                {
                    swing_manager_notify_impact(current_swing.impact_idx);
                }
            }




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


static void reset_current_swing(void)
{
    memset(&current_swing, 0, sizeof(current_swing));
}



/* =====================================================
   GZ ANGLE TEST
   Bruges til at teste om integreret gz-vinkel passer
   med en kendt fysisk rotation
===================================================== */

void imu_gz_angle_test_task(void *pvParameters)
{
    struct bmi2_sens_data sensor_data;
    TickType_t last_wake = xTaskGetTickCount();

    float angle_z_deg = 0.0f;
    int64_t last_t_us = esp_timer_get_time();

    ESP_LOGI(TAG, "GZ angle test started");
    ESP_LOGI(TAG, "Hold IMU still for a moment, then rotate known angle around Z-axis");

    while (1)
    {
        if (bmi2_get_sensor_data(&sensor_data, &s_bmi) == BMI2_OK)
        {
            int64_t now_us = esp_timer_get_time();
            float dt = (now_us - last_t_us) / 1000000.0f;
            last_t_us = now_us;

            float gz_raw_dps = sensor_data.gyr.z * (2000.0f / 32768.0f);
            float gz_corr_dps = gz_raw_dps - gyro_bias_z;

            angle_z_deg += gz_corr_dps * dt;

            static uint32_t print_counter = 0;
            print_counter++;

            // Print ca. 10 gange i sekundet ved 200 Hz
            if (print_counter >= 20)
            {
                print_counter = 0;

                printf("gz_corr_dps=%.3f, angle_z_deg=%.2f\n",
                       gz_corr_dps,
                       angle_z_deg);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5)); // 200 Hz
    }
}

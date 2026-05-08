// imu_processing.c - upgraded version

#include "imu_processing.h"
#include "../swing_manager/swing_manager.h"
#include "../imu/imu_driver.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <string.h>

/* =====================================================
   CONFIG
===================================================== */

#define IMPACT_ENERGY_WINDOW        8
#define IMPACT_THRESHOLD            18.0f
#define IMPACT_RISE_THRESHOLD       12.0f
#define IMPACT_COOLDOWN_SAMPLES     200
#define IMPACT_FORWARD_GZ_MIN_DPS  -15.0f

#define GZ_BACKSWING_START_DPS      10.0f
#define GZ_IDLE_DPS                  3.0f

#define SWING_CONFIRM_SAMPLES        2
#define RESET_CONFIRM_SAMPLES       20
#define FORWARD_TIMEOUT_SAMPLES    300

#define ADDRESS_WRONG_DIR_DPS             -10.0f
#define ADDRESS_WRONG_ROT_GYRO_MAG_DPS    30.0f
#define ADDRESS_WRONG_ROT_GZ_MAX_DPS       8.0f
#define ADDRESS_OTHER_AXIS_DPS            60.0f
#define ADDRESS_WRONG_CONFIRM_SAMPLES      3

/* =====================================================
   ENERGY DETECTOR
===================================================== */

static float energy_buffer[IMPACT_ENERGY_WINDOW] = {0};
static uint8_t energy_index = 0;

static float energy_sum = 0.0f;
static float prev_energy_sum = 0.0f;

static uint32_t cooldown_counter = 0;

/* =====================================================
   STATE MACHINE
===================================================== */

typedef enum
{
    SWING_IDLE,
    SWING_ADDRESS,
    SWING_BACKSWING,
    SWING_FORWARD,
    SWING_FOLLOW
} swing_state_t;

static swing_state_t swing_state = SWING_IDLE;

/* =====================================================
   INTERNAL STATE
===================================================== */

static uint32_t still_counter = 0;
static uint8_t backswing_confirm = 0;
static uint8_t zero_cross_confirm = 0;
static uint8_t reset_confirm = 0;
static uint8_t address_wrong_dir_count = 0;
static uint32_t forward_counter = 0;

/* =====================================================
   CALIBRATION 
===================================================== */
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

static float acc_bias_x = 0.0f;
static float acc_bias_y = 0.0f;
static float acc_bias_z = 0.0f;

/* =====================================================
   SWING TIMING
===================================================== */

static swing_timing_t current_swing = {0};
static uint32_t next_swing_id = 1;

/* =====================================================
   FIND BACKSWING PEAK & ZERO CROSS (forward start)
===================================================== */
static float backswing_peak_gz = 0.0f;
static uint32_t backswing_peak_idx = 0;
static int32_t backswing_zero_cross_idx = 0;

#define ZERO_CROSS_CONFIRM_SAMPLES 2

/* =====================================================
   INIT
===================================================== */

void imu_processing_init(void)
{
    memset(energy_buffer, 0, sizeof(energy_buffer));

    energy_index = 0;
    energy_sum = 0;
    prev_energy_sum = 0;
    cooldown_counter = 0;

    swing_state = SWING_IDLE;
    still_counter = 0;

    memset(&current_swing, 0, sizeof(current_swing));

    gyro_bias_x = gyro_bias_y = gyro_bias_z = 0.0f;
    acc_bias_x = acc_bias_y = acc_bias_z = 0.0f;

    printk("IMU processing upgraded ready\n");
}

/* =====================================================
   IMPACT DETECTION
===================================================== */

static int detect_impact(float acc_dynamic, float gz)
{
    float energy = acc_dynamic * acc_dynamic;

    energy_sum -= energy_buffer[energy_index];
    energy_buffer[energy_index] = energy;
    energy_sum += energy;

    float dE = energy_sum - prev_energy_sum;
    prev_energy_sum = energy_sum;

    energy_index = (energy_index + 1) % IMPACT_ENERGY_WINDOW;

    if (cooldown_counter > 0)
    {
        cooldown_counter--;
        return 0;
    }

    if (energy_sum > IMPACT_THRESHOLD &&
        dE > IMPACT_RISE_THRESHOLD &&
        gz < IMPACT_FORWARD_GZ_MIN_DPS)
    {
        cooldown_counter = IMPACT_COOLDOWN_SAMPLES;

        printk("🏌️ IMPACT! E=%.2f dE=%.2f gz=%.2f\n",
            (double)energy_sum,
            (double)dE,
            (double)gz);

        return 1;
    }

    return 0;
}


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

        float gz_dps =
            (rb->buffer[idx].gz * (2000.0f / 32768.0f)) - gyro_bias_z;

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
                best_idx = idx;
                break;
            }

            still_count = 0;
        }
    }

    return best_idx;
}


/* =====================================================
   CALIBRATION
===================================================== */

void imu_processing_calibrate(void)
{
    imu_sample_t sample;
    const int samples = 500;

    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;

    printk("Calibrating IMU... Keep it still!\n");

    for (int i = 0; i < samples; i++)
    {
        imu_get_latest(&sample);

        sum_ax += (sample.ax / 16384.0f) * 9.81f;
        sum_ay += (sample.ay / 16384.0f) * 9.81f;
        sum_az += (sample.az / 16384.0f) * 9.81f;

        sum_gx += sample.gx * (2000.0f / 32768.0f);
        sum_gy += sample.gy * (2000.0f / 32768.0f);
        sum_gz += sample.gz * (2000.0f / 32768.0f);

        k_msleep(5);
    }

    float avg_ax = sum_ax / samples;
    float avg_ay = sum_ay / samples;
    float avg_az = sum_az / samples;

    float avg_gx = sum_gx / samples;
    float avg_gy = sum_gy / samples;
    float avg_gz = sum_gz / samples;

    acc_bias_x = avg_ax;
    acc_bias_y = avg_ay + 9.81f;    // Y-aksen er lodret, og avg_ay er ca. -9.81 m/s²
    acc_bias_z = avg_az;

    gyro_bias_x = avg_gx;
    gyro_bias_y = avg_gy;
    gyro_bias_z = avg_gz;

    // float corr_ax = avg_ax - acc_bias_x;
    // float corr_ay = avg_ay - acc_bias_y;
    // float corr_az = avg_az - acc_bias_z;

    // float corr_gx = avg_gx - gyro_bias_x;
    // float corr_gy = avg_gy - gyro_bias_y;
    // float corr_gz = avg_gz - gyro_bias_z;

    // printk("\n=== IMU CALIBRATION ===\n");

    // printk("ACC avg [m/s²]:  X=%.3f Y=%.3f Z=%.3f\n",
    //        (double)avg_ax,
    //        (double)avg_ay,
    //        (double)avg_az);

    // printk("GYRO avg [dps]:  X=%.3f Y=%.3f Z=%.3f\n",
    //        (double)avg_gx,
    //        (double)avg_gy,
    //        (double)avg_gz);

    // printk("ACC bias [m/s²]: X=%.3f Y=%.3f Z=%.3f\n",
    //        (double)acc_bias_x,
    //        (double)acc_bias_y,
    //        (double)acc_bias_z);

    // printk("GYRO bias [dps]: X=%.3f Y=%.3f Z=%.3f\n",
    //        (double)gyro_bias_x,
    //        (double)gyro_bias_y,
    //        (double)gyro_bias_z);
        
    // printk("ACC corrected [m/s²]: X=%.3f Y=%.3f Z=%.3f | |acc|=%.3f\n",
    //     (double)corr_ax,
    //     (double)corr_ay,
    //     (double)corr_az,
    //     (double)sqrtf(corr_ax*corr_ax + corr_ay*corr_ay + corr_az*corr_az));

    // printk("GYRO corrected [dps]: X=%.3f Y=%.3f Z=%.3f | |gyro|=%.3f\n",
    //     (double)corr_gx,
    //     (double)corr_gy,
    //     (double)corr_gz,
    //     (double)sqrtf(corr_gx*corr_gx + corr_gy*corr_gy + corr_gz*corr_gz));

    // printk("=======================\n");

}



/* =====================================================
   MAIN PROCESS
===================================================== */

void imu_process_sample(const imu_sample_t *sample, uint32_t sample_idx)
{
    /* ---------- Convert ---------- */

    float ax = ((sample->ax / 16384.0f) * 9.81f) - acc_bias_x;
    float ay = ((sample->ay / 16384.0f) * 9.81f) - acc_bias_y;
    float az = ((sample->az / 16384.0f) * 9.81f) - acc_bias_z;

    float gx = (sample->gx * (2000.0f / 32768.0f)) - gyro_bias_x;
    float gy = (sample->gy * (2000.0f / 32768.0f)) - gyro_bias_y;
    float gz = (sample->gz * (2000.0f / 32768.0f)) - gyro_bias_z;

    float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
    float acc_dynamic = acc_mag - 9.81f;

    float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);

    /* =====================================================
       STATE MACHINE
    ===================================================== */

    switch (swing_state)
    {
        /* ---------------- IDLE ---------------- */

        case SWING_IDLE:
        {
            if (gyro_mag < 5.0f)
                still_counter++;
            else
                still_counter = 0;

            if (still_counter > 100)   // stå stille i 0.5 s ved 200 Hz før vi går i ADDRESS
            {
                memset(&current_swing, 0, sizeof(current_swing));

                current_swing.swing_id = next_swing_id++;

                current_swing.address_start_us = sample->timestamp_us;
                current_swing.address_start_idx = sample_idx;
                
                backswing_confirm = 0;
                zero_cross_confirm = 0;
                reset_confirm = 0;
                address_wrong_dir_count = 0;
                forward_counter = 0;

                swing_state = SWING_ADDRESS;
                still_counter = 0;

                printk("ADDRESS\n");
            }

            break;
        }

        /* ---------------- ADDRESS ---------------- */
        case SWING_ADDRESS:
        {
            /* ---------- gyldigt backswing ---------- */

            if (gz > GZ_BACKSWING_START_DPS)
            {
                backswing_confirm++;
            }
            else
            {
                backswing_confirm = 0;
            }

            if (backswing_confirm >= SWING_CONFIRM_SAMPLES)
            {
                uint32_t trigger_idx =
                    (sample_idx + IMU_BUFFER_SIZE -
                    (SWING_CONFIRM_SAMPLES - 1)) % IMU_BUFFER_SIZE;

                uint32_t start_idx =
                    find_backswing_start_idx(trigger_idx);

                current_swing.backswing_start_us =
                    imu_get_ringbuffer()->buffer[start_idx].timestamp_us;

                current_swing.backswing_start_idx = start_idx;

                backswing_peak_gz = gz;
                backswing_peak_idx = sample_idx;

                swing_state = SWING_BACKSWING;

                backswing_confirm = 0;
                address_wrong_dir_count = 0;

                printk("BACKSWING\n");
                break;
            }

            /* ---------- ugyldige bevægelser ---------- */

            bool wrong_direction =
                (gz < ADDRESS_WRONG_DIR_DPS);

            bool wrong_rotation_pattern =
                (gyro_mag > ADDRESS_WRONG_ROT_GYRO_MAG_DPS &&
                gz < ADDRESS_WRONG_ROT_GZ_MAX_DPS);

            bool too_much_other_axis =
                (fabsf(gx) > ADDRESS_OTHER_AXIS_DPS ||
                fabsf(gy) > ADDRESS_OTHER_AXIS_DPS);

            if (wrong_direction ||
                wrong_rotation_pattern ||
                too_much_other_axis)
            {
                address_wrong_dir_count++;
            }
            else
            {
                address_wrong_dir_count = 0;
            }

            if (address_wrong_dir_count >=
                ADDRESS_WRONG_CONFIRM_SAMPLES)
            {
                swing_state = SWING_IDLE;

                backswing_confirm = 0;
                address_wrong_dir_count = 0;

                printk("RESET from ADDRESS\n");
            }

            break;
        }
        // case SWING_ADDRESS:
        // {
        //     if (gz > GZ_BACKSWING_START_DPS)
        //         backswing_confirm++;
        //     else
        //         backswing_confirm = 0;

        //     if (backswing_confirm >= SWING_CONFIRM_SAMPLES)
        //     {
        //         uint32_t trigger_idx =
        //             (sample_idx + IMU_BUFFER_SIZE - (SWING_CONFIRM_SAMPLES - 1)) % IMU_BUFFER_SIZE;

        //         uint32_t start_idx = find_backswing_start_idx(trigger_idx);

        //         current_swing.backswing_start_us =
        //             imu_get_ringbuffer()->buffer[start_idx].timestamp_us;

        //         current_swing.backswing_start_idx = start_idx;

        //         backswing_peak_gz = gz;
        //         backswing_peak_idx = sample_idx;

        //         swing_state = SWING_BACKSWING;
        //         backswing_confirm = 0;

        //         printk("BACKSWING\n");
        //     }

        //     break;
        // }

        /* ---------------- BACKSWING ---------------- */

        case SWING_BACKSWING:
        {
            if (gz > backswing_peak_gz)
            {
                backswing_peak_gz = gz;
                backswing_peak_idx = sample_idx;
            }

            if (backswing_peak_gz > GZ_BACKSWING_START_DPS)
            {
                if (gz < 0.0f)
                    zero_cross_confirm++;
                else
                    zero_cross_confirm = 0;

                if (zero_cross_confirm >= ZERO_CROSS_CONFIRM_SAMPLES)
                {
                    backswing_zero_cross_idx = sample_idx;

                    current_swing.forward_start_us =
                        imu_get_ringbuffer()->buffer[backswing_zero_cross_idx].timestamp_us;

                    current_swing.forward_start_idx = backswing_zero_cross_idx;

                    swing_state = SWING_FORWARD;
                    zero_cross_confirm = 0;
                    forward_counter = 0;

                    printk("FORWARD (peak=%d)\n", (int)backswing_peak_gz);
                    break;
                }
            }
            else
            {
                zero_cross_confirm = 0;
            }

            if (fabsf(gz) < GZ_IDLE_DPS && gyro_mag < 6.0f)
                reset_confirm++;
            else
                reset_confirm = 0;

            if (reset_confirm >= RESET_CONFIRM_SAMPLES)
            {
                swing_state = SWING_IDLE;
                reset_confirm = 0;

                printk("RESET from BACKSWING\n");
            }

            break;
        }

        /* ---------------- FORWARD ---------------- */

        case SWING_FORWARD:
        {
            forward_counter++;

            if (detect_impact(acc_dynamic, gz))
            {
                current_swing.impact_us = sample->timestamp_us;
                current_swing.impact_idx = sample_idx;

                current_swing.follow_start_us = sample->timestamp_us;
                current_swing.follow_start_idx = sample_idx;

                swing_manager_notify_impact(current_swing.impact_idx,
                                            current_swing.impact_us);

                swing_state = SWING_FOLLOW;

                printk("IMPACT -> FOLLOW idx=%u t=%llu us\n",
                    current_swing.impact_idx,
                    (unsigned long long)current_swing.impact_us);
                break;
            }

            if (forward_counter > FORWARD_TIMEOUT_SAMPLES)
            {
                swing_state = SWING_IDLE;

                printk("RESET from FORWARD (timeout)\n");
            }

            break;
        }

        /* ---------------- FOLLOW ---------------- */

        case SWING_FOLLOW:
        {
            if (gyro_mag < GZ_IDLE_DPS)
            {
                current_swing.end_us = sample->timestamp_us;
                current_swing.end_idx = sample_idx;

                swing_manager_add_swing(current_swing);

                swing_state = SWING_IDLE;

                printk("SWING END\n");
            }

            break;
        }
    }
}
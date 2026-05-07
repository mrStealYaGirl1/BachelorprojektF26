// imu_processing.c - upgraded version

#include "imu_processing.h"
#include "../swing_manager/swing_manager.h"

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
#define FORWARD_TIMEOUT_SAMPLES    200

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

static float gyro_bias_z = 0;

static uint32_t still_counter = 0;
static uint8_t backswing_confirm = 0;
static uint8_t zero_cross_confirm = 0;
static uint8_t reset_confirm = 0;

static float backswing_peak = 0.0f;

static uint32_t forward_counter = 0;

/* =====================================================
   SWING TIMING
===================================================== */

static swing_timing_t current_swing = {0};
static uint32_t next_swing_id = 1;

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

/* =====================================================
   MAIN PROCESS
===================================================== */

void imu_process_sample(const imu_sample_t *sample, uint32_t sample_idx)
{
    /* ---------- Convert ---------- */

    float ax = (sample->ax / 16384.0f) * 9.81f;
    float ay = (sample->ay / 16384.0f) * 9.81f;
    float az = (sample->az / 16384.0f) * 9.81f;

    float gx = sample->gx * (2000.0f / 32768.0f);
    float gy = sample->gy * (2000.0f / 32768.0f);
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

            if (still_counter > 50)
            {
                memset(&current_swing, 0, sizeof(current_swing));

                current_swing.swing_id = next_swing_id++;

                current_swing.address_start_us = sample->timestamp_us;
                current_swing.address_start_idx = sample_idx;

                swing_state = SWING_ADDRESS;
                still_counter = 0;

                printk("ADDRESS\n");
            }

            break;
        }

        /* ---------------- ADDRESS ---------------- */

        case SWING_ADDRESS:
        {
            if (gz > GZ_BACKSWING_START_DPS)
                backswing_confirm++;
            else
                backswing_confirm = 0;

            if (backswing_confirm >= 3)
            {
                current_swing.backswing_start_us = sample->timestamp_us;
                current_swing.backswing_start_idx = sample_idx;

                swing_state = SWING_BACKSWING;

                backswing_confirm = 0;
                backswing_peak = gz;

                printk("BACKSWING\n");
            }

            break;
        }

        /* ---------------- BACKSWING ---------------- */

        case SWING_BACKSWING:
        {
            if (gz > backswing_peak)
                backswing_peak = gz;

            if (backswing_peak > 40.0f)
            {
                if (gz < 0)
                    zero_cross_confirm++;
                else
                    zero_cross_confirm = 0;

                if (zero_cross_confirm >= SWING_CONFIRM_SAMPLES)
                {
                    current_swing.forward_start_us = sample->timestamp_us;
                    current_swing.forward_start_idx = sample_idx;

                    swing_state = SWING_FORWARD;

                    zero_cross_confirm = 0;
                    forward_counter = 0;

                    printk("FORWARD (peak=%d)\n", (int)(backswing_peak));

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

                swing_manager_notify_impact(sample_idx,
                                            sample->timestamp_us);

                swing_state = SWING_FOLLOW;

                printk("IMPACT → FOLLOW\n");

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
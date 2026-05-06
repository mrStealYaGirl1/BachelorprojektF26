//imu_processing.c - IMU data processing and swing detection

#include "imu_processing.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <string.h>

/* =====================================================
   CONFIG
===================================================== */

#define IMPACT_ENERGY_WINDOW      8
#define IMPACT_THRESHOLD          25.0f
#define IMPACT_COOLDOWN_SAMPLES   200

#define GZ_BACKSWING_START_DPS   10.0f
#define GZ_IDLE_DPS              3.0f

#define IMPACT_ENERGY_WINDOW      8
#define IMPACT_THRESHOLD          25.0f
#define IMPACT_RISE_THRESHOLD     20.0f
#define IMPACT_COOLDOWN_SAMPLES   200
#define IMPACT_FORWARD_GZ_MIN_DPS   -15.0f


/* =====================================================
   ENERGY DETECTION BUFFER
===================================================== */
static float energy_buffer[IMPACT_ENERGY_WINDOW] = {0};
static uint8_t energy_index = 0;

static float energy_sum = 0.0f;
static float prev_energy_sum = 0.0f;

static uint32_t cooldown_counter = 0;

/* =====================================================
   STATE
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

/* =====================================================
   FORWARD DECLARATIONS
===================================================== */

static int detect_impact(float acc_dynamic, float gz);

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

    still_counter = 0;
    swing_state = SWING_IDLE;

    printk("IMU processing ready\n");
}

/* =====================================================
   MAIN PROCESS FUNCTION
===================================================== */

void imu_process_sample(const imu_sample_t *sample, uint32_t sample_idx)
{
    (void)sample_idx;

    /* ---------- Convert to physical units ---------- */

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

            if (still_counter > 50)   // ~250 ms
            {
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
            {
                swing_state = SWING_BACKSWING;
                printk("BACKSWING\n");
            }
            break;
        }

        /* ---------------- BACKSWING ---------------- */

        case SWING_BACKSWING:
        {
            if (gz < 0)
            {
                swing_state = SWING_FORWARD;
                printk("FORWARD\n");
            }
            break;
        }

        /* ---------------- FORWARD ---------------- */

        case SWING_FORWARD:
        {
            if (detect_impact(acc_dynamic, gz))
            {
                swing_state = SWING_FOLLOW;
                break;
            }

            break;
        }

        /* ---------------- FOLLOW ---------------- */

        case SWING_FOLLOW:
        {
            if (gyro_mag < GZ_IDLE_DPS)
            {
                swing_state = SWING_IDLE;
                printk("SWING END\n");
            }
            break;
        }
    }
}

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
        printk("🏌️ IMPACT! E=%.2f dE=%.2f gz=%.2f\n",
               energy_sum, dE, gz);

        cooldown_counter = IMPACT_COOLDOWN_SAMPLES;
        return 1;
    }

    return 0;
}
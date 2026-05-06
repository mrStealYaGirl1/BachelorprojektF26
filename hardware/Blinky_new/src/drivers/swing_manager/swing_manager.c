#include "swing_manager.h"
#include "imu/imu_driver.h"
#include "ble/ble_driver.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define PRE_SAMPLES   600   // 3 sek
#define POST_SAMPLES  400   // 2 sek
#define EVENT_SIZE    (PRE_SAMPLES + POST_SAMPLES)

typedef enum {
    STATE_WAIT,
    STATE_CAPTURE_POST,
    STATE_PROCESS,
    STATE_COOLDOWN
} swing_state_t;

static swing_state_t state = STATE_WAIT;

static imu_sample_t swing_buffer[EVENT_SIZE];

static uint32_t impact_index = 0;
static uint32_t event_impact_index = 0;
static uint32_t post_counter = 0;
static uint8_t impact_pending = 0;

void swing_manager_notify_impact(uint32_t index)
{
    if (state == STATE_WAIT)
    {
        impact_index = index;
        impact_pending = 1;
    }
}

static void copy_event(void)
{
    imu_ringbuffer_t *rb = imu_get_ringbuffer();

    uint32_t start =
        (event_impact_index + IMU_BUFFER_SIZE - PRE_SAMPLES) % IMU_BUFFER_SIZE;

    for (uint32_t i = 0; i < EVENT_SIZE; i++)
    {
        uint32_t idx = (start + i) % IMU_BUFFER_SIZE;
        swing_buffer[i] = rb->buffer[idx];
    }
}

void swing_manager_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1)
    {
        switch (state)
        {
            case STATE_WAIT:

                if (impact_pending)
                {
                    impact_pending = 0;

                    event_impact_index = impact_index;
                    post_counter = 0;

                    state = STATE_CAPTURE_POST;

                    printk("Impact event started\n");
                }
                break;

            case STATE_CAPTURE_POST:

                post_counter++;

                if (post_counter >= POST_SAMPLES)
                {
                    copy_event();
                    state = STATE_PROCESS;
                }
                break;

            case STATE_PROCESS:
            {
                printk("Sending swing event...\n");

                uint16_t seq = 0;

                for (uint32_t i = 0; i < EVENT_SIZE; i++)
                {
                    ble_send_imu_sample(&swing_buffer[i]);

                    k_msleep(1); // pacing
                }

                printk("Event done\n");

                state = STATE_COOLDOWN;
                break;
            }

            case STATE_COOLDOWN:

                k_msleep(500);
                state = STATE_WAIT;
                break;
        }

        k_msleep(5);
    }
}
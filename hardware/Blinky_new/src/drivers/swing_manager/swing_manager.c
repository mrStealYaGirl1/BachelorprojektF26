#include "swing_manager.h"
#include "../imu/imu_driver.h"
#include "../ble/ble_driver.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>

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

static swing_timing_t current_event_timing;
static uint8_t current_event_timing_valid = 0;


void swing_manager_init(void)
{
    state = STATE_WAIT;

    impact_index = 0;
    event_impact_index = 0;
    post_counter = 0;
    impact_pending = 0;

    memset(&current_event_timing, 0, sizeof(current_event_timing));
    current_event_timing_valid = 0;
}

void swing_manager_notify_impact(uint32_t index, uint64_t impact_us)
{
    if (state == STATE_WAIT)
    {
        impact_index = index;

        current_event_timing.impact_idx = index;
        current_event_timing.impact_us = impact_us;

        current_event_timing_valid = 1;
        impact_pending = 1;
    }
}

void swing_manager_add_swing(swing_timing_t swing)
{
    current_event_timing = swing;
    current_event_timing_valid = 1;
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

    static uint16_t event_id_counter = 0;

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
                    post_counter = 0;
                    copy_event();
                    state = STATE_PROCESS;
                }
                break;

            case STATE_PROCESS:
            {
                printk("Processing swing event...\n");

                uint16_t event_id = ++event_id_counter;

                ble_swing_meta_packet_t meta = {0};

                meta.event_id = event_id;
                meta.packet_type = BLE_PKT_TYPE_META;

                if (current_event_timing_valid)
                {
                    meta.swing_id = current_event_timing.swing_id;

                    meta.address_start_us   = current_event_timing.address_start_us;
                    meta.backswing_start_us = current_event_timing.backswing_start_us;
                    meta.forward_start_us   = current_event_timing.forward_start_us;
                    meta.impact_us          = current_event_timing.impact_us;
                    meta.follow_start_us    = current_event_timing.follow_start_us;
                    meta.end_us             = current_event_timing.end_us;
                }
                else
                {
                    meta.swing_id = event_id;
                    meta.impact_us = swing_buffer[PRE_SAMPLES].timestamp_us;
                }

                meta.sample_rate_hz = IMU_SAMPLE_RATE_HZ;
                meta.total_samples = EVENT_SIZE;

                meta.pre_samples = PRE_SAMPLES;
                meta.post_samples = POST_SAMPLES;
                meta.impact_index_in_event = PRE_SAMPLES;

                meta.event_start_us = swing_buffer[0].timestamp_us;
                meta.event_end_us   = swing_buffer[EVENT_SIZE - 1].timestamp_us;

                printk("Sending meta packet...\n");
                ble_queue_meta_packet(&meta);

                k_msleep(50);

                printk("Sending swing event...\n");

                uint16_t seq = 0;

                for (uint32_t i = 0; i < EVENT_SIZE; i += BLE_SAMPLES_PER_PKT)
                {
                    ble_imu_packet_t pkt = {0};

                    pkt.event_id = event_id;
                    pkt.packet_type = BLE_PKT_TYPE_IMU;
                    pkt.sample_count = 0;

                    for (uint32_t j = 0; j < BLE_SAMPLES_PER_PKT && (i + j) < EVENT_SIZE; j++)
                    {
                        uint32_t idx = i + j;

                        pkt.samples[j].ax = swing_buffer[idx].ax;
                        pkt.samples[j].ay = swing_buffer[idx].ay;
                        pkt.samples[j].az = swing_buffer[idx].az;

                        pkt.samples[j].gx = swing_buffer[idx].gx;
                        pkt.samples[j].gy = swing_buffer[idx].gy;
                        pkt.samples[j].gz = swing_buffer[idx].gz;

                        pkt.samples[j].ts_us = (uint32_t)swing_buffer[idx].timestamp_us;
                        pkt.samples[j].seq = seq++;

                        pkt.sample_count++;
                    }

                    if (!ble_queue_imu_packet(&pkt))
                    {
                        printk("IMU packet queue failed: event=%u seq_start=%u\n",
                            event_id,
                            pkt.samples[0].seq);
                    }
                    else
                    {
                        k_msleep(2);
                    }
                }
                printk("Event sent\n");

                memset(&current_event_timing, 0, sizeof(current_event_timing));
                current_event_timing_valid = 0;

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
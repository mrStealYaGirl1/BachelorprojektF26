#include "swing_manager.h"
#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ble_manager.h"
#include "esp_timer.h"
#include <limits.h>

static const char *TAG = "SWING";

#define BLE_DEBUG_PIN GPIO_NUM_4

/* 200 Hz */
#define PRE_SAMPLES    600   // 3 sek
#define POST_SAMPLES   400    // 2 sek
#define EVENT_SIZE     (PRE_SAMPLES + POST_SAMPLES)

typedef enum {
    STATE_WAIT,
    STATE_CAPTURE_POST,
    STATE_PROCESS,
    STATE_COOLDOWN
} swing_state_t;

static swing_state_t state = STATE_WAIT;

static imu_sample_t swing_buffer[EVENT_SIZE];

static uint32_t event_impact_index = 0;
static uint32_t impact_index = 0;
static uint32_t post_counter = 0;
static uint32_t cooldown_counter = 0;
static uint8_t impact_pending = 0;
static swing_timing_t current_event_timing;
static uint8_t current_event_timing_valid = 0;

void swing_manager_notify_impact(uint32_t index)
{
    if (state == STATE_WAIT)
    {
        impact_index = index;
        impact_pending = 1;
    }
}

void swing_manager_add_swing(swing_timing_t swing)
{
    current_event_timing = swing;
    current_event_timing_valid = 1;
}


static void copy_event_from_ringbuffer(void)
{
    imu_ringbuffer_t *rb = imu_get_ringbuffer();

    uint32_t start =
    (   event_impact_index + IMU_BUFFER_SIZE - PRE_SAMPLES) % IMU_BUFFER_SIZE;

    for (uint32_t i = 0; i < EVENT_SIZE; i++)
    {
        uint32_t idx = (start + i) % IMU_BUFFER_SIZE;
        swing_buffer[i] = rb->buffer[idx];
    }
}


void swing_manager_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        switch (state)
        {

            case STATE_WAIT:

                if (impact_pending)
                {
                    impact_pending = 0;

                    event_impact_index = impact_index;   // lås index til dette event

                    post_counter = 0;
                    cooldown_counter = 0;

                    state = STATE_CAPTURE_POST;

                    ESP_LOGI(TAG, "Impact event started");
                    //ESP_LOGI(TAG, "Impact index: %lu", (unsigned long)event_impact_index);
                }

                break;


            case STATE_CAPTURE_POST:

                post_counter++;

            if (post_counter >= POST_SAMPLES)
            {
                post_counter = 0;
                copy_event_from_ringbuffer();
                state = STATE_PROCESS;
            }

                break;

            case STATE_PROCESS:
            {
                ESP_LOGI(TAG, "Processing swing event");

                float max_acc_energy = 0.0f;

                static uint16_t s_event_id = 0;
                const uint16_t event_id = ++s_event_id;

                // for (uint32_t i = 0; i < EVENT_SIZE; i++)
                // {
                //     float ax = swing_buffer[i].ax;
                //     float ay = swing_buffer[i].ay;
                //     float az = swing_buffer[i].az;

                //     float mag = ax * ax + ay * ay + az * az;

                //     if (mag > max_acc_energy)
                //     {
                //         max_acc_energy = mag;
                //     }
                // }

                //ESP_LOGI(TAG, "Max raw acc energy: %.2f", max_acc_energy);
                vTaskDelay(pdMS_TO_TICKS(200));

                // META data                
                ble_swing_meta_pkt_t meta;
                memset(&meta, 0, sizeof(meta));

                meta.event_id = event_id;
                meta.packet_type = BLE_PKT_TYPE_META;

                meta.swing_id = current_event_timing.swing_id;
                meta.sample_rate_hz = IMU_SAMPLE_RATE_HZ;
                meta.total_samples = EVENT_SIZE;

                meta.pre_samples = PRE_SAMPLES;
                meta.post_samples = POST_SAMPLES;
                meta.impact_index_in_event = PRE_SAMPLES;

                meta.address_start_us   = current_event_timing.address_start_us;
                meta.backswing_start_us = current_event_timing.backswing_start_us;
                meta.forward_start_us   = current_event_timing.forward_start_us;
                meta.impact_us          = current_event_timing.impact_us;
                meta.follow_start_us    = current_event_timing.follow_start_us;
                meta.end_us             = current_event_timing.end_us;

                meta.event_start_us = swing_buffer[0].timestamp_us;
                meta.event_end_us   = swing_buffer[EVENT_SIZE - 1].timestamp_us;

                gpio_set_level(BLE_DEBUG_PIN, 1);

                if (current_event_timing_valid)
                {
                    bool meta_queued = ble_manager_send_swing_meta(&meta);
                    if (!meta_queued)
                    {
                        ESP_LOGW(TAG, "Failed to queue META packet event=%u", event_id);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "META packet queued for event %u", event_id);
                    }
                    
                    // int meta_rc = ble_manager_notify_swing_meta_rc(&meta);
                    // if (meta_rc != 0)
                    // {
                    //     ESP_LOGW(TAG, "Failed to send META packet rc=%d event=%u", meta_rc, event_id);
                    // }
                    // else
                    // {
                    //     ESP_LOGI(TAG, "META packet sent for event %u", event_id);
                    // }

                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                else
                {
                    ESP_LOGW(TAG, "No swing timing metadata available for event %u", event_id);
                }


                // // IMU data                
                // for (uint32_t i = 0; i < 10 && i < EVENT_SIZE; i++)
                //                 {
                //                     ESP_LOGI(TAG,
                //                     "RAW[%lu] ax=%d ay=%d az=%d gx=%d gy=%d gz=%d",
                //                     (unsigned long)i,
                //                     (int)swing_buffer[i].ax,
                //                     (int)swing_buffer[i].ay,
                //                     (int)swing_buffer[i].az,
                //                     (int)swing_buffer[i].gx,
                //                     (int)swing_buffer[i].gy,
                //                     (int)swing_buffer[i].gz);
                //                 }

                int64_t t0 = swing_buffer[0].timestamp_us;
                int64_t tImpact = swing_buffer[PRE_SAMPLES].timestamp_us;
                int64_t tEnd = swing_buffer[EVENT_SIZE-1].timestamp_us;

                //ESP_LOGI(TAG, "write_index now: %u", imu_get_ringbuffer()->write_index);
                //ESP_LOGI(TAG, "event_impact_index: %u", event_impact_index);
                //ESP_LOGI(TAG, "start index: %u",
                        //(event_impact_index + IMU_BUFFER_SIZE - PRE_SAMPLES) % IMU_BUFFER_SIZE);                

                ESP_LOGI(TAG, "Pre duration:  %.3f sec", (tImpact - t0) / 1000000.0);
                ESP_LOGI(TAG, "Post duration: %.3f sec", (tEnd - tImpact) / 1000000.0);
                ESP_LOGI(TAG, "Total duration: %.3f sec", (tEnd - t0) / 1000000.0);

                //  BLE streaming af event data + metadata
                

                


                 /* -------- BLE STREAMING - IMU data -------- */

                

                uint16_t seq = 0;

                int64_t ble_start_us = esp_timer_get_time();
                uint32_t packets_queued = 0;
                uint32_t queue_drop_count = 0;

                 /* overflow counters for this event */
                uint32_t overflow_acc_count = 0;
                uint32_t overflow_gyro_count = 0;

                /* nulstil BLE notify-statistik for dette event */
                ble_manager_reset_notify_stats();

                 for (uint32_t i = 0; i < EVENT_SIZE; i += BLE_IMU_SAMPLES_PER_PKT)
                {
                    ble_imu_pkt_t pkt;
                    pkt.event_id = event_id;
                    pkt.packet_type = BLE_PKT_TYPE_IMU;
                    pkt.sample_count = 0;

                    /* nulstil resten for pænhed */
                    for (uint32_t k = 0; k < BLE_IMU_SAMPLES_PER_PKT; k++)
                    {
                        pkt.samples[k].ax = 0;
                        pkt.samples[k].ay = 0;
                        pkt.samples[k].az = 0;
                        pkt.samples[k].gx = 0;
                        pkt.samples[k].gy = 0;
                        pkt.samples[k].gz = 0;
                        pkt.samples[k].ts_ms = 0;
                        pkt.samples[k].seq = 0;
                    }

                    for (uint32_t j = 0; j < BLE_IMU_SAMPLES_PER_PKT && (i + j) < EVENT_SIZE; j++)
                    {
                        uint32_t idx = i + j;

                        int32_t ax_raw = (int32_t)swing_buffer[idx].ax;
                        int32_t ay_raw = (int32_t)swing_buffer[idx].ay;
                        int32_t az_raw = (int32_t)swing_buffer[idx].az;

                        int32_t gx_raw = (int32_t)swing_buffer[idx].gx;
                        int32_t gy_raw = (int32_t)swing_buffer[idx].gy;
                        int32_t gz_raw = (int32_t)swing_buffer[idx].gz;

                        if (ax_raw > INT16_MAX || ax_raw < INT16_MIN ||
                            ay_raw > INT16_MAX || ay_raw < INT16_MIN ||
                            az_raw > INT16_MAX || az_raw < INT16_MIN)
                        {
                            overflow_acc_count++;
                        }

                        if (gx_raw > INT16_MAX || gx_raw < INT16_MIN ||
                            gy_raw > INT16_MAX || gy_raw < INT16_MIN ||
                            gz_raw > INT16_MAX || gz_raw < INT16_MIN)
                        {
                            overflow_gyro_count++;
                        }

                        pkt.samples[j].ax = (int16_t)ax_raw;
                        pkt.samples[j].ay = (int16_t)ay_raw;
                        pkt.samples[j].az = (int16_t)az_raw;

                        pkt.samples[j].gx = (int16_t)gx_raw;
                        pkt.samples[j].gy = (int16_t)gy_raw;
                        pkt.samples[j].gz = (int16_t)gz_raw;

                        pkt.samples[j].ts_ms = (uint32_t)(swing_buffer[idx].timestamp_us / 1000ULL);
                        pkt.samples[j].seq = seq++;

                        pkt.sample_count++;
                    }
                    
                    // Check returværdi fra ble_manager_send_imu_pkt
                    bool queued = ble_manager_send_imu_pkt(&pkt);
                    if (!queued)
                    {
                        queue_drop_count++;
                        vTaskDelay(pdMS_TO_TICKS(10));
                        ESP_LOGW(TAG,
                                 "IMU pkt dropped before queue: event=%u seq_start=%u",
                                 event_id,
                                 pkt.samples[0].seq);
                    }
                    else
                    {
                        packets_queued++;
                         vTaskDelay(pdMS_TO_TICKS(2));   // lille pause mellem pakker
                    }

                    /* Ingen delay her – pacing styres i BLE TX-tasken */
                    //vTaskDelay(pdMS_TO_TICKS(2));
                                  
                }



                
                
                /* vent til BLE TX-task faktisk er færdig med at sende */
                while (ble_manager_is_imu_tx_busy())
                {
                    vTaskDelay(pdMS_TO_TICKS(5));
                }

                gpio_set_level(BLE_DEBUG_PIN, 0);
                
                ESP_LOGI(TAG, "Swing event sent over BLE");
                
                int64_t ble_end_us = esp_timer_get_time();
                float ble_time_sec = (ble_end_us - ble_start_us) / 1000000.0f;
                float packets_per_sec = (ble_time_sec > 0.0f) ? (packets_queued / ble_time_sec) : 0.0f;
                float data_rate_kB = (ble_time_sec > 0.0f)
                    ? ((packets_queued * sizeof(ble_imu_pkt_t)) / 1024.0f / ble_time_sec)
                    : 0.0f;

                ESP_LOGI(TAG, "BLE queueing finished");
                ESP_LOGI(TAG, "Packets queued: %lu", (unsigned long)packets_queued);
                ESP_LOGI(TAG, "Queue drops: %lu", (unsigned long)queue_drop_count);
                ESP_LOGI(TAG, "Queueing time: %.3f sec", ble_time_sec);
                ESP_LOGI(TAG, "Queue rate: %.1f packets/sec", packets_per_sec);
                ESP_LOGI(TAG, "Estimated queue throughput: %.2f kB/s", data_rate_kB);

                ESP_LOGI(TAG, "Packet size: %d bytes", (int)sizeof(ble_imu_pkt_t));
                ESP_LOGI(TAG, "Total queued data: %.2f kB",
                         (packets_queued * sizeof(ble_imu_pkt_t)) / 1024.0f);

                ESP_LOGI(TAG, "ACC overflow count: %lu", (unsigned long)overflow_acc_count);
                ESP_LOGI(TAG, "GYRO overflow count: %lu", (unsigned long)overflow_gyro_count);

                cooldown_counter = 0;
                state = STATE_COOLDOWN;
                break;
            }


            case STATE_COOLDOWN:

                cooldown_counter++;

                if (cooldown_counter >= 200) // 1 sekund
                {
                    state = STATE_WAIT;
                }

                break;
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5)); // 200 Hz
    }
}
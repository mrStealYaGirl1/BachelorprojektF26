#include "swing_manager.h"
#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ble_manager.h"
#include "esp_timer.h"

static const char *TAG = "SWING";

/* 200 Hz */
#define PRE_SAMPLES    1000   // 5 sek
#define POST_SAMPLES   600    // 3 sek
#define EVENT_SIZE     (PRE_SAMPLES + POST_SAMPLES)

#define SAMPLES_PER_PACKET 10

typedef enum {
    STATE_WAIT,
    STATE_CAPTURE_POST,
    STATE_PROCESS,
    STATE_COOLDOWN
} swing_state_t;

static swing_state_t state = STATE_WAIT;

static imu_sample_t swing_buffer[EVENT_SIZE];

static uint32_t impact_index = 0;
static uint32_t post_counter = 0;
static uint32_t cooldown_counter = 0;
static uint8_t impact_pending = 0;


void swing_manager_notify_impact(uint32_t index)
{
    if (state == STATE_WAIT)
    {
        impact_index = index;
        impact_pending = 1;
    }
}


static void copy_event_from_ringbuffer(void)
{
    imu_ringbuffer_t *rb = imu_get_ringbuffer();

    uint32_t start =
        (impact_index + IMU_BUFFER_SIZE - PRE_SAMPLES) % IMU_BUFFER_SIZE;

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
                    post_counter = 0;
                    state = STATE_CAPTURE_POST;

                    ESP_LOGI(TAG, "Impact event started");
                }

                break;


            case STATE_CAPTURE_POST:

                post_counter++;

                if (post_counter >= POST_SAMPLES)
                {
                    copy_event_from_ringbuffer();
                    state = STATE_PROCESS;
                }

                break;


            case STATE_PROCESS:
            {
                ESP_LOGI(TAG, "Processing swing event");

                float max_acc = 0;

                for (uint32_t i = 0; i < EVENT_SIZE; i++)
                {
                    float ax = swing_buffer[i].ax;
                    float ay = swing_buffer[i].ay;
                    float az = swing_buffer[i].az;

                    float mag = ax*ax + ay*ay + az*az;

                    if (mag > max_acc)
                        max_acc = mag;
                }

                ESP_LOGI(TAG, "Max raw acc energy: %.2f", max_acc);

                int64_t t0 = swing_buffer[0].timestamp_us;
                int64_t tImpact = swing_buffer[PRE_SAMPLES].timestamp_us;
                int64_t tEnd = swing_buffer[EVENT_SIZE-1].timestamp_us;

                ESP_LOGI(TAG, "Pre duration:  %.3f sec", (tImpact - t0) / 1000000.0);
                ESP_LOGI(TAG, "Post duration: %.3f sec", (tEnd - tImpact) / 1000000.0);
                ESP_LOGI(TAG, "Total duration: %.3f sec", (tEnd - t0) / 1000000.0);


                 /* -------- BLE STREAMING -------- */

                static uint16_t s_event_id = 0;
                const uint16_t event_id = ++s_event_id;

                uint16_t seq = 0;

                int64_t ble_start_us = esp_timer_get_time();
                uint32_t packets_sent = 0;

                for (uint32_t i = 0; i < EVENT_SIZE; i++)
                {
                    ble_imu_pkt_t pkt;

                    pkt.ax = (int16_t)(swing_buffer[i].ax * 1000);
                    pkt.ay = (int16_t)(swing_buffer[i].ay * 1000);
                    pkt.az = (int16_t)(swing_buffer[i].az * 1000);

                    pkt.gx = (int16_t)(swing_buffer[i].gx * 100);
                    pkt.gy = (int16_t)(swing_buffer[i].gy * 100);
                    pkt.gz = (int16_t)(swing_buffer[i].gz * 100);

                    pkt.ts_ms = (uint32_t)(swing_buffer[i].timestamp_us / 1000ULL);

                    pkt.seq = seq++;
                    pkt.event_id = event_id;

                    /* send via BLE manager queue */
                    (void)ble_manager_send_imu(&pkt);

                    packets_sent++;

                    /* pacing for BLE stability */
                    vTaskDelay(pdMS_TO_TICKS(1));   // change to 2 ms if experiencing BLE congestion
                }

                //ESP_LOGI(TAG, "Swing event sent over BLE");
                int64_t ble_end_us = esp_timer_get_time();

                float ble_time_sec = (ble_end_us - ble_start_us) / 1000000.0f;
                float packets_per_sec = packets_sent / ble_time_sec;
                float data_rate_kB = (packets_sent * sizeof(ble_imu_pkt_t)) / 1024.0f / ble_time_sec;

                ESP_LOGI(TAG, "BLE transfer finished");
                ESP_LOGI(TAG, "Packets sent: %lu", packets_sent);
                ESP_LOGI(TAG, "Transfer time: %.3f sec", ble_time_sec);
                ESP_LOGI(TAG, "Packets/sec: %.1f", packets_per_sec);
                ESP_LOGI(TAG, "Throughput: %.2f kB/s", data_rate_kB);

                ESP_LOGI(TAG, "Packet size: %d bytes", sizeof(ble_imu_pkt_t));
                ESP_LOGI(TAG, "Total data: %.2f kB", (packets_sent * sizeof(ble_imu_pkt_t)) / 1024.0f);

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
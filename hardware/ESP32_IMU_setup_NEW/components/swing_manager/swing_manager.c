#include "swing_manager.h"
#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ble_manager.h"

static const char *TAG = "SWING";

/* 200 Hz */
#define PRE_SAMPLES    1000   // 5 sek
#define POST_SAMPLES   600    // 3 sek
#define EVENT_SIZE     (PRE_SAMPLES + POST_SAMPLES)

/* Scaling factors for binary transport */
#define ACC_SCALE   1000.0f   // 0.001g resolution
#define GYRO_SCALE  100.0f    // 0.01 dps resolution

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

/* -------- Binary BLE transport struct -------- */
typedef struct __attribute__((packed)) {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    uint32_t timestamp_us;
} ble_sample_t;

/* -------- Header struct -------- */
typedef struct __attribute__((packed)) {
    uint16_t total_samples;
    uint16_t sample_size;
} ble_header_t;


/* Called from IMU when impact is detected */
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

                /* Allocate transport buffer */
                static ble_sample_t tx_buffer[EVENT_SIZE];

                int64_t t0 = swing_buffer[0].timestamp_us;

                for (uint32_t i = 0; i < EVENT_SIZE; i++)
                {
                    /* Scale float -> int16 */
                    tx_buffer[i].ax = (int16_t)(swing_buffer[i].ax * ACC_SCALE);
                    tx_buffer[i].ay = (int16_t)(swing_buffer[i].ay * ACC_SCALE);
                    tx_buffer[i].az = (int16_t)(swing_buffer[i].az * ACC_SCALE);

                    tx_buffer[i].gx = (int16_t)(swing_buffer[i].gx * GYRO_SCALE);
                    tx_buffer[i].gy = (int16_t)(swing_buffer[i].gy * GYRO_SCALE);
                    tx_buffer[i].gz = (int16_t)(swing_buffer[i].gz * GYRO_SCALE);

                    /* Normalize timestamp to event start */
                    tx_buffer[i].timestamp_us =
                        (uint32_t)(swing_buffer[i].timestamp_us - t0);
                }

                /* Prepare header */
                ble_header_t header;
                header.total_samples = EVENT_SIZE;
                header.sample_size   = sizeof(ble_sample_t);

                ESP_LOGI(TAG, "Sending swing event: %u samples (%u bytes each)",
                         header.total_samples, header.sample_size);

                /* Send header first */
                ble_manager_send((uint8_t*)&header, sizeof(header));

                /* Send binary sample data */
                ble_manager_send((uint8_t*)tx_buffer,
                                 sizeof(ble_sample_t) * EVENT_SIZE);

                ESP_LOGI(TAG, "Swing event sent over BLE");

                cooldown_counter = 0;
                state = STATE_COOLDOWN;
                break;
            }

            case STATE_COOLDOWN:
                cooldown_counter++;
                if (cooldown_counter >= 200) // 1 second cooldown
                {
                    state = STATE_WAIT;
                }
                break;
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5)); // 200 Hz
    }
}
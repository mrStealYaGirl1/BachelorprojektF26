#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "IMU";

void imu_init(void)
{
    ESP_LOGI(TAG, "IMU init (stub)");
}

void imu_task(void *pvParameters)
{
    ESP_LOGI(TAG, "IMU task started");

    while (1)
    {
        ESP_LOGI(TAG, "IMU running");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
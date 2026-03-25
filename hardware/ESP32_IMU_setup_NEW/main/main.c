#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "imu.h"
#include "swing_manager.h"
#include "ble_manager.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-FW starter");

    ble_manager_init();     // starter NimBLE + advertising (connectable)
    ble_manager_start_imu_tx_task(); // starter intern task + queue til at sende IMU data via BLE
    
    imu_init();             // starter IMU + swing detection

    xTaskCreate(
        imu_task,
        "imu_task",
        4096,
        NULL,
        5,
        NULL
    );

    xTaskCreate(
        swing_manager_task,
        "swing_task",
        4096,
        NULL,
        5,
        NULL
    );

    while (1)
    {
        ESP_LOGI(TAG, "System kører");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
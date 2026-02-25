#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "imu.h"
#include "swing_manager.h"
#include "ble_manager.h"


static const char *TAG = "BLE_TEST";

static void ble_test_task(void *arg)
{
    // Vent lidt så NimBLE når at sync'e
    vTaskDelay(pdMS_TO_TICKS(1500));

    while (1) {
        ESP_LOGI(TAG, "Triggering BLE burst...");
        ble_manager_send_burst(12.34f, 0.56f, 3000); // 3 sek (nemt at fange)
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    ESP_LOGI("MAIN", "ESP32-FW starter");

    ble_manager_init();

    // Start test burst task
    xTaskCreate(ble_test_task, "ble_test", 4096, NULL, 5, NULL);

    imu_init();
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
    xTaskCreate(swing_manager_task, "swing_task", 4096, NULL, 5, NULL);

    while (1) {
        ESP_LOGI("MAIN", "System kører");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// static const char *TAG = "MAIN";

// void app_main(void)
// {
//     ESP_LOGI(TAG, "ESP32-FW starter");

//     // Initialiser NimBLE
//     ble_manager_init();

//     imu_init();

//     xTaskCreate(
//         imu_task,
//         "imu_task",
//         4096,
//         NULL,
//         5,
//         NULL
//     );

//     xTaskCreate(
//         swing_manager_task,
//         "swing_task",
//         4096,
//         NULL,
//         5,
//         NULL
//     );

//     while (1)
//     {
//         ESP_LOGI(TAG, "System kører");
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }
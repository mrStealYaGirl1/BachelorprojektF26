// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"

// #include "imu.h"
// #include "swing_manager.h"
// #include "ble_manager.h"

// static const char *TAG = "MAIN";

// void app_main(void)
// {
//     ESP_LOGI(TAG, "ESP32-FW starter");

//     ble_manager_init();     // starter NimBLE + advertising (connectable)
//     ble_manager_start_tx_task(); // starter intern task + queue til at sende IMU data via BLE
    
//     imu_init();             // starter IMU + swing detection

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
//         //ESP_LOGI(TAG, "System kører");
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }



/******************************************************************************************************/
/* test main - for analyzing IMU-data for drifting/drift */
/******************************************************************************************************/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "imu.h"

static const char *TAG = "MAIN";

// Hvis denne skal bruges - husk at fjerne ESP_LOGI'er i imu_init og imu_calibrate for at undgå at forstyrre CSV output

void app_main(void)
{
    ESP_LOGI(TAG, "Starter IMU drift-test");

    imu_init();

    xTaskCreate(
        imu_csv_logger_task,
        "imu_csv_logger_task",
        4096,
        NULL,
        5,
        NULL
    );
}




/******************************************************************************************************/
/* Memory usage logging and stack high water mark checks - used for debugging or optimization purposes.*/
/******************************************************************************************************/
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "esp_heap_caps.h"

// #include "imu.h"
// #include "swing_manager.h"
// #include "ble_manager.h"

// static const char *TAG = "MAIN";
// static const char *MEM_TAG = "MEM";

// static TaskHandle_t imu_task_handle = NULL;
// static TaskHandle_t swing_task_handle = NULL;

// static void print_mem(const char *place)
// {
//     ESP_LOGI(MEM_TAG, "[%s] free heap: %u bytes",
//              place,
//              (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));

//     ESP_LOGI(MEM_TAG, "[%s] min free heap: %u bytes",
//              place,
//              (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
// }

// static void print_stack_usage(const char *place)
// {
//     UBaseType_t main_hwm = uxTaskGetStackHighWaterMark(NULL);

//     ESP_LOGI("STACK", "[%s] main_task min free stack: %u words (%u bytes)",
//              place,
//              (unsigned)main_hwm,
//              (unsigned)(main_hwm * sizeof(StackType_t)));

//     if (imu_task_handle != NULL) {
//         UBaseType_t imu_hwm = uxTaskGetStackHighWaterMark(imu_task_handle);
//         ESP_LOGI("STACK", "[%s] imu_task min free stack: %u words (%u bytes)",
//                  place,
//                  (unsigned)imu_hwm,
//                  (unsigned)(imu_hwm * sizeof(StackType_t)));
//     }

//     if (swing_task_handle != NULL) {
//         UBaseType_t swing_hwm = uxTaskGetStackHighWaterMark(swing_task_handle);
//         ESP_LOGI("STACK", "[%s] swing_task min free stack: %u words (%u bytes)",
//                  place,
//                  (unsigned)swing_hwm,
//                  (unsigned)(swing_hwm * sizeof(StackType_t)));
//     }
// }

// void app_main(void)
// {
//     ESP_LOGI(TAG, "ESP32-FW starter");
//     print_mem("startup");

//     ble_manager_init();
//     print_mem("after_ble_manager_init");

//     ble_manager_start_imu_tx_task();
//     print_mem("after_ble_tx_task_start");

//     imu_init();
//     print_mem("after_imu_init");

//     xTaskCreate(
//         imu_task,
//         "imu_task",
//         4096,
//         NULL,
//         5,
//         &imu_task_handle
//     );
//     print_mem("after_imu_task_create");

//     xTaskCreate(
//         swing_manager_task,
//         "swing_task",
//         4096,
//         NULL,
//         5,
//         &swing_task_handle
//     );
//     print_mem("after_swing_task_create");
//     print_stack_usage("after_task_create");

//     while (1)
//     {
//         ESP_LOGI(TAG, "System kører");
//         print_mem("runtime");
//         print_stack_usage("runtime");
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }
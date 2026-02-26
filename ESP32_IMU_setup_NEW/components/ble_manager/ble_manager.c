#include "ble_manager.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "BLE_MANAGER";

static bool ble_initialized = false;
static volatile bool ble_synced = false;
static uint8_t own_addr_type;

/* ---------------- GAP EVENT HANDLER ---------------- */

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising complete");
            break;

        default:
            break;
    }
    return 0;
}

/* ---------------- SYNC CALLBACK ---------------- */

static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_synced = true;
    ESP_LOGI(TAG, "BLE synced (addr_type=%u)", own_addr_type);
}

/* ---------------- NIMBLE TASK ---------------- */

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ---------------- INIT ---------------- */

void ble_manager_init(void)
{
    if (ble_initialized)
        return;

    // Robust NVS init
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    nimble_port_init();

    // Init GAP service (device name lives here)
    ble_svc_gap_init();
    ble_svc_gap_device_name_set("GOLF_IMU");

    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(nimble_host_task);

    ble_initialized = true;
}

/* ---------------- HELPER: WAIT FOR SYNC ---------------- */

static bool wait_for_sync(uint32_t timeout_ms)
{
    if (ble_synced) return true;

    const TickType_t start = xTaskGetTickCount();
    while (!ble_synced) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (timeout_ms > 0) {
            TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= pdMS_TO_TICKS(timeout_ms)) {
                return false;
            }
        }
    }
    return true;
}

/* ---------------- BURST ADVERTISING ---------------- */

void ble_manager_send_burst(float peak, float tempo, uint32_t duration_ms)
{
    if (!ble_initialized)
        ble_manager_init();

    // Make sure sync happened so own_addr_type is valid
    if (!wait_for_sync(1000)) {
        ESP_LOGW(TAG, "BLE not synced yet - skip burst");
        return;
    }

    // Stop any previous advertising (safe even if not advertising)
    ble_gap_adv_stop();

    // Manufacturer payload: 2 bytes company ID + text
    uint8_t mfg[2 + 18]; // 2 bytes ID + up to ~18 bytes text
    mfg[0] = 0xFF;       // Company ID LSB (test value)
    mfg[1] = 0xFF;       // Company ID MSB (test value)
    snprintf((char *)&mfg[2], sizeof(mfg) - 2, "P=%.2f,T=%.2f", peak, tempo);

    /* ---------------- PRIMARY ADVERTISING ---------------- */
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Device name in primary advertising
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = (uint8_t)strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_adv_set_fields rc=%d", rc);
        return;
    }

    /* ---------------- SCAN RESPONSE ---------------- */
    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));

    rsp_fields.mfg_data = mfg;
    rsp_fields.mfg_data_len = 2 + (uint8_t)strlen((char *)&mfg[2]);

    int rc2 = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc2 != 0) {
        ESP_LOGW(TAG, "ble_gap_adv_rsp_set_fields rc=%d", rc2);
        return;
    }

    /* ---------------- ADVERTISING PARAMETERS ---------------- */
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.itvl_min = 0x20;  // 0x20 * 0.625ms = 20 ms
    adv_params.itvl_max = 0x40;  // 0x40 * 0.625ms = 40 ms
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON; // non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    /* ---------------- START ADVERTISING ---------------- */
    rc = ble_gap_adv_start(
        own_addr_type,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        gap_event,
        NULL);

    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_adv_start rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising burst started: %s", (char *)&mfg[2]);

    // Run burst for requested duration
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // Stop advertising
    ble_gap_adv_stop();
    ESP_LOGI(TAG, "Advertising burst stopped");
}


// #include "ble_manager.h"

// #include "esp_log.h"
// #include "nvs_flash.h"

// #include "nimble/nimble_port.h"
// #include "nimble/nimble_port_freertos.h"
// #include "host/ble_hs.h"
// #include "host/ble_gap.h"
// #include "host/ble_uuid.h"
// #include "services/gap/ble_svc_gap.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include <stdio.h>
// #include <string.h>

// static const char *TAG = "BLE_MANAGER";

// static bool ble_initialized = false;
// static uint8_t own_addr_type;

// /* ---------------- GAP EVENT HANDLER ---------------- */

// static int gap_event(struct ble_gap_event *event, void *arg)
// {
//     switch (event->type)
//     {
//         case BLE_GAP_EVENT_ADV_COMPLETE:
//             ESP_LOGI(TAG, "Advertising complete");
//             break;

//         default:
//             break;
//     }
//     return 0;
// }

// /* ---------------- SYNC CALLBACK ---------------- */

// static void on_sync(void)
// {
//     ble_hs_id_infer_auto(0, &own_addr_type);
//     ESP_LOGI(TAG, "BLE synced");
// }

// /* ---------------- NIMBLE TASK ---------------- */

// static void nimble_host_task(void *param)
// {
//     nimble_port_run();
//     nimble_port_freertos_deinit();
// }

// /* ---------------- INIT ---------------- */

// void ble_manager_init(void)
// {
//     if (ble_initialized)
//         return;

//     ESP_ERROR_CHECK(nvs_flash_init());

//     nimble_port_init();

//     // Init GAP + GATT services
//     ble_svc_gap_init();

//     // 👉 Sæt device name HER
//     ble_svc_gap_device_name_set("GOLF_IMU");

//     ble_hs_cfg.sync_cb = on_sync;

//     nimble_port_freertos_init(nimble_host_task);

//     ble_initialized = true;
// }

// /* ---------------- BURST ADVERTISING ---------------- */

// void ble_manager_send_burst(float peak, float tempo, uint32_t duration_ms)
// {
//     if (!ble_initialized)
//         ble_manager_init();

//     char payload[20];
//     snprintf(payload, sizeof(payload), "P=%.2f,T=%.2f", peak, tempo);

//     /* ---------------- PRIMARY ADVERTISING ---------------- */
//     struct ble_hs_adv_fields fields;
//     memset(&fields, 0, sizeof(fields));

//     fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

//     /* Device name in primary advertising */
//     const char *name = ble_svc_gap_device_name();
//     fields.name = (uint8_t *)name;
//     fields.name_len = strlen(name);
//     fields.name_is_complete = 1;

//     int rc = ble_gap_adv_set_fields(&fields);
//     ESP_LOGI(TAG, "ADV SET rc=%d", rc);

//     /* ---------------- SCAN RESPONSE ---------------- */
//     struct ble_hs_adv_fields rsp_fields;
//     memset(&rsp_fields, 0, sizeof(rsp_fields));

//     rsp_fields.mfg_data = (uint8_t *)payload;
//     rsp_fields.mfg_data_len = strlen(payload);

//     int rc2 = ble_gap_adv_rsp_set_fields(&rsp_fields);
//     ESP_LOGI(TAG, "RSP SET rc=%d", rc2);

//     /* ---------------- ADVERTISING PARAMETERS ---------------- */
//     struct ble_gap_adv_params adv_params;
//     memset(&adv_params, 0, sizeof(adv_params));

//     adv_params.itvl_min = 0x20;  // ~20 ms
//     adv_params.itvl_max = 0x40;  // ~40 ms
//     adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
//     adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

//     /* ---------------- START ADVERTISING ---------------- */
//     rc = ble_gap_adv_start(
//         own_addr_type,
//         NULL,
//         BLE_HS_FOREVER,
//         &adv_params,
//         gap_event,
//         NULL);

//     ESP_LOGI(TAG, "Advertising burst started (rc=%d)", rc);

//     /* Run burst for requested duration */
//     vTaskDelay(pdMS_TO_TICKS(duration_ms));

//     /* ---------------- STOP ADVERTISING ---------------- */
//     ble_gap_adv_stop();
//     ESP_LOGI(TAG, "Advertising burst stopped");
// }







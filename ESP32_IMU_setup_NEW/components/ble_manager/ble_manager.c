#include "ble_manager.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "BLE_MANAGER";

static bool ble_initialized = false;
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
    ESP_LOGI(TAG, "BLE synced");
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

    ESP_ERROR_CHECK(nvs_flash_init());

    nimble_port_init();

    // Init GAP + GATT services
    ble_svc_gap_init();

    // 👉 Sæt device name HER
    ble_svc_gap_device_name_set("GOLF_IMU");

    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(nimble_host_task);

    ble_initialized = true;
}

/* ---------------- BURST ADVERTISING ---------------- */

void ble_manager_send_burst(float peak, float tempo, uint32_t duration_ms)
{
    if (!ble_initialized)
        ble_manager_init();

    char payload[20];
    snprintf(payload, sizeof(payload), "P=%.2f,T=%.2f", peak, tempo);

    /* ---------------- PRIMARY ADVERTISING ---------------- */
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Device name in primary advertising */
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    ESP_LOGI(TAG, "ADV SET rc=%d", rc);

    /* ---------------- SCAN RESPONSE ---------------- */
    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));

    rsp_fields.mfg_data = (uint8_t *)payload;
    rsp_fields.mfg_data_len = strlen(payload);

    int rc2 = ble_gap_adv_rsp_set_fields(&rsp_fields);
    ESP_LOGI(TAG, "RSP SET rc=%d", rc2);

    /* ---------------- ADVERTISING PARAMETERS ---------------- */
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.itvl_min = 0x20;  // ~20 ms
    adv_params.itvl_max = 0x40;  // ~40 ms
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    /* ---------------- START ADVERTISING ---------------- */
    rc = ble_gap_adv_start(
        own_addr_type,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        gap_event,
        NULL);

    ESP_LOGI(TAG, "Advertising burst started (rc=%d)", rc);

    /* Run burst for requested duration */
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    /* ---------------- STOP ADVERTISING ---------------- */
    ble_gap_adv_stop();
    ESP_LOGI(TAG, "Advertising burst stopped");
}
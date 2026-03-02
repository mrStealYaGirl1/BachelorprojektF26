// components/ble_manager/ble_manager.c

#include "ble_manager.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "BLE_MANAGER";

static bool ble_initialized = false;
static volatile bool ble_synced = false;
static uint8_t own_addr_type;

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_chr_val_handle = 0;
static bool s_notify_enabled = false;

/* =========================================================
   FORWARD DECLARATIONS  (fixer dine compile-fejl)
========================================================= */
static void ble_start_advertising(void);
static int gap_event(struct ble_gap_event *event, void *arg);

static int gatt_svc_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);
static const struct ble_gatt_svc_def gatt_svcs[];

/* =========================================================
   SIMPLE MESSAGE TYPE (notify payload)
========================================================= */
typedef struct {
    uint32_t counter;
    uint32_t timestamp_ms;
} ble_simple_msg_t;

/* =========================================================
   GATT: UUIDs + Services
========================================================= */
// Demo 128-bit UUIDs (lav evt. jeres egne senere)
static const ble_uuid128_t GOLF_SVC_UUID =
    BLE_UUID128_INIT(0x12,0x34,0x56,0x78,0x90,0xab,0xcd,0xef,0xfe,0xdc,0xba,0x09,0x87,0x65,0x43,0x21);

static const ble_uuid128_t GOLF_CHR_UUID =
    BLE_UUID128_INIT(0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99);

static int gatt_svc_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    // Optional: tillad READ så du kan "Read" i nRF Connect
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        ble_simple_msg_t demo = { .counter = 123, .timestamp_ms = 456 };
        int rc = os_mbuf_append(ctxt->om, &demo, sizeof(demo));
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &GOLF_SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &GOLF_CHR_UUID.u,
                .access_cb = gatt_svc_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_chr_val_handle,
            },
            {0}
        }
    },
    {0}
};

/* =========================================================
   GAP EVENT HANDLER
========================================================= */
static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type)
    {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "Connected (handle=%u)", s_conn_handle);
            } else {
                ESP_LOGW(TAG, "Connect failed; status=%d", event->connect.status);
                ble_start_advertising();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            s_notify_enabled = false;
            ble_start_advertising();
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == s_chr_val_handle) {
                s_notify_enabled = event->subscribe.cur_notify;
                ESP_LOGI(TAG, "Subscribe: notify=%d", (int)s_notify_enabled);
            }
            return 0;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising complete");
            ble_start_advertising();
            return 0;

        default:
            return 0;
    }
}

/* =========================================================
   ADVERTISING (connectable)
========================================================= */
static void ble_start_advertising(void)
{
    // Må ikke starte adv før sync + addr_type
    if (!ble_synced) return;

    // Stop tidligere adv hvis den kører
    ble_gap_adv_stop();

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Navn i advertising
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = (uint8_t)strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_adv_set_fields rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.itvl_min = 0x20;              // 20 ms
    adv_params.itvl_max = 0x40;              // 40 ms
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;  // ✅ connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_adv_start rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising (connectable) started");
}

/* =========================================================
   SYNC CALLBACK
========================================================= */
static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_synced = true;
    ESP_LOGI(TAG, "BLE synced (addr_type=%u)", own_addr_type);

    ble_start_advertising();
}

/* =========================================================
   NIMBLE HOST TASK
========================================================= */
static void nimble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* =========================================================
   INIT
========================================================= */
void ble_manager_init(void)
{
    if (ble_initialized) return;

    // Robust NVS init
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    nimble_port_init();

    // Standard GAP/GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_svc_gap_device_name_set("GOLF_IMU");

    // Registrer dine custom services
    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) ESP_LOGE(TAG, "ble_gatts_count_cfg rc=%d", rc);

    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) ESP_LOGE(TAG, "ble_gatts_add_svcs rc=%d", rc);

    // Callbacks
    ble_hs_cfg.sync_cb = on_sync;

    // Start NimBLE task
    nimble_port_freertos_init(nimble_host_task);

    ble_initialized = true;
}

/* =========================================================
   NOTIFY API
========================================================= */
bool ble_manager_notify_simple(uint32_t counter, uint32_t timestamp_ms)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return false;
    if (!s_notify_enabled) return false;
    if (s_chr_val_handle == 0) return false;

    ble_simple_msg_t msg = { .counter = counter, .timestamp_ms = timestamp_ms };

    struct os_mbuf *om = ble_hs_mbuf_from_flat(&msg, sizeof(msg));
    if (!om) return false;

    int rc = ble_gatts_notify_custom(s_conn_handle, s_chr_val_handle, om);
    return (rc == 0);
}

/* =========================================================
   QUEUE-BASED TX TASK (brug den fra IMU)
========================================================= */
static QueueHandle_t s_ble_q = NULL;
static TaskHandle_t s_ble_tx_task_handle = NULL;

static void ble_tx_task(void *arg)
{
    (void)arg;

    ble_simple_msg_t msg;

    while (1) {
        if (xQueueReceive(s_ble_q, &msg, portMAX_DELAY) == pdTRUE) {
            bool ok = ble_manager_notify_simple(msg.counter, msg.timestamp_ms);
            if (!ok) {
                // Ikke connected / notify ikke enabled endnu -> vi dropper bare
                // (kan logges hvis du vil)
                // ESP_LOGD(TAG, "notify skipped (not connected or not subscribed)");
            }
        }
    }
}

void ble_manager_start_tx_task(void)
{
    if (s_ble_q != NULL) return;

    s_ble_q = xQueueCreate(8, sizeof(ble_simple_msg_t));
    configASSERT(s_ble_q);

    xTaskCreate(ble_tx_task, "ble_tx", 4096, NULL, 6, &s_ble_tx_task_handle);
}

bool ble_manager_send_simple(uint32_t counter, uint32_t timestamp_ms)
{
    if (s_ble_q == NULL) return false;

    ble_simple_msg_t msg = { .counter = counter, .timestamp_ms = timestamp_ms };
    return (xQueueSend(s_ble_q, &msg, 0) == pdTRUE);
}


bool ble_manager_notify_imu(const ble_imu_pkt_t *pkt)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return false;
    if (!s_notify_enabled) return false;
    if (s_chr_val_handle == 0) return false;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, sizeof(*pkt));
    if (!om) return false;

    int rc = ble_gatts_notify_custom(s_conn_handle, s_chr_val_handle, om);
    return (rc == 0);
}

static QueueHandle_t s_imu_q = NULL;
static TaskHandle_t s_imu_tx_task = NULL;

static void ble_imu_tx_task(void *arg)
{
    (void)arg;
    ble_imu_pkt_t pkt;

    while (1) {
        if (xQueueReceive(s_imu_q, &pkt, portMAX_DELAY) == pdTRUE) {
            (void)ble_manager_notify_imu(&pkt); // dropper hvis ikke connected/subscribed
        }
    }
}

void ble_manager_start_imu_tx_task(void)
{
    if (s_imu_q) return;

    s_imu_q = xQueueCreate(16, sizeof(ble_imu_pkt_t));
    configASSERT(s_imu_q);

    xTaskCreate(ble_imu_tx_task, "ble_imu_tx", 4096, NULL, 6, &s_imu_tx_task);
}

bool ble_manager_send_imu(const ble_imu_pkt_t *pkt)
{
    if (!s_imu_q) return false;
    return (xQueueSend(s_imu_q, pkt, 0) == pdTRUE);
}
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include "ble_driver.h"


/*
I skal tænke BLE-opstart i to trin:

1. ble_init() starter Bluetooth stacken
2. bt_ready_cb() fortæller, når stacken er klar
3. først derefter laver I resten:
   - notify_attr
   - k_work_init 
   - bt_gatt_cb_register
   - bt_le_adv_start
*/


/* =========================
   GLOBALS
========================= */

static struct bt_conn *current_conn = NULL;
static bool notify_enabled = false;
static bool att_ready = false;
static bool ble_stack_ready = false;

static struct k_work ble_work;
static const struct bt_gatt_attr *notify_attr;
static struct bt_gatt_exchange_params mtu_exchange_params;

static volatile bool ble_tx_busy = false;

/* forward declaration */
static void ble_notify_work(struct k_work *work);

/* =========================
   TEST DATA
========================= */

static ble_imu_packet_t pkt;

/* =========================
   CALLBACKS
========================= */



static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_exchange_params *params)
{
    if (err) {
        printk("MTU exchange failed (%u)\n", err);
        return;
    }

    printk("MTU exchange done. Current MTU: %u\n", bt_gatt_get_mtu(conn));
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("CCC CHANGED: %d\n", notify_enabled);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (%u)\n", err);
        return;
    }

    printk("Connected\n");

    if (current_conn) {
        bt_conn_unref(current_conn);
    }

    current_conn = bt_conn_ref(conn);

    notify_enabled = false;
    att_ready = false;

    mtu_exchange_params.func = mtu_exchange_cb;

    int ret = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
    printk("bt_gatt_exchange_mtu() returned: %d\n", ret);
}


static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (%u)\n", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    notify_enabled = false;
    att_ready = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* =========================
   MTU / ATT READY
========================= */

static void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    printk("MTU updated: TX=%d RX=%d\n", tx, rx);
    att_ready = true;
}

static struct bt_gatt_cb gatt_callbacks = {
    .att_mtu_updated = mtu_updated,
};

/* =========================
   BLE WORKQUEUE
========================= */
static void ble_notify_work(struct k_work *work)
{
    if (!current_conn || !notify_enabled || !att_ready) {
        ble_tx_busy = false;
        return;
    }

    uint16_t mtu = bt_gatt_get_mtu(current_conn);
    uint16_t max_payload = (mtu >= 3) ? (mtu - 3) : 0;
    uint16_t len = sizeof(pkt);

    if (len > max_payload)
    {
        printk("Packet too large: len=%u, max=%u, mtu=%u\n",
               len, max_payload, mtu);
        ble_tx_busy = false;
        return;
    }

    int err = bt_gatt_notify(current_conn,
                             notify_attr,
                             &pkt,
                             len);

    if (err)
    {
        printk("Notify error: %d\n", err);
    }
    else
    {
        if ((pkt.sample.seq % 50) == 0) {
            printk("BLE seq=%d\n", pkt.sample.seq);
        }
    }

    ble_tx_busy = false;
}


/* =========================
   GATT SERVICE
========================= */

BT_GATT_SERVICE_DEFINE(test_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(0xFFF0)),

    BT_GATT_CHARACTERISTIC(
        BT_UUID_DECLARE_16(0xFFF1),
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, NULL
    ),

    BT_GATT_CCC(ccc_cfg_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);


/* =========================
   Bluetooth ready callback and post-init
========================= */

static void bt_ready_cb(int err)
{
    if (err) {
        printk("BLE init failed (%d)\n", err);
        return;
    }

    printk("BLE ready callback\n");
    ble_stack_ready = true;
}

void ble_post_init(void)
{
    notify_attr = &test_svc.attrs[2];
    k_work_init(&ble_work, ble_notify_work);
    bt_gatt_cb_register(&gatt_callbacks);

    static const struct bt_le_adv_param adv_param = {
        .options = BT_LE_ADV_OPT_CONN,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    };

    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS,
            (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    };

    const struct bt_data sd[] = {
        BT_DATA(BT_DATA_NAME_COMPLETE,
                CONFIG_BT_DEVICE_NAME,
                sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };

    int err = bt_le_adv_start(&adv_param,
                              ad, ARRAY_SIZE(ad),
                              sd, ARRAY_SIZE(sd));

    if (err && err != -EALREADY) {
        printk("ADV START failed: %d\n", err);
        return;
    }

    printk("Advertising started\n");
}

/* =========================
   INIT
========================= */

void ble_init(void)
{
    printk("ble_init: before bt_enable async\n");

    int err = bt_enable(bt_ready_cb);

    printk("ble_init: bt_enable returned %d\n", err);

    if (err) {
        printk("bt_enable start failed (%d)\n", err);
    }
}

/* =========================
   TEST NOTIFY (FIXED SIZE!)
========================= */

void ble_send_test(void)
{
    if (!current_conn || !notify_enabled || !att_ready)
    {
        printk("Not ready: conn=%p notify=%d att=%d\n",
               current_conn, notify_enabled, att_ready);
        return;
    }

    static uint16_t seq = 0;

    pkt.event_id = 1;
    pkt.packet_type = 2;
    pkt.sample_count = 1;

    pkt.sample.ax = 100 + seq;
    pkt.sample.ay = 200;
    pkt.sample.az = 300;

    pkt.sample.gx = 10;
    pkt.sample.gy = 20;
    pkt.sample.gz = 30;

    pkt.sample.ts_ms = k_uptime_get_32();
    pkt.sample.seq = seq++;

    /* 🔥 KUN DET HER */
    k_work_submit(&ble_work);
}

static void fill_ble_sample_from_imu(const imu_sample_t *src,
                                     ble_imu_sample_t *dst,
                                     uint16_t seq)
{
    dst->ax = src->ax;
    dst->ay = src->ay;
    dst->az = src->az;

    dst->gx = src->gx;
    dst->gy = src->gy;
    dst->gz = src->gz;

    dst->ts_ms = (uint32_t)src->timestamp_ms;
    dst->seq = seq;
}

// void ble_send_imu_sample(const imu_sample_t *sample)
// {
//     if (sample == NULL)
//     {
//         return;
//     }

//     if (!current_conn || !notify_enabled || !att_ready)
//     {
//         return;
//     }

//     static uint16_t seq = 0;

//     pkt.event_id = 1;
//     pkt.packet_type = 2;
//     pkt.sample_count = 1;

//     fill_ble_sample_from_imu(sample, &pkt.sample, seq++);

//     k_work_submit(&ble_work);
// }
void ble_send_imu_sample(const imu_sample_t *sample)
{
    if (sample == NULL)
    {
        return;
    }

    if (!current_conn || !notify_enabled || !att_ready)
    {
        return;
    }

    if (ble_tx_busy)
    {
        return;
    }

    ble_tx_busy = true;

    static uint16_t seq = 0;

    pkt.event_id = 1;
    pkt.packet_type = 2;
    pkt.sample_count = 1;

    fill_ble_sample_from_imu(sample, &pkt.sample, seq++);

    k_work_submit(&ble_work);
}

bool ble_is_stack_ready(void)
{
    return ble_stack_ready;
}

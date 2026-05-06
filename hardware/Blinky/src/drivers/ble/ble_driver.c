#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include "ble_driver.h"

/* =========================
   GLOBALS
========================= */

static struct bt_conn *current_conn = NULL;
static bool notify_enabled = false;
static bool att_ready = false;
static struct k_work ble_work; 
static struct bt_gatt_attr *notify_attr; 

/* =========================
   TEST DATA
========================= */

static ble_imu_packet_t pkt;

/* =========================
   CALLBACKS
========================= */

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

    /* 🔥 MTU exchange */
    bt_gatt_exchange_mtu(conn, NULL);
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
    if (!current_conn || !notify_enabled || !att_ready)
        return;

    int err = bt_gatt_notify(current_conn,
                             notify_attr,
                             &pkt,
                             sizeof(pkt));

    if (err)
    {
        printk("Notify error: %d\n", err);
    }
    else
    {
        printk("IMU Notify OK (seq=%d)\n", pkt.sample.seq);
    }
}


/* =========================
   GATT SERVICE
========================= */

static struct bt_gatt_attr *notify_attr;   // 🔥 rigtig attr

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
   INIT
========================= */

void ble_init(void)
{
    int err = bt_enable(NULL);
    if (err) {
        printk("BLE init failed (%d)\n", err);
        return;
    }

    printk("BLE initialized\n");

    // notify_attr = &test_svc.attrs[1];
    // notify_attr = &test_svc.attrs[2];
    notify_attr = &test_svc.attrs[3];

    printk("attrs[0]=%p\n", &test_svc.attrs[0]);
    printk("attrs[1]=%p\n", &test_svc.attrs[1]);
    printk("attrs[2]=%p\n", &test_svc.attrs[2]);
    printk("attrs[3]=%p\n", &test_svc.attrs[3]);

    k_work_init(&ble_work, ble_notify_work);

    bt_gatt_cb_register(&gatt_callbacks);

    /* Advertising */

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

    err = bt_le_adv_start(&adv_param,
                          ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));

    printk("ADV START: %d\n", err);
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
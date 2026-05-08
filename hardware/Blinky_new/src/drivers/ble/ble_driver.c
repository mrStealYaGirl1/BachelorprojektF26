#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <string.h>

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

static ble_imu_packet_t pkt;
static uint8_t sample_index = 0;

static struct bt_conn *current_conn = NULL;
static bool notify_enabled = false;
static bool att_ready = false;
static bool ble_stack_ready = false;

static const struct bt_gatt_attr *notify_attr;
static struct bt_gatt_exchange_params mtu_exchange_params;

static struct k_work_delayable adv_restart_work;

static volatile bool ble_tx_busy = false;


// fælles TX type for både IMU og meta pakker
typedef struct {
    uint16_t packet_type;

    union {
        ble_imu_packet_t imu;
        ble_swing_meta_packet_t meta;
    } payload;
} ble_tx_item_t;



// tx queue definitions
#define BLE_TX_QUEUE_LEN 64
#define BLE_TX_STACK_SIZE 4096
#define BLE_TX_PRIORITY 6

K_MSGQ_DEFINE(ble_tx_q, sizeof(ble_tx_item_t), BLE_TX_QUEUE_LEN, 4);

K_THREAD_STACK_DEFINE(ble_tx_stack, BLE_TX_STACK_SIZE);
static struct k_thread ble_tx_thread_data;

static struct k_sem notify_done_sem;
static bool ble_tx_thread_started = false;


/* forward declaration */
static void adv_restart_handler(struct k_work *work);


/* =========================
   CALLBACKS
========================= */

static void notify_cb(struct bt_conn *conn, void *user_data)
{
    ble_tx_busy = false;
    k_sem_give(&notify_done_sem);
}

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


/* =========================
   Advertising start
========================= */

static int start_advertising(void)
{
    static const struct bt_le_adv_param adv_param = {
        .options = BT_LE_ADV_OPT_CONN,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    };

    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS,
            (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),

        BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xF0, 0xFF),

        BT_DATA(BT_DATA_NAME_COMPLETE,
            STRINGIFY(CONFIG_BT_DEVICE_NAME),
            strlen(STRINGIFY(CONFIG_BT_DEVICE_NAME))),
    };

    return bt_le_adv_start(&adv_param,
                           ad, ARRAY_SIZE(ad),
                           NULL, 0);
}

// static int start_advertising(void)
// {
//     static const struct bt_le_adv_param adv_param = {
//         .options = BT_LE_ADV_OPT_CONN,
//         .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
//         .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
//     };

//     const struct bt_data ad[] = {
//         BT_DATA_BYTES(BT_DATA_FLAGS,
//             (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),

//         BT_DATA(BT_DATA_NAME_COMPLETE,
//             STRINGIFY(CONFIG_BT_DEVICE_NAME),
//             strlen(STRINGIFY(CONFIG_BT_DEVICE_NAME))),

//         BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_GOLF_SVC_VAL),
//     };

//     return bt_le_adv_start(&adv_param,
//                            ad, ARRAY_SIZE(ad),
//                            NULL, 0);
// }

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
    att_ready = true;
    ble_tx_busy = false;
    sample_index = 0;

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
    ble_tx_busy = false;
    sample_index = 0;

    k_work_schedule(&adv_restart_work, K_MSEC(500));
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
   BLE TX THREAD
========================= */
static void ble_tx_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    ble_tx_item_t item;

    while (1)
    {
        k_msgq_get(&ble_tx_q, &item, K_FOREVER);

        if (!current_conn || !notify_enabled || !att_ready) {
            continue;
        }

        const void *data = NULL;
        uint16_t len = 0;

        /* =========================
           META packet
        ========================== */
        if (item.packet_type == BLE_PKT_TYPE_META)
        {
            data = &item.payload.meta;
            len = sizeof(ble_swing_meta_packet_t);

            printk("Sending META packet len=%u mtu=%u\n",
                len, bt_gatt_get_mtu(current_conn));
        }

        /* =========================
           IMU packet
        ========================== */
        else if (item.packet_type == BLE_PKT_TYPE_IMU)
        {
            data = &item.payload.imu;

            len =
                sizeof(item.payload.imu.event_id)
              + sizeof(item.payload.imu.packet_type)
              + sizeof(item.payload.imu.sample_count)
              + item.payload.imu.sample_count
                    * sizeof(ble_imu_sample_t);

            if (item.payload.imu.sample_count > 0)
            {
                uint16_t last_seq =
                    item.payload.imu.samples[
                        item.payload.imu.sample_count - 1
                    ].seq;

                if ((last_seq % 50) == 0) {
                    printk("BLE seq=%u\n", last_seq);
                }
            }
        }

        else
        {
            printk("Unknown BLE packet type: %u\n",
                   item.packet_type);
            continue;
        }

        uint16_t mtu = bt_gatt_get_mtu(current_conn);
        uint16_t max_payload = (mtu >= 3) ? (mtu - 3) : 0;

        if (len > max_payload)
        {
            printk("Packet too large: len=%u max=%u mtu=%u\n",
                   len, max_payload, mtu);
            continue;
        }

        while (k_sem_take(&notify_done_sem, K_NO_WAIT) == 0) {
            /* drain semaphore */
        }

        ble_tx_busy = true;

        struct bt_gatt_notify_params params = {
            .attr = notify_attr,
            .data = data,
            .len  = len,
            .func = notify_cb,
        };

        int err = bt_gatt_notify_cb(current_conn, &params);

        if (err)
        {
            printk("Notify error: %d\n", err);
            ble_tx_busy = false;
            continue;
        }

        k_sem_take(&notify_done_sem, K_MSEC(200));

        k_msleep(5); // pacing
    }
}


bool ble_queue_imu_packet(const ble_imu_packet_t *packet)
{
    if (!packet) return false;
    if (!current_conn || !notify_enabled || !att_ready) return false;

    ble_tx_item_t item;
    item.packet_type = BLE_PKT_TYPE_IMU;
    item.payload.imu = *packet;

    return k_msgq_put(&ble_tx_q, &item, K_FOREVER) == 0;
}


bool ble_queue_meta_packet(const ble_swing_meta_packet_t *packet)
{
    if (!packet) return false;
    if (!current_conn || !notify_enabled || !att_ready) return false;

    ble_tx_item_t item;
    item.packet_type = BLE_PKT_TYPE_META;
    item.payload.meta = *packet;

    return k_msgq_put(&ble_tx_q, &item, K_MSEC(20)) == 0;
}


/* =========================
   GATT SERVICE
========================= */

static uint8_t dummy_value = 0;

static ssize_t read_dummy(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          void *buf,
                          uint16_t len,
                          uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &dummy_value, sizeof(dummy_value));
}

BT_GATT_SERVICE_DEFINE(test_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(0xFFF0)),

    BT_GATT_CHARACTERISTIC(
        BT_UUID_DECLARE_16(0xFFF1),
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        read_dummy, NULL, &dummy_value
    ),

    BT_GATT_CCC(ccc_cfg_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);


// BT_GATT_SERVICE_DEFINE(test_svc,
//     BT_GATT_PRIMARY_SERVICE(&golf_svc_uuid.uuid),

//     BT_GATT_CHARACTERISTIC(
//         &golf_chr_uuid.uuid,
//         BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
//         BT_GATT_PERM_READ,
//         read_dummy, NULL, &dummy_value
//     ),

//     BT_GATT_CCC(ccc_cfg_changed,
//         BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
// );


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

    k_sem_init(&notify_done_sem, 0, 1);
    k_work_init_delayable(&adv_restart_work, adv_restart_handler);

    if (!ble_tx_thread_started) {
        k_thread_create(&ble_tx_thread_data,
                        ble_tx_stack,
                        BLE_TX_STACK_SIZE,
                        ble_tx_thread,
                        NULL, NULL, NULL,
                        BLE_TX_PRIORITY,
                        0,
                        K_NO_WAIT);

        ble_tx_thread_started = true;
    }

    bt_gatt_cb_register(&gatt_callbacks);

    int err = start_advertising();

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
    static uint16_t seq = 0;

    ble_imu_packet_t test_pkt;

    test_pkt.event_id = 1;
    test_pkt.packet_type = 2;
    test_pkt.sample_count = 1;

    test_pkt.samples[0].ax = 100 + seq;
    test_pkt.samples[0].ay = 200;
    test_pkt.samples[0].az = 300;

    test_pkt.samples[0].gx = 10;
    test_pkt.samples[0].gy = 20;
    test_pkt.samples[0].gz = 30;

    test_pkt.samples[0].ts_ms = k_uptime_get_32();
    test_pkt.samples[0].seq = seq++;

    ble_queue_imu_packet(&test_pkt);
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

    dst->ts_ms = (uint32_t)(src->timestamp_us / 1000ULL);
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


void ble_send_imu_sample_for_event(const imu_sample_t *sample, uint16_t event_id)
{
    if (sample == NULL)
        return;

    if (!current_conn || !notify_enabled || !att_ready)
        return;

    static uint16_t seq = 0;

    pkt.event_id = event_id;
    pkt.packet_type = BLE_PKT_TYPE_IMU;

    if (sample_index < BLE_SAMPLES_PER_PKT)
    {
        fill_ble_sample_from_imu(sample,
                                 &pkt.samples[sample_index],
                                 seq++);

        sample_index++;
    }

    if (sample_index >= BLE_SAMPLES_PER_PKT)
    {
        pkt.sample_count = sample_index;

        if (ble_queue_imu_packet(&pkt)) {
            sample_index = 0;
        }
    }
}

// void ble_send_imu_sample(const imu_sample_t *sample)
// {
//     if (sample == NULL)
//         return;

//     if (!current_conn || !notify_enabled || !att_ready)
//         return;

//     static uint16_t seq = 0;

//     pkt.event_id = 1;
//     pkt.packet_type = 2;

//     /* 🔒 overflow protection */
//     if (sample_index < BLE_SAMPLES_PER_PKT)
//     {
//         fill_ble_sample_from_imu(sample,
//                                  &pkt.samples[sample_index],
//                                  seq++);

//         sample_index++;
//     }

//     if (sample_index >= BLE_SAMPLES_PER_PKT)
//     {
//         pkt.sample_count = sample_index;

//         if (ble_queue_imu_packet(&pkt)) {
//             sample_index = 0;
//         }
//     }
// }

bool ble_is_stack_ready(void)
{
    return ble_stack_ready;
}


static void adv_restart_handler(struct k_work *work)
{
    int err = start_advertising();
    printk("Advertising restart: %d\n", err);
}



bool ble_is_tx_busy(void)
{
    return ble_tx_busy || (k_msgq_num_used_get(&ble_tx_q) > 0);
}
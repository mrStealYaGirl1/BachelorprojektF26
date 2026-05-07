#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <SEGGER_RTT.h>

#include "drivers/ble/ble_driver.h"
#include "drivers/imu/imu_driver.h"
#include "drivers/imu/imu_processing.h"
#include "drivers/swing_manager/swing_manager.h"

#define LED_NODE DT_NODELABEL(gpio1)
#define LED_PIN 12

#define IMU_STACK_SIZE     4096
#define IMU_PRIORITY       5

#define SWING_STACK_SIZE   4096
#define SWING_PRIORITY     5

K_THREAD_STACK_DEFINE(imu_stack, IMU_STACK_SIZE);
static struct k_thread imu_thread_data;

K_THREAD_STACK_DEFINE(swing_stack, SWING_STACK_SIZE);
static struct k_thread swing_thread_data;

int main(void)
{
    const struct device *gpio1 = DEVICE_DT_GET(LED_NODE);

    SEGGER_RTT_WriteString(0, "BOOT\r\n");

    if (device_is_ready(gpio1)) {
        gpio_pin_configure(gpio1, LED_PIN, GPIO_OUTPUT_INACTIVE);
    }

    printk("Before BLE init\n");

    ble_init();

    while (!ble_is_stack_ready()) {
        printk("Waiting for BLE ready...\n");
        k_sleep(K_MSEC(100));
    }

    printk("BLE stack ready\n");

    ble_post_init();
    printk("BLE advertising started\n");

    printk("Before IMU init\n");

    if (imu_init() != 0) {
        printk("IMU init failed\n");
        return 0;
    }

    imu_ringbuffer_init();
    imu_processing_init();

    k_thread_create(&imu_thread_data,
                    imu_stack,
                    IMU_STACK_SIZE,
                    imu_thread,
                    NULL, NULL, NULL,
                    IMU_PRIORITY,
                    0,
                    K_NO_WAIT);

    printk("IMU thread started\n");

    k_thread_create(&swing_thread_data,
                    swing_stack,
                    SWING_STACK_SIZE,
                    swing_manager_thread,
                    NULL, NULL, NULL,
                    SWING_PRIORITY,
                    0,
                    K_NO_WAIT);

    printk("Swing manager thread started\n");

    while (1) {
        gpio_pin_toggle(gpio1, LED_PIN);
        k_sleep(K_SECONDS(1));
    }
}


// ble connect virker
// #include <zephyr/kernel.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/sys/printk.h>
// #include <SEGGER_RTT.h>

// #include "drivers/ble/ble_driver.h"

// #define LED_NODE DT_NODELABEL(gpio1)
// #define LED_PIN 12

// int main(void)
// {
//     const struct device *gpio1 = DEVICE_DT_GET(LED_NODE);

//     SEGGER_RTT_WriteString(0, "BOOT\r\n");

//     if (device_is_ready(gpio1)) {
//         gpio_pin_configure(gpio1, LED_PIN, GPIO_OUTPUT_INACTIVE);
//     }

//     printk("Before BLE init\n");

//     ble_init();

//     while (!ble_is_stack_ready()) {
//         printk("Waiting for BLE ready...\n");
//         k_sleep(K_MSEC(100));
//     }

//     printk("BLE stack ready\n");

//     ble_post_init();

//     printk("BLE advertising started\n");

//     while (1) {
//         gpio_pin_toggle(gpio1, LED_PIN);
//         //printk("main alive\n");
//         ble_send_test();
//         k_sleep(K_SECONDS(1));
//     }
// }


// kan se device på nrf connect
// #include <zephyr/kernel.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/sys/printk.h>
// #include <SEGGER_RTT.h>

// #include "drivers/ble/ble_driver.h"

// #define LED_NODE DT_NODELABEL(gpio1)
// #define LED_PIN 12

// int main(void)
// {
//     const struct device *gpio1 = DEVICE_DT_GET(LED_NODE);

//     SEGGER_RTT_WriteString(0, "BOOT\r\n");

//     if (device_is_ready(gpio1)) {
//         gpio_pin_configure(gpio1, LED_PIN, GPIO_OUTPUT_INACTIVE);
//     }

//     printk("Before BLE init\n");

//     ble_init();

//     while (!ble_is_stack_ready()) {
//         printk("Waiting for BLE ready...\n");
//         k_sleep(K_MSEC(100));
//     }

//     printk("BLE stack ready\n");

//     ble_post_init();

//     printk("BLE advertising started\n");

//     while (1) {
//         gpio_pin_toggle(gpio1, LED_PIN);
//         printk("main alive\n");
//         k_sleep(K_SECONDS(1));
//     }
// }


// #include <zephyr/kernel.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/sys/printk.h>
// #include <SEGGER_RTT.h>

// #include "drivers/imu/imu_driver.h"
// #include "drivers/ble/ble_driver.h"

// #define LED_NODE DT_NODELABEL(gpio1)
// #define LED_PIN 12

// #define IMU_STACK_SIZE 4096
// #define IMU_PRIORITY   5

// K_THREAD_STACK_DEFINE(imu_stack, IMU_STACK_SIZE);
// static struct k_thread imu_thread_data;

// int main(void)
// {
//     const struct device *gpio1 = DEVICE_DT_GET(LED_NODE);

//     SEGGER_RTT_WriteString(0, "BOOT\r\n");

//     if (device_is_ready(gpio1)) {
//         gpio_pin_configure(gpio1, LED_PIN, GPIO_OUTPUT_INACTIVE);
//     }

//     if (imu_init() != 0) {
//         printk("IMU init failed\n");
//         return 0;
//     }

//     imu_ringbuffer_init();

//     k_thread_create(&imu_thread_data,
//                     imu_stack,
//                     IMU_STACK_SIZE,
//                     imu_thread,
//                     NULL, NULL, NULL,
//                     IMU_PRIORITY,
//                     0,
//                     K_NO_WAIT);

//     printk("IMU thread started\n");


//     printk("Before BLE init\n");

//     ble_init();

//     while (!ble_is_stack_ready()) {
//         printk("Waiting for BLE ready...\n");
//         k_sleep(K_MSEC(100));
//     }

//     printk("BLE stack ready\n");

//     ble_post_init();

//     printk("BLE advertising started\n");

//     while (1) {
//         gpio_pin_toggle(gpio1, LED_PIN);
//         printk("main alive\n");
//         k_sleep(K_SECONDS(1));
//     }
// }


// IMU-læsning live virker
// #include <zephyr/kernel.h>
// #include <zephyr/drivers/gpio.h>
// #include <SEGGER_RTT.h>
// #include <zephyr/sys/printk.h>

// #include "drivers/imu/imu_driver.h"

// #define LED_NODE DT_NODELABEL(gpio1)
// #define LED_PIN 12

// int main(void)
// {
//     const struct device *gpio1 = DEVICE_DT_GET(LED_NODE);
//     imu_sample_t sample;

//     SEGGER_RTT_WriteString(0, "BOOT\r\n");

//     if (device_is_ready(gpio1)) {
//         gpio_pin_configure(gpio1, LED_PIN, GPIO_OUTPUT_INACTIVE);
//     }

//     SEGGER_RTT_WriteString(0, "Before IMU init\r\n");

//     if (imu_init() != 0) {
//         SEGGER_RTT_WriteString(0, "IMU init failed\r\n");
//         while (1) {
//             gpio_pin_toggle(gpio1, LED_PIN);
//             k_sleep(K_MSEC(200));
//         }
//     }

//     SEGGER_RTT_WriteString(0, "IMU init OK\r\n");

//     while (1) {
//         imu_get_latest(&sample);

//         printk("ACC: %d %d %d | GYRO: %d %d %d | t=%llu\n",
//                sample.ax, sample.ay, sample.az,
//                sample.gx, sample.gy, sample.gz,
//                sample.timestamp_ms);

//         gpio_pin_toggle(gpio1, LED_PIN);
//         k_sleep(K_MSEC(100));
//     }
// }


// SPI OK
// #include <zephyr/kernel.h>
// #include <zephyr/drivers/gpio.h>
// #include <SEGGER_RTT.h>

// #include "drivers/imu/imu_driver.h"

// #define LED_NODE DT_NODELABEL(gpio1)
// #define LED_PIN 12

// int main(void)
// {
//     const struct device *gpio1 = DEVICE_DT_GET(LED_NODE);

//     SEGGER_RTT_WriteString(0, "BOOT\r\n");

//     if (device_is_ready(gpio1)) {
//         gpio_pin_configure(gpio1, LED_PIN, GPIO_OUTPUT_INACTIVE);
//     }

//     SEGGER_RTT_WriteString(0, "Before IMU init\r\n");

//     int ret = imu_init();

//     if (ret != 0) {
//         SEGGER_RTT_WriteString(0, "IMU init failed\r\n");
//         while (1) {
//             gpio_pin_toggle(gpio1, LED_PIN);
//             k_sleep(K_MSEC(200));
//         }
//     }

//     SEGGER_RTT_WriteString(0, "IMU init OK\r\n");

//     while (1) {
//         gpio_pin_toggle(gpio1, LED_PIN);
//         SEGGER_RTT_WriteString(0, "ALIVE + IMU OK\r\n");
//         k_sleep(K_MSEC(500));
//     }
// }

// blink led test for nRF52840, using Zephyr RTOS and SEGGER RTT for logging
// #include <zephyr/kernel.h>
// #include <zephyr/drivers/gpio.h>
// #include <SEGGER_RTT.h>

// #define LED_NODE DT_NODELABEL(gpio1)
// #define LED_PIN 12

// int main(void)
// {
//     const struct device *gpio1 = DEVICE_DT_GET(LED_NODE);

//     SEGGER_RTT_WriteString(0, "BOOT\r\n");

//     if (!device_is_ready(gpio1)) {
//         while (1) {
//             SEGGER_RTT_WriteString(0, "GPIO1 not ready\r\n");
//             k_sleep(K_MSEC(500));
//         }
//     }

//     gpio_pin_configure(gpio1, LED_PIN, GPIO_OUTPUT_INACTIVE);

//     while (1) {
//         gpio_pin_toggle(gpio1, LED_PIN);
//         SEGGER_RTT_WriteString(0, "ALIVE\r\n");
//         k_sleep(K_MSEC(500));
//     }
// }

// #include <zephyr/kernel.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/bluetooth/bluetooth.h>
// #include <zephyr/bluetooth/gatt.h>
// #include <zephyr/bluetooth/gap.h>
// #include <zephyr/bluetooth/hci.h>

// #include "drivers/ble/ble_driver.h"
// #include "drivers/imu/imu_driver.h"
// #include "drivers/imu/imu_processing.h"
// #include "drivers/swing_manager/swing_manager.h"

// /* =========================
// THREAD CONFIG
// ========================= */

// #define IMU_STACK_SIZE    8192   // lidt stor til debug (ok)
// #define IMU_PRIORITY      5

// #define SWING_STACK_SIZE  4096
// #define SWING_PRIORITY    5

// /* =========================
// THREAD STACKS
// ========================= */

// K_THREAD_STACK_DEFINE(imu_stack, IMU_STACK_SIZE);
// static struct k_thread imu_thread_data;

// K_THREAD_STACK_DEFINE(swing_stack, SWING_STACK_SIZE);
// static struct k_thread swing_thread_data;

// /* =========================
// MAIN
// ========================= */

// int main(void)
// {
// k_sleep(K_SECONDS(2));
// printk("MAIN START\n");


// /* -------- BLE INIT -------- */
// ble_init();

// while (!ble_is_stack_ready()) {
//     printk("Waiting for BLE ready...\n");
//     k_sleep(K_MSEC(100));
// }

// printk("After ble_init\n");

// ble_post_init();
// printk("After ble_post_init\n");

// /* -------- IMU INIT -------- */
// if (imu_init() != 0) {
//     printk("IMU init failed\n");
//     return 0;
// }

// printk("After imu_init\n");

// /* -------- PROCESSING INIT -------- */
// imu_processing_init();
// imu_ringbuffer_init();

// /* -------- IMU THREAD -------- */
// k_thread_create(&imu_thread_data,
//                 imu_stack,
//                 IMU_STACK_SIZE,
//                 imu_thread,
//                 NULL, NULL, NULL,
//                 IMU_PRIORITY,
//                 0,
//                 K_NO_WAIT);

// printk("IMU thread started\n");

// /* -------- SWING MANAGER THREAD -------- */
// k_thread_create(&swing_thread_data,
//                 swing_stack,
//                 SWING_STACK_SIZE,
//                 swing_manager_thread,
//                 NULL, NULL, NULL,
//                 SWING_PRIORITY,
//                 0,
//                 K_NO_WAIT);

// printk("Swing manager thread started\n");

// /* -------- MAIN LOOP -------- */
// while (1) {
//     k_sleep(K_SECONDS(1));
// }


// }


// int main(void)
// {
//     k_sleep(K_SECONDS(2));

//     printk("MAIN START\n");

//     ble_init();

//     while (!ble_is_stack_ready()) {
//         printk("Waiting for BLE ready...\n");
//         k_sleep(K_MSEC(100));
//     }

//     printk("After ble_init\n");

//     if (imu_init() != 0) {
//         printk("IMU init failed\n");
//         return 0;
//     }
//     printk("After imu_init\n");

//     imu_processing_init();
//     imu_ringbuffer_init();

//     k_thread_create(&imu_thread_data,
//                     imu_stack,
//                     IMU_STACK_SIZE,
//                     imu_thread,
//                     NULL, NULL, NULL,
//                     IMU_PRIORITY,
//                     0,
//                     K_NO_WAIT);

//     printk("After k_thread_create\n");

//     while (1)
//     {
//         printk("main alive\n");
//         k_sleep(K_SECONDS(1));
//     }
// }



// int main(void)
// {
//     printk("MAIN START\n");

//     ble_init();

//     while (1)
//     {
//         ble_send_test();
//         k_sleep(K_MSEC(300));
//     }
// }

// //----------------------------------------------------------------
// //nrF820
// //----------------------------------------------------------------

// #include <zephyr/kernel.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/bluetooth/bluetooth.h>
// #include <zephyr/bluetooth/gatt.h>
// #include <zephyr/bluetooth/gap.h> 
// #include <zephyr/bluetooth/hci.h>

// /* =========================
//    CONFIG
// ========================= */

// #define BLE_PKT_TYPE_IMU 2

// /* =========================
//    GLOBALS
// ========================= */

// static struct bt_conn *current_conn = NULL;
// static bool notify_enabled = false;
// static uint16_t seq_counter = 0;
// static bool att_ready = false;

// /* =========================
//    BLE PACKETS
// ========================= */

// #pragma pack(push, 1)

// typedef struct {
//     int16_t ax, ay, az;
//     int16_t gx, gy, gz;
//     uint32_t ts_ms;
//     uint16_t seq;
// } ble_imu_sample_t;

// typedef struct {
//     uint16_t event_id;
//     uint16_t packet_type;
//     uint16_t sample_count;
//     ble_imu_sample_t sample;
// } ble_imu_packet_t;

// #pragma pack(pop)

// static ble_imu_packet_t pkt;

// /* =========================
//    BLE CALLBACKS
// ========================= */

// static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
// {
//     notify_enabled = (value == BT_GATT_CCC_NOTIFY);

//     printk("CCC CHANGED: %d\n", value);

//     if (notify_enabled) {
//         printk("Notifications ENABLED\n");
//     }
// }

// static void connected(struct bt_conn *conn, uint8_t err)
// {
//     if (err) {
//         printk("Connection failed (%u)\n", err);
//         return;
//     }

//     printk("Connected\n");

//     if (current_conn) {
//         bt_conn_unref(current_conn);
//     }

//     current_conn = bt_conn_ref(conn);

//     bt_gatt_exchange_mtu(conn, NULL);
// }

// static void disconnected(struct bt_conn *conn, uint8_t reason)
// {
//     printk("Disconnected (%u)\n", reason);

//     if (current_conn) {
//         bt_conn_unref(current_conn);
//         current_conn = NULL;
//     }

//     notify_enabled = false;
//     att_ready = false;

// }

// BT_CONN_CB_DEFINE(conn_callbacks) = {
//     .connected = connected,
//     .disconnected = disconnected,
// };

// static void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
// {
//     printk("MTU updated: TX=%d RX=%d\n", tx, rx);

//     att_ready = true;
// }

// static struct bt_gatt_cb gatt_callbacks = {
//     .att_mtu_updated = mtu_updated,
// };

// /* =========================
//    GATT SERVICE
// ========================= */

// static uint8_t dummy_value[20];

// BT_GATT_SERVICE_DEFINE(test_svc,
//     BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(0xFFF0)),

//     BT_GATT_CHARACTERISTIC(
//         BT_UUID_DECLARE_16(0xFFF1),
//         BT_GATT_CHRC_NOTIFY,
//         BT_GATT_PERM_NONE,
//         NULL, NULL, dummy_value
//     ),

//     BT_GATT_CCC(ccc_cfg_changed,
//         BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
// );

// /* =========================
//    MAIN
// ========================= */

// int main(void)
// {
//     printk("TEST START\n");

//     /* 1. Start BLE */
//     int err = bt_enable(NULL);
//     if (err) {
//         printk("BLE failed (%d)\n", err);
//         return 0;
//     }
    
//     bt_gatt_cb_register(&gatt_callbacks);
    
//     printk("BLE OK\n");

//     /* 2. Advertising */
//     static const struct bt_le_adv_param adv_param = {
//         .options = BT_LE_ADV_OPT_CONN,
//         .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
//         .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
//     };

//     const struct bt_data ad[] = {
//         BT_DATA_BYTES(BT_DATA_FLAGS,
//             (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
//     };

//     const struct bt_data sd[] = {
//         BT_DATA(BT_DATA_NAME_COMPLETE,
//                 CONFIG_BT_DEVICE_NAME,
//                 sizeof(CONFIG_BT_DEVICE_NAME) - 1),
//     };

//     err = bt_le_adv_start(&adv_param,
//                           ad, ARRAY_SIZE(ad),
//                           sd, ARRAY_SIZE(sd));

//     printk("ADV RESULT: %d\n", err);

//     /* 3. Main loop */
//     while (1)
//     {
//         printk("RUNNING\n");

//         if (!current_conn || !notify_enabled || !att_ready)
//         {
//             printk("Not ready: conn=%p notify=%d att=%d\n",
//                 current_conn, notify_enabled, att_ready);

//             k_sleep(K_MSEC(300));
//             continue;
//         }

//         /* Fake data */
//         pkt.event_id = 1;
//         pkt.packet_type = BLE_PKT_TYPE_IMU;
//         pkt.sample_count = 1;

//         pkt.sample.ax = 100;
//         pkt.sample.ay = 200;
//         pkt.sample.az = 300;

//         pkt.sample.gx = 10;
//         pkt.sample.gy = 20;
//         pkt.sample.gz = 30;

//         pkt.sample.ts_ms = k_uptime_get_32();
//         pkt.sample.seq = seq_counter++;

//         int err = bt_gatt_notify(current_conn,
//                                  &test_svc.attrs[2],
//                                  &pkt,
//                                  sizeof(pkt));

//         if (err == -ENOTCONN || err == -EAGAIN || err == -ENOMEM)
//         {
//             printk("BLE not ready yet (%d)\n", err);
//         }
//         else if (err)
//         {
//             printk("Notify error: %d\n", err);
//         }
//         else
//         {
//             printk("Notify OK\n");
//         }

//         k_sleep(K_MSEC(300));
//     }

//     return 0;
// }

// //----------------------------------------------------------------
// // Main application code for IMU data transmission over BLE
// //----------------------------------------------------------------
// #include <zephyr/kernel.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/bluetooth/bluetooth.h>
// #include <zephyr/bluetooth/gatt.h>
// #include <zephyr/bluetooth/gap.h> 
// #include <zephyr/bluetooth/hci.h>

// #include "drivers/spi/spi_driver.h"
// #include "drivers/imu/imu_driver.h"
// #include "drivers/imu/imu_processing.h"

// /* =========================
//    CONFIG
// ========================= */

// #define BLE_PKT_TYPE_IMU 2
// #define BLE_IMU_SAMPLES_PER_PKT 2   // 🔥 lav = stabil BLE
// #define IMU_STACK_SIZE 4096
// #define IMU_PRIORITY   5

// /* =========================
//    GLOBALS
// ========================= */

// K_THREAD_STACK_DEFINE(imu_stack, IMU_STACK_SIZE);
// static struct k_thread imu_thread_data;

// static struct bt_conn *current_conn = NULL;
// static bool notify_enabled = false;

// static uint16_t current_event_id = 1;
// static uint16_t seq_counter = 0;
// static uint8_t sample_index = 0;

// /* =========================
//    BLE PACKETS
// ========================= */

// #pragma pack(push, 1)

// typedef struct {
//     int16_t ax, ay, az;
//     int16_t gx, gy, gz;
//     uint32_t ts_ms;
//     uint16_t seq;
// } ble_imu_sample_t;

// typedef struct {
//     uint16_t event_id;
//     uint16_t packet_type;
//     uint16_t sample_count;
//     ble_imu_sample_t samples[BLE_IMU_SAMPLES_PER_PKT];
// } ble_imu_packet_t;

// #pragma pack(pop)

// static ble_imu_packet_t pkt;

// /* =========================
//    HELPERS
// ========================= */

// static void convert_sample(const imu_sample_t *in,
//                            ble_imu_sample_t *out,
//                            uint16_t seq)
// {
//     out->ax = in->ax;
//     out->ay = in->ay;
//     out->az = in->az;

//     out->gx = in->gx;
//     out->gy = in->gy;
//     out->gz = in->gz;

//     out->ts_ms = (uint32_t)in->timestamp_ms;
//     out->seq = seq;
// }

// /* =========================
//    BLE CALLBACKS
// ========================= */

// static volatile bool att_ready = false;

// static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
// {
//     notify_enabled = (value == BT_GATT_CCC_NOTIFY);

//     printk("Notifications %s\n",
//            notify_enabled ? "ENABLED" : "DISABLED");

//     if (notify_enabled) {
//         printk("Waiting for ATT ready...\n");
//     }
// }

// static void connected(struct bt_conn *conn, uint8_t err)
// {
//     if (err) {
//         printk("Connection failed (%u)\n", err);
//         return;
//     }

//     printk("Connected\n");

//     if (current_conn) {
//         bt_conn_unref(current_conn);
//     }

//     current_conn = bt_conn_ref(conn);
// }

// static void disconnected(struct bt_conn *conn, uint8_t reason)
// {
//     printk("Disconnected (%u)\n", reason);

//     if (current_conn) {
//         bt_conn_unref(current_conn);
//         current_conn = NULL;
//     }

//     notify_enabled = false;
// }

// BT_CONN_CB_DEFINE(conn_callbacks) = {
//     .connected = connected,
//     .disconnected = disconnected,
// };

// /* =========================
//    GATT SERVICE (DIN CUSTOM!)
// ========================= */

// BT_GATT_SERVICE_DEFINE(test_svc,
//     BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(0xFFF0)),

//     BT_GATT_CHARACTERISTIC(
//         BT_UUID_DECLARE_16(0xFFF1),
//         BT_GATT_CHRC_NOTIFY,
//         BT_GATT_PERM_NONE,
//         NULL, NULL, NULL
//     ),

//     BT_GATT_CCC(ccc_cfg_changed,
//         BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
// );

// /* =========================
//    SEND PACKET
// ========================= */

// static void send_packet(void)
// {
//     if (!current_conn || !notify_enabled) {
//         return;
//     }

//     pkt.event_id = current_event_id;
//     pkt.packet_type = BLE_PKT_TYPE_IMU;
//     pkt.sample_count = sample_index;

//     int err = bt_gatt_notify(current_conn,
//                              &test_svc.attrs[2],   // 🔥 korrekt attribute
//                              &pkt,
//                              sizeof(pkt));

//     if (err) {
//         printk("Notify failed: %d\n", err);
//         return;
//     }

//     sample_index = 0;
// }

// /* =========================
//    MAIN
// ========================= */

// int main(void)
// {
//     printk("TEST START\n");

//     /* =========================
//        1. Start BLE
//     ========================= */
//     int err = bt_enable(NULL);
//     if (err) {
//         printk("BLE failed (%d)\n", err);
//         return 0;
//     }

//     printk("BLE OK\n");

//     /* =========================
//        2. Advertising setup
//     ========================= */
//     static const struct bt_le_adv_param adv_param = {
//         .id = BT_ID_DEFAULT,
//         .sid = 0,
//         .secondary_max_skip = 0,
//         .options = BT_LE_ADV_OPT_CONN,
//         .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
//         .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
//         .peer = NULL,
//     };

//     const struct bt_data ad[] = {
//         BT_DATA_BYTES(BT_DATA_FLAGS,
//             (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
//     };

//     const struct bt_data sd[] = {
//         BT_DATA(BT_DATA_NAME_COMPLETE,
//                 CONFIG_BT_DEVICE_NAME,
//                 sizeof(CONFIG_BT_DEVICE_NAME) - 1),
//     };

//     /* =========================
//        3. Start advertising
//     ========================= */
//     err = bt_le_adv_start(&adv_param,
//                           ad, ARRAY_SIZE(ad),
//                           sd, ARRAY_SIZE(sd));

//     printk("ADV RESULT: %d\n", err);

//     /* =========================
//        4. Main loop
//     ========================= */
//         while (1)
//         {
//             printk("RUNNING\n");

//             // 🔥 HER SKAL DEN VÆRE (øverst i loopet)
//             if (!current_conn || !notify_enabled || !att_ready)
//             {
//                 printk("Not ready: conn=%p notify=%d att=%d\n",
//                     current_conn, notify_enabled, att_ready);

//                 k_sleep(K_MSEC(100));
//                 continue;   // ← stopper resten af loopet
//             }

//             // 🔥 KUN hvis alt er klar, kommer vi herned

//             pkt.event_id = 1;
//             pkt.packet_type = BLE_PKT_TYPE_IMU;
//             pkt.sample_count = 1;

//             pkt.samples[0].ax = 100;
//             pkt.samples[0].ay = 200;
//             pkt.samples[0].az = 300;

//             pkt.samples[0].gx = 10;
//             pkt.samples[0].gy = 20;
//             pkt.samples[0].gz = 30;

//             pkt.samples[0].ts_ms = k_uptime_get_32();
//             pkt.samples[0].seq = seq_counter++;

//             int size = sizeof(pkt.event_id) +
//                     sizeof(pkt.packet_type) +
//                     sizeof(pkt.sample_count) +
//                     pkt.sample_count * sizeof(ble_imu_sample_t);

//             struct bt_gatt_notify_params params = {0};

//             params.attr = &test_svc.attrs[2];
//             params.data = &pkt;
//             params.len  = size;

//             int err = bt_gatt_notify_cb(current_conn, &params);
//             k_sleep(K_MSEC(200));   // i stedet for 100

//             printk("Notify: %d\n", err);

//             k_sleep(K_MSEC(100));
//         }

//     return 0;
// }


//----------------------------------------------------------------
// Simpel BLE test, virker på nRF52840
//----------------------------------------------------------------

// #include <zephyr/kernel.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/bluetooth/bluetooth.h>
// #include <zephyr/bluetooth/gap.h>

// /* Advertising params */
// static const struct bt_le_adv_param adv_param = {
//     .id = BT_ID_DEFAULT,
//     .sid = 0,
//     .secondary_max_skip = 0,
//     .options = BT_LE_ADV_OPT_CONN,
//     .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
//     .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
//     .peer = NULL,
// };

// int main(void)
// {
//     printk("=== SIMPLE BLE TEST ===\n");

//     int err = bt_enable(NULL);
//     if (err) {
//         printk("BLE init failed (%d)\n", err);
//         return 0;
//     }

//     printk("BLE initialized\n");

//     /* 🔥 VIGTIG: vent lidt */
//     k_sleep(K_MSEC(500));

//     const struct bt_data ad[] = {
//         BT_DATA_BYTES(BT_DATA_FLAGS,
//             (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
//     };

//     const struct bt_data sd[] = {
//         BT_DATA(BT_DATA_NAME_COMPLETE,
//                 CONFIG_BT_DEVICE_NAME,
//                 sizeof(CONFIG_BT_DEVICE_NAME) - 1),
//     };

//     err = bt_le_adv_start(&adv_param,
//                           ad, ARRAY_SIZE(ad),
//                           sd, ARRAY_SIZE(sd));

//     if (err) {
//         printk("Advertising failed (%d)\n", err);
//         return 0;
//     }

//     printk("Advertising started!\n");

//     while (1) {
//         k_sleep(K_SECONDS(1));
//     }
// }

//---------------------------------------------------------------
// Gammel main.c, virkede ikke efter vi skiftede til nRF52840
//----------------------------------------------------------------

// #include <zephyr/kernel.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/bluetooth/bluetooth.h>
// #include <zephyr/bluetooth/gatt.h>
// #include <zephyr/bluetooth/gap.h> 
// #include <zephyr/bluetooth/hci.h>

// #include "drivers/spi/spi_driver.h"
// #include "drivers/imu/imu_driver.h"
// #include "drivers/imu/imu_processing.h"

// /* =========================
//    CONFIG
// ========================= */

// #define BLE_PKT_TYPE_IMU 2
// #define BLE_IMU_SAMPLES_PER_PKT 2   // vigtigt (stabil BLE)
// #define IMU_STACK_SIZE 4096
// #define IMU_PRIORITY   5

// /* =========================
//    GLOBALS
// ========================= */

// K_THREAD_STACK_DEFINE(imu_stack, IMU_STACK_SIZE);
// static struct k_thread imu_thread_data;

// static struct bt_conn *current_conn = NULL;
// static bool notify_enabled = false;

// static uint16_t current_event_id = 1;
// static uint16_t seq_counter = 0;
// static uint8_t sample_index = 0;

// /* =========================
//    BLE PACKETS
// ========================= */

// #pragma pack(push, 1)

// typedef struct {
//     int16_t ax, ay, az;
//     int16_t gx, gy, gz;
//     uint32_t ts_ms;
//     uint16_t seq;
// } ble_imu_sample_t;

// typedef struct {
//     uint16_t event_id;
//     uint16_t packet_type;
//     uint16_t sample_count;
//     ble_imu_sample_t samples[BLE_IMU_SAMPLES_PER_PKT];
// } ble_imu_packet_t;

// #pragma pack(pop)

// static ble_imu_packet_t pkt;

// /* =========================
//    HELPERS
// ========================= */

// static void convert_sample(const imu_sample_t *in,
//                            ble_imu_sample_t *out,
//                            uint16_t seq)
// {
//     out->ax = in->ax;
//     out->ay = in->ay;
//     out->az = in->az;

//     out->gx = in->gx;
//     out->gy = in->gy;
//     out->gz = in->gz;

//     out->ts_ms = (uint32_t)in->timestamp_ms;
//     out->seq = seq;
// }

// /* =========================
//    BLE CALLBACKS
// ========================= */

// static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
// {
//     notify_enabled = (value == BT_GATT_CCC_NOTIFY);

//     printk("Notifications %s\n",
//            notify_enabled ? "ENABLED" : "DISABLED");

//     if (notify_enabled) {
//         k_sleep(K_MSEC(200));   // FIX
//     }
// }

// static void connected(struct bt_conn *conn, uint8_t err)
// {
//     if (err) {
//         printk("Connection failed (%u)\n", err);
//         return;
//     }

//     printk("Connected\n");

//     if (current_conn) {
//         bt_conn_unref(current_conn);
//     }

//     current_conn = bt_conn_ref(conn);
// }

// static void disconnected(struct bt_conn *conn, uint8_t reason)
// {
//     printk("Disconnected (%u)\n", reason);

//     if (current_conn) {
//         bt_conn_unref(current_conn);
//         current_conn = NULL;
//     }

//     notify_enabled = false;
// }

// BT_CONN_CB_DEFINE(conn_callbacks) = {
//     .connected = connected,
//     .disconnected = disconnected,
// };

// /* =========================
//    GATT SERVICE
// ========================= */

// BT_GATT_SERVICE_DEFINE(test_svc,
//     BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(0xFFF0)),

//     BT_GATT_CHARACTERISTIC(
//         BT_UUID_DECLARE_16(0xFFF1),
//         BT_GATT_CHRC_NOTIFY,
//         BT_GATT_PERM_NONE,
//         NULL, NULL, NULL
//     ),

//     BT_GATT_CCC(ccc_cfg_changed,
//         BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
// );

// /* =========================
//    SEND PACKET
// ========================= */

// static void send_packet(void)
// {
//     if (!current_conn || !notify_enabled) {
//         return;
//     }

//     pkt.event_id = current_event_id;
//     pkt.packet_type = BLE_PKT_TYPE_IMU;
//     pkt.sample_count = sample_index;

//     int err = bt_gatt_notify(current_conn,
//                              &test_svc.attrs[2],
//                              &pkt,
//                              sizeof(pkt));

//     if (err) {
//         printk("Notify failed: %d\n", err);
//         return;
//     }

//     sample_index = 0;
// }

// /* =========================
//    MAIN
// ========================= */

// int main(void)
// {
//     printk("Starting BLE...\n");

//     int err = bt_enable(NULL);
//     if (err) {
//         printk("BLE init failed (%d)\n", err);
//         return 0;
//     }

//     printk("BLE ready\n");

//     printk("Before IMU init\n");

//     if (imu_init() != 0) {
//         printk("IMU init failed\n");
//         return 0;
//     }

//     printk("After IMU init\n");
//     printk("IMU ready\n");

//     k_thread_create(&imu_thread_data,
//                     imu_stack,
//                     IMU_STACK_SIZE,
//                     imu_thread,
//                     NULL, NULL, NULL,
//                     IMU_PRIORITY,
//                     0,
//                     K_NO_WAIT);

//     // 1️⃣ Først: adv_param
//     static const struct bt_le_adv_param adv_param = {
//         .id = BT_ID_DEFAULT,
//         .sid = 0,
//         .secondary_max_skip = 0,
//         .options = BT_LE_ADV_OPT_CONN,
//         .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
//         .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
//         .peer = NULL,
//     };

//     // 2️⃣ Så: advertising data
//     const struct bt_data ad[] = {
//         BT_DATA_BYTES(BT_DATA_FLAGS,
//             (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
//     };

//     const struct bt_data sd[] = {
//         BT_DATA(BT_DATA_NAME_COMPLETE,
//                 CONFIG_BT_DEVICE_NAME,
//                 sizeof(CONFIG_BT_DEVICE_NAME) - 1),
//     };

//     // 3️⃣ Så: start advertising
//     err = bt_le_adv_start(&adv_param,
//                         ad,
//                         ARRAY_SIZE(ad),
//                         sd,
//                         ARRAY_SIZE(sd));

//     if (err) {
//         printk("Advertising failed (%d)\n", err);
//         return 0;
//     }

//     printk("Advertising...\n");

//     while (1)
//     {
//         if (!current_conn || !notify_enabled) {
//             k_sleep(K_MSEC(50));
//             continue;
//         }

//         imu_ringbuffer_t *rb = imu_get_ringbuffer();

//         uint32_t idx = (rb->write_index == 0)
//                      ? (IMU_BUFFER_SIZE - 1)
//                      : (rb->write_index - 1);

//         imu_sample_t sample = rb->buffer[idx];

//         convert_sample(&sample,
//                        &pkt.samples[sample_index],
//                        seq_counter);

//         sample_index++;
//         seq_counter++;

//         if (sample_index >= BLE_IMU_SAMPLES_PER_PKT) {
//             send_packet();
//         }

//         k_sleep(K_MSEC(10)); // 100 Hz
//     }
// }
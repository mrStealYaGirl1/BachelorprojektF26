#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

// behold dine headers (men de virker ikke endnu)
#include "imu.h"
#include "swing_manager.h"
#include "ble_manager.h"

int main(void)
{
    printk("nRF FW starter\n");

    // ble_manager_init();     // starter NimBLE + advertising (connectable)
    // ble_manager_start_imu_tx_task(); // starter intern task + queue til at sende IMU data via BLE
    // imu_init();             // starter IMU + swing detection
    while (1)
    {
        printk("System kører\n");
        k_sleep(K_MSEC(1000));
    }

}
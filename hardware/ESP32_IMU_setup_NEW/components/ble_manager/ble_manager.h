#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct __attribute__((packed)) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    uint32_t ts_ms;
} ble_imu_pkt_t;

void ble_manager_init(void);

// Enkel tx task + queue til at sende simple beskeder fra swing_manager (eller andre steder)
void ble_manager_start_tx_task(void);
bool ble_manager_send_simple(uint32_t counter, uint32_t timestamp_ms);
bool ble_manager_notify_simple(uint32_t counter, uint32_t timestamp_ms);

// Fremtidige API'er til at sende IMU data (i stedet for simple beskeder)
bool ble_manager_notify_imu(const ble_imu_pkt_t *pkt);
void ble_manager_start_imu_tx_task(void);
bool ble_manager_send_imu(const ble_imu_pkt_t *pkt);
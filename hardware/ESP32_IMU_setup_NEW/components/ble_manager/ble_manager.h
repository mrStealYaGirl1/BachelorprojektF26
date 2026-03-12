#pragma once
#include <stdint.h>
#include <stdbool.h>

#define BLE_IMU_SAMPLES_PER_PKT 5

typedef struct __attribute__((packed))
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    uint32_t ts_ms;
    uint16_t seq;
} ble_imu_sample_t;

typedef struct __attribute__((packed))
{
    uint16_t event_id;
    uint16_t sample_count;   // hvor mange samples er gyldige i denne pakke
    ble_imu_sample_t samples[BLE_IMU_SAMPLES_PER_PKT];
} ble_imu_pkt_t;

void ble_manager_init(void);

// Enkel tx task + queue til at sende simple beskeder fra swing_manager (eller andre steder)
void ble_manager_start_tx_task(void);
bool ble_manager_send_simple(uint32_t counter, uint32_t timestamp_ms);
bool ble_manager_notify_simple(uint32_t counter, uint32_t timestamp_ms);

// IMU batch API
bool ble_manager_notify_imu_pkt(const ble_imu_pkt_t *pkt);
int ble_manager_notify_imu_pkt_rc(const ble_imu_pkt_t *pkt);

void ble_manager_start_imu_tx_task(void);
bool ble_manager_send_imu_pkt(const ble_imu_pkt_t *pkt);

void ble_manager_reset_notify_stats(void);
uint32_t ble_manager_get_notify_ok_count(void);
uint32_t ble_manager_get_notify_fail_count(void);
bool ble_manager_is_imu_tx_busy(void);
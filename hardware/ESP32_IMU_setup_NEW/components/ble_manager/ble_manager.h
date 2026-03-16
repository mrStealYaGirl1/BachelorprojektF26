#pragma once
#include <stdint.h>
#include <stdbool.h>

#define BLE_IMU_SAMPLES_PER_PKT 5
#define BLE_PKT_TYPE_META  1
#define BLE_PKT_TYPE_IMU   2

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
    uint16_t packet_type;
    uint16_t sample_count;
    ble_imu_sample_t samples[BLE_IMU_SAMPLES_PER_PKT];
} ble_imu_pkt_t;

typedef struct __attribute__((packed))
{
    uint16_t event_id;
    uint16_t packet_type;

    uint32_t swing_id;
    uint16_t sample_rate_hz;
    uint16_t total_samples;

    uint16_t pre_samples;
    uint16_t post_samples;
    uint16_t impact_index_in_event;

    uint64_t address_start_us;
    uint64_t backswing_start_us;
    uint64_t forward_start_us;
    uint64_t impact_us;
    uint64_t follow_start_us;
    uint64_t end_us;

    uint64_t event_start_us;
    uint64_t event_end_us;

} ble_swing_meta_pkt_t;


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

// META API
bool ble_manager_send_swing_meta(const ble_swing_meta_pkt_t *pkt);
int ble_manager_notify_swing_meta_rc(const ble_swing_meta_pkt_t *pkt);
bool ble_manager_notify_swing_meta(const ble_swing_meta_pkt_t *pkt);
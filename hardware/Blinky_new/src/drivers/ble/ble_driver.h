#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../imu/imu_driver.h"

#define BLE_PKT_TYPE_META 1
#define BLE_PKT_TYPE_IMU  2

/* =========================
   IMU DATA STRUCTURES
========================= */

/* ❌ FJERNET: #pragma pack(push, 1) */
/* ❌ FJERNET: #pragma pack(pop)     */
/* 👉 Zephyr + ARM håndterer alignment korrekt selv */

#define BLE_SAMPLES_PER_PKT 5

/* ---------------------------------
   SAMPLE STRUCT (uændret)
--------------------------------- */
typedef struct __attribute__((packed))
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    uint32_t ts_us;
    uint16_t seq;

} ble_imu_sample_t;

/* ---------------------------------
   PACKET STRUCT (RETTET)
--------------------------------- */
typedef struct __attribute__((packed))
{
    uint16_t event_id;
    uint16_t packet_type;
    uint16_t sample_count;

    ble_imu_sample_t samples[BLE_SAMPLES_PER_PKT];

} ble_imu_packet_t;

/* ---------------------------------
   META PACKET STRUCT (NY)
--------------------------------- */

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

} ble_swing_meta_packet_t;

/* =========================
   API
========================= */

void ble_init(void);
bool ble_is_stack_ready(void);
void ble_send_test(void);
//void ble_send_imu_sample(const imu_sample_t *sample);
void ble_send_imu_sample_for_event(const imu_sample_t *sample, uint16_t event_id);
void ble_post_init(void);
bool ble_queue_imu_packet(const ble_imu_packet_t *packet);
bool ble_queue_meta_packet(const ble_swing_meta_packet_t *packet);
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../imu/imu_driver.h"

/* =========================
   IMU DATA STRUCTURES
========================= */

/* ❌ FJERNET: #pragma pack(push, 1) */
/* ❌ FJERNET: #pragma pack(pop)     */
/* 👉 Zephyr + ARM håndterer alignment korrekt selv */

#define BLE_SAMPLES_PER_PKT 10

/* ---------------------------------
   SAMPLE STRUCT (uændret)
--------------------------------- */
typedef struct __packed
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

/* ---------------------------------
   PACKET STRUCT (RETTET)
--------------------------------- */
typedef struct __packed
{
    uint16_t event_id;
    uint16_t packet_type;
    uint16_t sample_count;

    ble_imu_sample_t samples[BLE_SAMPLES_PER_PKT];

} ble_imu_packet_t;

/* =========================
   API
========================= */

void ble_init(void);
bool ble_is_stack_ready(void);
void ble_send_test(void);
void ble_send_imu_sample(const imu_sample_t *sample);
void ble_post_init(void);
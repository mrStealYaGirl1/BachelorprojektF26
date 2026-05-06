#pragma once

#include <stdint.h>

/* =========================
   IMU DATA STRUCTURES
========================= */

/* ❌ FJERNET: #pragma pack(push, 1) */
/* ❌ FJERNET: #pragma pack(pop)     */
/* 👉 Zephyr + ARM håndterer alignment korrekt selv */

/* ---------------------------------
   SAMPLE STRUCT (uændret)
--------------------------------- */
typedef struct
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
typedef struct
{
    uint16_t event_id;
    uint16_t packet_type;
    uint16_t sample_count;

    /* 🔥 RETTET:
       FRA:
       ble_imu_sample_t samples[1];

       TIL:
       enkelt sample (stabil memory layout)
    */
    ble_imu_sample_t sample;

} ble_imu_packet_t;

/* =========================
   API
========================= */

void ble_init(void);
void ble_send_test(void);
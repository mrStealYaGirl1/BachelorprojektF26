#pragma once
#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"

/* =====================================================
   CONFIGURATION
===================================================== */

#define IMU_SAMPLE_RATE_HZ      250
#define IMU_TOTAL_SECONDS       8      // 5 sec pre + 3 sec post
#define IMU_BUFFER_SIZE  (IMU_SAMPLE_RATE_HZ * IMU_TOTAL_SECONDS)


/* =====================================================
   RAW IMU SAMPLE
===================================================== */

#pragma pack(push, 1)
typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    uint32_t timestamp_us;

} imu_sample_t;
#pragma pack(pop)


/* =====================================================
   RINGBUFFER
===================================================== */

typedef struct
{
    imu_sample_t buffer[IMU_BUFFER_SIZE];
    uint16_t write_index;
    uint8_t wrapped;

} imu_ringbuffer_t;


/* =====================================================
   SWING FEATURES (REAL-TIME DATA)
===================================================== */

#pragma pack(push, 1)
typedef struct
{
    uint32_t swing_id;

    float max_acc_ms2;
    float max_gyro_dps;

    float backswing_time_ms;
    float downswing_time_ms;
    float total_swing_time_ms;

    float impact_angle_deg;

} swing_features_t;
#pragma pack(pop)


/* =====================================================
   SESSION HEADER
===================================================== */

#pragma pack(push, 1)
typedef struct
{
    uint32_t magic;            // 0x474F4C46 = "GOLF"
    uint16_t version;
    uint16_t sample_rate_hz;

    uint32_t device_id;
    uint32_t session_id;

    uint16_t number_of_swings;

} session_header_t;
#pragma pack(pop)


/* =====================================================
   PUBLIC API
===================================================== */

void imu_init(void);
void imu_task(void *pvParameters);

void imu_ringbuffer_init(void);
void imu_ringbuffer_push(const imu_sample_t *sample);
imu_ringbuffer_t* imu_get_ringbuffer(void);

#endif
#pragma once
#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>


/* =====================================================
   CONFIG
===================================================== */

#define IMU_SAMPLE_RATE_HZ      200
#define IMU_TOTAL_SECONDS       6
#define IMU_BUFFER_SIZE  (IMU_SAMPLE_RATE_HZ * IMU_TOTAL_SECONDS)

/* =====================================================
   RAW SAMPLE
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

    uint64_t timestamp_us;

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
   API
===================================================== */

int imu_init(void);
void imu_thread(void *p1, void *p2, void *p3);

void imu_ringbuffer_init(void);
void imu_ringbuffer_push(const imu_sample_t *sample);
void imu_get_latest(imu_sample_t *sample);
imu_ringbuffer_t* imu_get_ringbuffer(void);

#endif
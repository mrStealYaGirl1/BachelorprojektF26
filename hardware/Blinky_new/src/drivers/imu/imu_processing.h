// imu_processing.h - Header for IMU data processing and swing detection

#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include "imu_driver.h"

void imu_processing_init(void);
void imu_processing_calibrate(void);
void imu_process_sample(const imu_sample_t *sample, uint32_t sample_idx);

#endif
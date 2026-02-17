#ifndef SWING_MANAGER_H
#define SWING_MANAGER_H

#include "imu.h"
#include <stdint.h>

void swing_manager_task(void *pvParameters);

/* Kaldes fra IMU når impact opdages */
void swing_manager_notify_impact(uint32_t index);

#endif
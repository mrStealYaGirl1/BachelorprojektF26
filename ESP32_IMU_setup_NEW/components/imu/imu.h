#pragma once
#include "freertos/FreeRTOS.h"

void imu_init(void);
void imu_task(void *pvParameters);
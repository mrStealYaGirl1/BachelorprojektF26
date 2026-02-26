#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <stdint.h>

void ble_manager_init(void);
void ble_manager_send_burst(float peak, float tempo, uint32_t duration_ms);

#endif
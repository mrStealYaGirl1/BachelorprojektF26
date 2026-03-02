#pragma once
#include <stdint.h>
#include <stdbool.h>

void ble_manager_init(void);

// Starter en intern TX-task + queue (kaldes én gang fra app_main)
void ble_manager_start_tx_task(void);

// Enqueue en simpel payload (non-blocking)
bool ble_manager_send_simple(uint32_t counter, uint32_t timestamp_ms);

//void ble_manager_send_burst(float peak, float tempo, uint32_t duration_ms);

bool ble_manager_notify_simple(uint32_t counter, uint32_t timestamp_ms);
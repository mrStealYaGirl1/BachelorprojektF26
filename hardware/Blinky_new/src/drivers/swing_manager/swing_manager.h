#pragma once
#include <stdint.h>

void swing_manager_init(void);
void swing_manager_notify_impact(uint32_t index);
void swing_manager_thread(void *p1, void *p2, void *p3);
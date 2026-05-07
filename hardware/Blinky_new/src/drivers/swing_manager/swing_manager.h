#pragma once
#include <stdint.h>

typedef struct
{
    uint32_t swing_id;

    uint64_t address_start_us;
    uint64_t backswing_start_us;
    uint64_t forward_start_us;
    uint64_t impact_us;
    uint64_t follow_start_us;
    uint64_t end_us;

    uint32_t address_start_idx;
    uint32_t backswing_start_idx;
    uint32_t forward_start_idx;
    uint32_t impact_idx;
    uint32_t follow_start_idx;
    uint32_t end_idx;

} swing_timing_t;

void swing_manager_init(void);
void swing_manager_notify_impact(uint32_t index, uint64_t impact_us);
void swing_manager_add_swing(swing_timing_t swing);
void swing_manager_thread(void *p1, void *p2, void *p3);
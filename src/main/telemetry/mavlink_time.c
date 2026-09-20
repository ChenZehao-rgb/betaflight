#include "telemetry/mavlink_time.h"

uint64_t mavlinkClockUpdate(mavlinkClock_t *clock, uint32_t current)
{
    clock->total += (uint32_t)(current - clock->previous);
    clock->previous = current;
    return clock->total;
}

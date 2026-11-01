#include "telemetry/mavlink_time.h"

#ifdef SIMULATOR_BUILD
#include "platform.h"
#endif

uint64_t mavlinkClockUpdate(mavlinkClock_t *clock, uint32_t current)
{
    clock->total += (uint32_t)(current - clock->previous);
    clock->previous = current;
    return clock->total;
}

uint64_t mavlinkTimeUs(uint32_t current)
{
#ifdef SIMULATOR_BUILD
    // State snapshots, observation ages and TIMESYNC share host monotonic
    // uptime. The physics clock changes rate with FDM packet arrival jitter
    // and is not a stable oscillator for synchronization with the companion.
    (void)current;
    return micros64_real();
#else
    static mavlinkClock_t clock;
    return mavlinkClockUpdate(&clock, current);
#endif
}

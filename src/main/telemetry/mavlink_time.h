#pragma once
#include <stdint.h>

// Cooperative-task clock; sample at least once every 2^32 microseconds.
// This tracks device uptime, not UTC, and must not be called concurrently.
typedef struct mavlinkClock_s {
    uint32_t previous;
    uint64_t total;
} mavlinkClock_t;

uint64_t mavlinkClockUpdate(mavlinkClock_t *clock, uint32_t current);

// One shared cooperative-task uptime epoch for state production and TIMESYNC.
uint64_t mavlinkTimeUs(uint32_t current);

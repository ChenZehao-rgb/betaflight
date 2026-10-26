#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "flight/position_estimator.h"
#include "telemetry/mavlink.h"

#define BF_ESTIMATOR_HEALTH_TYPE 0xBF01
#define BF_ESTIMATOR_HEALTH_LENGTH 68
#define BF_ODOMETRY_INTERVAL_MS 10
#define BF_HEALTH_INTERVAL_MS 20
#define BF_STATE_MAX_AGE_US 20000

struct __mavlink_status;

void mavlinkPackOdometry(mavlink_message_t *msg, struct __mavlink_status *status, const fcStateSnapshot_t *state);
void mavlinkPackEstimatorHealth(mavlink_message_t *msg, struct __mavlink_status *status, const fcStateSnapshot_t *state);
// Advance from the previous deadline without accumulating jitter or catching up
// missed frames. Works across millis wrap; interval is <= INT32_MAX milliseconds.
bool mavlinkStreamDue(uint32_t now, uint32_t *deadline, uint32_t interval);

/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include "common/time.h"

#define MIN_MAVLINK_TELEMETRY_UPDATE_INTERVAL_MS 10

void initMAVLinkTelemetry(void);
void handleMAVLinkTelemetry(void);
void checkMAVLinkTelemetryState(void);

void freeMAVLinkTelemetryPort(void);
void configureMAVLinkTelemetryPort(void);

// Forward-typedef so consumers can hold pointers without dragging in
// common/mavlink.h (which carries -Wpedantic-noisy unnamed unions).
typedef struct __mavlink_message mavlink_message_t;

// Pack a fully-formed mavlink_message_t into the shared TX buffer and write it
// to the open MAVLink serial port. Implemented in telemetry/mavlink.c; shared
// with telemetry/mavlink_mission.c so the mission module reuses the same
// buffer/port path.
void mavlinkSendMessage(mavlink_message_t *msg);

typedef struct mavlinkTelemetryOutputMessage_s {
    const uint32_t id;
    const uint8_t stream;
    timeMs_t updateInterval;
    timeMs_t updateTime;
    bool (*const sendMessageFunc)(void); // false: no new state yet; keep the deadline pending
} mavlinkTelemetryOutputMessage_t;

// Inspect with debugger; counters include nonblocking whole-frame TX drops.
typedef struct mavlinkTelemetryStats_s {
    uint32_t sentFrames, droppedFrames;
    uint32_t odometryFrames, healthFrames;
    uint32_t noNewState, staleStates;
    uint32_t lastStateLatencyUs, maxStateLatencyUs;
} mavlinkTelemetryStats_t;
const mavlinkTelemetryStats_t *mavlinkGetTelemetryStats(void);

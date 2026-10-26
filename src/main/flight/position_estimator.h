/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/axis.h"
#include "common/vector.h"
#include "common/time.h"

#include "io/gps.h"

// Unified position estimate from Kalman-filter sensor fusion.
// All values in local ENU (East-North-Up) centimeters, zeroed at arm point.
typedef struct positionEstimate3d_s {
    vector3_t position;        // cm, ENU
    vector3_t velocity;        // cm/s, ENU
    float trustXY;             // 0-1, derived from KF XY covariance
    float trustZ;              // 0-1, derived from KF Z covariance
    bool isValidXY;            // true if at least one XY measurement source active
    bool isValidZ;             // true if at least one Z measurement source active
} positionEstimate3d_t;

void positionEstimatorInit(void);
void positionEstimatorUpdate(void);

// Request XY fusion (normally automatic from armed state / sensors / modes).
// May re-initialize kfEast/kfNorth when transitioning to enabled; disabling stops prediction only.
void positionEstimatorEnableXY(bool enable);

// Read the unified estimate
const positionEstimate3d_t *positionEstimatorGetEstimate(void);

float positionEstimatorGetAltitudeCm(void);
float positionEstimatorGetAltitudeDerivative(void);
bool positionEstimatorIsValidXY(void);
bool positionEstimatorIsValidZ(void);
float positionEstimatorGetTrustXY(void);
// True when GPS has a fix and is not excluded by positionSource config.
// Use to decide whether heading validity is required before engaging position hold.
bool positionEstimatorIsHeadingRequired(void);

// Reset (e.g. on arm/disarm)
void positionEstimatorResetZ(void);
void positionEstimatorResetXY(void);

// Returns the GPS location captured when XY estimation became active (arm point).
// Returns false if no GPS origin has been captured yet.
bool positionEstimatorGetGpsOrigin(gpsLocation_t *out);

// Latest completed estimator update. ENU SI position/velocity; raw IMU
// quaternion rotates BF body to NWU; gyro is filtered BF body degrees/s.
// Telemetry must only copy this snapshot, never resample individual sensors.
typedef struct fcStateSnapshot_s {
    uint64_t timeUs;
    uint32_t sequence;
    uint32_t resetCounter;
    float position[3];
    float velocity[3];
    float quaternion[4];
    float bodyRates[3];
    float positionVariance[3];
    float velocityVariance[3];
    float trustXY;
    float trustZ;
    uint32_t xyObservationAgeUs;
    uint32_t zObservationAgeUs;
    uint32_t sourceFlags; // Observations incorporated in this update; see protocol.
    uint8_t gpsFixType;
    bool xyValid;
    bool zValid;
    bool headingValid;
    bool magHeadingFresh;
    bool armed;
    bool gpsFix;
    bool xyEnabled;
    bool present;
} fcStateSnapshot_t;

// Cooperative scheduler only: producer and reader must not run concurrently.
const fcStateSnapshot_t *positionEstimatorGetSnapshot(void);

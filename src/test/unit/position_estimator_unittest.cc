#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "pg/pg_ids.h"

#include "common/maths.h"
#include "common/vector.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "flight/position_estimator.h"
#include "telemetry/mavlink_odometry.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

// STATIC_UNIT_TESTED in position_estimator.c — gravity-removed earth-frame
// linear acceleration in ENU (cm/s^2).
void getLinearAccelENU(float *accelEast, float *accelNorth, float *accelUp);

#include "io/gps.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"

PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);
}

#include "gtest/gtest.h"

extern "C" {
uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;
uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];

acc_t acc;
gyro_t gyro;
void getQuaternion(quaternion_t *q) { *q = (quaternion_t)QUATERNION_INITIALIZE; }
bool imuIsHeadingValid(void) { return true; }
bool imuIsMagHeadingValid(void) { return false; }
matrix33_t rMat;
gpsSolutionData_t gpsSol;

static uint32_t enabledSensors = 0;
static bool rfHealthy = false;
static float rfAltCm = 0.0f;
static float baroAltCm = 0.0f;
static timeUs_t fakeMicros = 0;
static bool newGpsData = true;

bool sensors(uint32_t mask) { return (enabledSensors & mask) != 0; }
bool rangefinderIsHealthy(void) { return rfHealthy; }
int32_t rangefinderGetLatestAltitude(void) { return lrintf(rfAltCm); }
float getBaroAltitude(void) { return baroAltCm; }

timeUs_t micros(void) { return fakeMicros; }

bool gpsHasNewData(uint16_t *gpsStamp)
{
    if (!newGpsData) return false;
    (*gpsStamp)++;
    return true;
}

void GPS_distance2d(const gpsLocation_t *from, const gpsLocation_t *to, vector2_t *dest)
{
    // Simplified (no Earth-scale projection): return integer-unit deltas so sign tests work.
    dest->x = (float)(to->lon - from->lon);
    dest->y = (float)(to->lat - from->lat);
}
}

static void stepEstimator(unsigned count = 1)
{
    for (unsigned i = 0; i < count; i++) {
        fakeMicros += 10000; // 100 Hz
        positionEstimatorUpdate();
    }
}

class PositionEstimatorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(&acc, 0, sizeof(acc));
        memset(&rMat, 0, sizeof(rMat));
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(debug, 0, sizeof(debug));

        // Identity rotation and 1G reciprocal scale so zero accel stays zero.
        rMat.m[NWU_N][X] = 1.0f;
        rMat.m[NWU_W][Y] = 1.0f;
        rMat.m[NWU_U][Z] = 1.0f;
        acc.dev.acc_1G_rec = 1.0f;
        acc.accADC.z = 1.0f;

        enabledSensors = SENSOR_GPS | SENSOR_BARO | SENSOR_RANGEFINDER;
        rfHealthy = true;
        rfAltCm = 100.0f;
        baroAltCm = 100.0f;
        gpsSol.llh.altCm = 100.0f;
        gpsSol.dop.pdop = 100; // pDOP 1.0

        stateFlags = GPS_FIX;
        armingFlags = ARMED;
        fakeMicros = 0;
        newGpsData = true;

        positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_RANGEFINDER_PREFER;
        positionConfigMutable()->altitude_prefer_baro = 100;
        positionConfigMutable()->rangefinder_max_range_cm = 400;

        positionEstimatorInit();
    }
};

TEST_F(PositionEstimatorTest, RangefinderPreferFallsBackAndRecovers)
{
    // Establish offsets/baseline near zero.
    stepEstimator(10);
    const float baseline = positionEstimatorGetAltitudeCm();

    // Rangefinder unavailable: fallback should use baro/GPS updates.
    rfHealthy = false;
    baroAltCm = 150.0f;
    gpsSol.llh.altCm = 150.0f;
    stepEstimator(40);
    const float fallbackAltitude = positionEstimatorGetAltitudeCm();
    EXPECT_GT(fallbackAltitude, baseline + 10.0f);

    // Rangefinder becomes available again at a lower relative altitude.
    rfHealthy = true;
    rfAltCm = 120.0f; // +20cm relative to RF offset baseline
    stepEstimator(40);
    const float recoveredAltitude = positionEstimatorGetAltitudeCm();

    // In RANGEFINDER_PREFER, recovered altitude should be pulled down toward RF value.
    EXPECT_LT(recoveredAltitude, fallbackAltitude - 5.0f);
    EXPECT_TRUE(positionEstimatorIsValidZ());
}

// Regression test: GPS_distance2d must be called as (arm, current) so that a craft
// East of the arm position produces a positive kfEast (East) position estimate.
// The bug had the arguments reversed — (current, arm) — giving a negative East estimate.
TEST_F(PositionEstimatorTest, GPSPositionEastOfArmIsPositive)
{
    // Run a couple of steps so the arm location is captured from the initial gpsSol (lon=0).
    stepEstimator(2);

    // Place the craft East of the arm (positive longitude delta).
    gpsSol.llh.lon = 100000;

    // Let the Kalman filter converge toward the GPS position measurement.
    stepEstimator(30);

    // position is ENU; index by ENU_E so "East" is explicit rather than ".x".
    EXPECT_GT(positionEstimatorGetEstimate()->position.v[ENU_E], 0.0f);
}

// Build rMat from roll/pitch/yaw using the exact quaternion + rotation-matrix
// formulas from imu.c (imuComputeQuaternionFromRPY + imuComputeRotationMatrix),
// so the matrix carries the firmware's body->earth NWU (North-West-Up) convention.
// Yaw follows the attitude/compass convention: 0 = North, +90 = East increases the
// reported heading, so a craft heading East is built with yawDeg = -90.
static void setRMatFromEuler(float rollDeg, float pitchDeg, float yawDeg)
{
    const float hr = DEGREES_TO_RADIANS(rollDeg) * 0.5f;
    const float hp = DEGREES_TO_RADIANS(pitchDeg) * 0.5f;
    const float hy = DEGREES_TO_RADIANS(yawDeg) * 0.5f;

    const float cr = cosf(hr), sr = sinf(hr);
    const float cp = cosf(hp), sp = sinf(hp);
    const float cy = cosf(hy), sy = sinf(hy);

    const float q0 = cr * cp * cy + sr * sp * sy;
    const float q1 = sr * cp * cy - cr * sp * sy;
    const float q2 = cr * sp * cy + sr * cp * sy;
    const float q3 = cr * cp * sy - sr * sp * cy;

    const float xx = q1 * q1, yy = q2 * q2, zz = q3 * q3;
    const float xy = q1 * q2, xz = q1 * q3, yz = q2 * q3;
    const float wx = q0 * q1, wy = q0 * q2, wz = q0 * q3;

    rMat.m[NWU_N][X] = 1.0f - 2.0f * yy - 2.0f * zz;
    rMat.m[NWU_N][Y] = 2.0f * (xy - wz);
    rMat.m[NWU_N][Z] = 2.0f * (xz + wy);
    rMat.m[NWU_W][X] = 2.0f * (xy + wz);
    rMat.m[NWU_W][Y] = 1.0f - 2.0f * xx - 2.0f * zz;
    rMat.m[NWU_W][Z] = 2.0f * (yz - wx);
    rMat.m[NWU_U][X] = 2.0f * (xz - wy);
    rMat.m[NWU_U][Y] = 2.0f * (yz + wx);
    rMat.m[NWU_U][Z] = 1.0f - 2.0f * xx - 2.0f * yy;
}

// Regression test for the East-West / magnitude bug discussed in #15321 and #15339.
// getLinearAccelENU must use matrixVectorMul (body->earth NWU) with East = -West;
// the previously-used matrixTrnVectorMul applied the inverse rotation, giving the
// wrong magnitude and a spurious North term.
//
// Scenario: heading East, 30 deg right roll, 0.5 g of forward (East) thrust.
// Gravity projects onto body Y and Z because of the roll, so accBF = (0.5, 0.5, 0.866).
// In the correct NWU formulation the gravity cross-terms cancel exactly, leaving
// the full 0.5 g of thrust on East and nothing on North or Up.
TEST_F(PositionEstimatorTest, EastThrustProducesEastAccelENU)
{
    constexpr float GRAVITY_CMSS = 980.665f;

    // rMat produced via imuComputeRotationMatrix's convention for heading East + 30 deg roll.
    setRMatFromEuler(30.0f, 0.0f, -90.0f);

    // Forward (East) thrust of 0.5 g; gravity projects onto body Y and Z due to the roll.
    acc.dev.acc_1G_rec = 1.0f;
    acc.accADC.x = 0.5f;
    acc.accADC.y = 0.5f;    // sin(30) * 1g
    acc.accADC.z = 0.866f;  // cos(30) * 1g

    float accelEast, accelNorth, accelUp;
    getLinearAccelENU(&accelEast, &accelNorth, &accelUp);

    EXPECT_NEAR(accelEast,  0.5f * GRAVITY_CMSS, 0.01f * GRAVITY_CMSS);
    EXPECT_NEAR(accelNorth, 0.0f,                0.01f * GRAVITY_CMSS);
    EXPECT_NEAR(accelUp,    0.0f,                0.01f * GRAVITY_CMSS);
}

// Integration counterpart: with the same attitude/thrust, the IMU prediction alone
// (GPS held at the origin) must drive a positive East velocity estimate.
TEST_F(PositionEstimatorTest, EastThrustProducesPositiveEastVelocity)
{
    // Run a couple of steps so XY fusion is established with the arm at origin.
    stepEstimator(2);

    setRMatFromEuler(30.0f, 0.0f, -90.0f);

    acc.accADC.x = 0.5f;
    acc.accADC.y = 0.5f;    // sin(30) * 1g
    acc.accADC.z = 0.866f;  // cos(30) * 1g

    // GPS position and velocity remain zero so any non-zero velocity estimate is
    // driven purely by the IMU prediction.
    stepEstimator(100);

    // velocity is ENU; index by ENU_E so "East" is explicit rather than ".x".
    EXPECT_GT(positionEstimatorGetEstimate()->velocity.v[ENU_E], 0.0f);
}


TEST_F(PositionEstimatorTest, SnapshotMatchesCompletedFusionAndDoesNotResample)
{
    stepEstimator(2);
    gpsSol.llh.lon = 1200;
    gpsSol.velned.velE = 150;
    stepEstimator(1);
    const auto *s = positionEstimatorGetSnapshot();
    const auto *e = positionEstimatorGetEstimate();
    ASSERT_TRUE(s->present);
    for (int i=0; i<3; ++i) {
        EXPECT_FLOAT_EQ(e->position.v[i]*.01f, s->position[i]);
        EXPECT_FLOAT_EQ(e->velocity.v[i]*.01f, s->velocity[i]);
        EXPECT_GT(s->positionVariance[i], 0);
        EXPECT_GT(s->velocityVariance[i], 0);
    }
    mavlink_status_t status = {};
    mavlink_message_t message;
    mavlinkPackOdometry(&message, &status, s);
    mavlink_odometry_t wire;
    mavlink_msg_odometry_decode(&message, &wire);
    EXPECT_FLOAT_EQ(e->position.v[ENU_N]*.01f, wire.x);
    EXPECT_FLOAT_EQ(e->position.v[ENU_E]*.01f, wire.y);
    EXPECT_FLOAT_EQ(-e->position.v[ENU_U]*.01f, wire.z);
    EXPECT_EQ(s->timeUs, wire.time_usec);
    // A real KF posterior need not equal the raw GPS position/velocity.
    EXPECT_NE(gpsSol.llh.lon*.01f,s->position[ENU_E]);
    const auto stamp=s->timeUs; const auto sequence=s->sequence;
    fakeMicros+=1234;
    EXPECT_EQ(stamp,positionEstimatorGetSnapshot()->timeUs);
    EXPECT_EQ(sequence,positionEstimatorGetSnapshot()->sequence);
    stepEstimator();
    EXPECT_EQ(stamp+11234,positionEstimatorGetSnapshot()->timeUs);
}

TEST_F(PositionEstimatorTest, ObservationOutageAgeTimeoutAndRecovery)
{
    stepEstimator(2);
    auto reset=positionEstimatorGetSnapshot()->resetCounter;
    EXPECT_EQ(0u,positionEstimatorGetSnapshot()->xyObservationAgeUs);
    newGpsData=false;
    stepEstimator(150);
    EXPECT_EQ(1500000u,positionEstimatorGetSnapshot()->xyObservationAgeUs);
    EXPECT_TRUE(positionEstimatorGetSnapshot()->xyValid); // Existing 2s validity retained.
    stepEstimator(51);
    EXPECT_FALSE(positionEstimatorGetSnapshot()->xyValid);
    EXPECT_EQ(2010000u,positionEstimatorGetSnapshot()->xyObservationAgeUs);
    newGpsData=true; stepEstimator();
    EXPECT_TRUE(positionEstimatorGetSnapshot()->xyValid);
    EXPECT_EQ(0u,positionEstimatorGetSnapshot()->xyObservationAgeUs);
    EXPECT_EQ(reset,positionEstimatorGetSnapshot()->resetCounter); // Same origin.
    stateFlags=0; stepEstimator();
    EXPECT_FALSE(positionEstimatorGetSnapshot()->xyEnabled);
    stateFlags=GPS_FIX; stepEstimator();
    EXPECT_EQ(reset+1,positionEstimatorGetSnapshot()->resetCounter); // XY restart + immediate origin = one event.
}

TEST_F(PositionEstimatorTest, ResetEventsAndLateGpsOriginCountExactlyOnce)
{
    const auto initial=positionEstimatorGetSnapshot()->resetCounter;
    EXPECT_FALSE(positionEstimatorGetSnapshot()->present);
    stepEstimator();
    EXPECT_EQ(initial+1,positionEstimatorGetSnapshot()->resetCounter);
    positionEstimatorResetXY();
    EXPECT_FALSE(positionEstimatorGetSnapshot()->present);
    stepEstimator();
    EXPECT_EQ(initial+2,positionEstimatorGetSnapshot()->resetCounter);
    positionEstimatorResetZ(); stepEstimator();
    EXPECT_EQ(initial+3,positionEstimatorGetSnapshot()->resetCounter);
    positionEstimatorInit();
    EXPECT_EQ(initial+4,positionEstimatorGetSnapshot()->resetCounter);
    stateFlags=0;
    positionEstimatorEnableXY(true); // Reinitialize with no GPS origin available.
    EXPECT_EQ(initial+5,positionEstimatorGetSnapshot()->resetCounter);
    stateFlags=GPS_FIX; stepEstimator(); // Late origin now captured, distinct event.
    EXPECT_EQ(initial+6,positionEstimatorGetSnapshot()->resetCounter);
    stepEstimator();
    EXPECT_EQ(initial+6,positionEstimatorGetSnapshot()->resetCounter);
}

TEST_F(PositionEstimatorTest, DisarmedXYInvalidAndNeverObservedAgesUnknown)
{
    armingFlags=0; enabledSensors=0;
    stepEstimator();
    const auto *s=positionEstimatorGetSnapshot();
    EXPECT_FALSE(s->xyValid); EXPECT_FALSE(s->xyEnabled); EXPECT_FALSE(s->armed);
    EXPECT_EQ(UINT32_MAX,s->xyObservationAgeUs);
    EXPECT_EQ(UINT32_MAX,s->zObservationAgeUs);
    EXPECT_EQ(0u,s->sourceFlags);
    EXPECT_FLOAT_EQ(1.0f,s->positionVariance[ENU_E]); // 10000 cm^2 -> 1 m^2.
    EXPECT_FLOAT_EQ(1.0f,s->velocityVariance[ENU_E]);
}

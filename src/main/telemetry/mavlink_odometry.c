#include <math.h>
#include <string.h>
#include "platform.h"
#include "common/float_bits.h"
#include "common/utils.h"
#include "telemetry/mavlink_odometry.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

bool mavlinkStreamDue(uint32_t now, uint32_t *deadline, uint32_t interval)
{
    if (interval == UINT32_MAX || (int32_t)(now - *deadline) < 0) {
        return false;
    }
    *deadline += ((now - *deadline) / interval + 1) * interval;
    return true;
}

void mavlinkPackOdometry(mavlink_message_t *msg, struct __mavlink_status *status, const fcStateSnapshot_t *s)
{
    // BF body -> NWU becomes FRD -> NED by applying diag(1,-1,-1)
    // on both sides. Equivalent to legacy ATTITUDE (roll,-pitch,yaw).
    float q[4] = {s->quaternion[0], s->quaternion[1], -s->quaternion[2], -s->quaternion[3]};
    float norm = 0;
    for (unsigned i = 0; i < 4; ++i) {
        norm += q[i] * q[i];
    }
    bool usable = s->present && s->xyValid && s->zValid && s->headingValid;
    if (!float32IsFinite(norm) || norm < 1e-12f) {
        usable = false;
        for (unsigned i = 0; i < 4; ++i) {
            q[i] = NAN;
        }
    } else {
        const float invNorm = 1.0f / sqrtf(norm);
        for (unsigned i = 0; i < 4; ++i) {
            q[i] *= invNorm;
        }
    }
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float ned[3] = {s->velocity[1], s->velocity[0], -s->velocity[2]};
    // R_NED_FRD transpose * v_NED, because child_frame_id governs velocity.
    const float body[3] = {
        (1-2*(y*y+z*z))*ned[0] + 2*(x*y+w*z)*ned[1] + 2*(x*z-w*y)*ned[2],
        2*(x*y-w*z)*ned[0] + (1-2*(x*x+z*z))*ned[1] + 2*(y*z+w*x)*ned[2],
        2*(x*z+w*y)*ned[0] + 2*(y*z-w*x)*ned[1] + (1-2*(x*x+y*y))*ned[2]
    };
    for (unsigned i = 0; i < 3; ++i) {
        usable &= float32IsFinite(s->position[i]) && float32IsFinite(body[i]) && float32IsFinite(s->bodyRates[i]);
    }
    // No attitude/rate covariance exists. MAVLink marks the complete covariance
    // unknown with NaN at element 0. Actual ENU translational diagonals are in health.
    float unknown[21];
    for (unsigned i = 0; i < 21; ++i) {
        unknown[i] = NAN;
    }
    const float rad = 0.017453292519943295f;
    status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    mavlink_msg_odometry_pack_status(1, MAV_COMP_ID_AUTOPILOT1, status, msg,
        s->timeUs, MAV_FRAME_LOCAL_NED, MAV_FRAME_BODY_FRD,
        s->position[1], s->position[0], -s->position[2], q,
        body[0], body[1], body[2],
        s->bodyRates[0]*rad, -s->bodyRates[1]*rad, -s->bodyRates[2]*rad,
        unknown, unknown, (uint8_t)s->resetCounter, MAV_ESTIMATOR_TYPE_AUTOPILOT, usable ? 0 : -1);
}

static void putLE(uint8_t *p, uint64_t value, unsigned bytes)
{
    for (unsigned i = 0; i < bytes; ++i) {
        p[i] = (uint8_t)(value >> (8*i));
    }
}

static void putFloat(uint8_t *p, float value)
{
    uint32_t bits;
    STATIC_ASSERT(sizeof(float) == sizeof(bits), float32_required);
    memcpy(&bits, &value, sizeof(bits));
    putLE(p, bits, sizeof(bits));
}

void mavlinkPackEstimatorHealth(mavlink_message_t *msg, struct __mavlink_status *status, const fcStateSnapshot_t *s)
{
    uint8_t payload[249] = {0};
    payload[0] = 1;
    payload[1] = BF_ESTIMATOR_HEALTH_LENGTH;
    const uint16_t flags = s->present ?
        (s->xyValid | s->zValid << 1 | s->headingValid << 2 | s->magHeadingFresh << 3 |
         s->armed << 4 | s->gpsFix << 5 | s->xyEnabled << 6 | 1 << 7) : 0;
    putLE(payload+2, flags, 2);
    putLE(payload+4, s->sequence, 4);
    putLE(payload+8, s->resetCounter, 4);
    putLE(payload+12, s->timeUs, 8);
    putLE(payload+20, s->present ? s->xyObservationAgeUs : UINT32_MAX, 4);
    putLE(payload+24, s->present ? s->zObservationAgeUs : UINT32_MAX, 4);
    putFloat(payload+28, s->present ? s->trustXY : NAN);
    putFloat(payload+32, s->present ? s->trustZ : NAN);
    putLE(payload+36, s->present ? s->sourceFlags : 0, 4);
    payload[40] = s->present ? s->gpsFixType : 0;
    for (unsigned i = 0; i < 3; ++i) {
        const float p = s->positionVariance[i], v = s->velocityVariance[i];
        putFloat(payload+44+4*i, s->present && float32IsFinite(p) && p >= 0 ? p : NAN);
        putFloat(payload+56+4*i, s->present && float32IsFinite(v) && v >= 0 ? v : NAN);
    }
    status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    mavlink_msg_v2_extension_pack_status(1, MAV_COMP_ID_AUTOPILOT1, status, msg, 0, 0, 0,
        BF_ESTIMATOR_HEALTH_TYPE, payload);
}

#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <vector>
extern "C" {
#include "platform.h"
#include "drivers/serial.h"
#include "telemetry/mavlink_odometry.h"
#include "telemetry/mavlink_time.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop
void mavlinkTestInit(serialPort_t *port);
void mavlinkTestProcess(void);
bool mavlinkTestSetInterval(uint32_t id, uint32_t ms);
void mavlinkTestTimesync(const mavlink_message_t *msg);
}

static uint32_t nowUs, freeBytes;
static fcStateSnapshot_t state;
static std::vector<std::vector<uint8_t>> writes;
extern "C" {
timeUs_t micros(void) { return nowUs; }
timeMs_t millis(void) { return nowUs / 1000; }
uint32_t serialTxBytesFree(const serialPort_t *) { return freeBytes; }
void serialWriteBuf(serialPort_t *, const uint8_t *data, int count)
{
    EXPECT_LE(unsigned(count), freeBytes);
    writes.emplace_back(data, data + count);
    freeBytes -= count;
}
const fcStateSnapshot_t *positionEstimatorGetSnapshot(void) { return &state; }
}

static fcStateSnapshot_t sample()
{
    fcStateSnapshot_t s = {};
    s.present = s.xyValid = s.zValid = s.headingValid = true;
    s.armed = s.gpsFix = s.xyEnabled = true;
    s.timeUs = 0x123456789ULL;
    s.sequence = 0x10203040;
    s.resetCounter = 0x1234;
    s.quaternion[0] = 1;
    s.position[0] = 12; s.position[1] = 23; s.position[2] = 34;
    s.velocity[0] = 2; s.velocity[1] = 3; s.velocity[2] = 4;
    s.bodyRates[0] = 90; s.bodyRates[1] = 180; s.bodyRates[2] = -45;
    s.xyObservationAgeUs = 123456;
    s.zObservationAgeUs = UINT32_MAX;
    s.sourceFlags = 17;
    s.gpsFixType = 3;
    s.trustXY = .25f; s.trustZ = .5f;
    for (int i = 0; i < 3; ++i) {
        s.positionVariance[i] = 1.25f + i;
        s.velocityVariance[i] = 2.5f + i;
    }
    return s;
}

static mavlink_message_t decode(const std::vector<uint8_t> &wire)
{
    EXPECT_EQ(0xfd, wire[0]); // Actual serialized bytes, not just message ID.
    mavlink_message_t parser = {}, msg = {};
    mavlink_status_t status = {}, result = {};
    unsigned frames = 0;
    for (auto byte : wire) {
        frames += mavlink_frame_char_buffer(&parser, &status, byte, &msg, &result) == MAVLINK_FRAMING_OK;
    }
    EXPECT_EQ(1u, frames);
    EXPECT_EQ(0, result.parse_error);
    return msg;
}

static mavlink_odometry_t roundtrip(fcStateSnapshot_t s)
{
    mavlink_message_t msg;
    mavlink_status_t status = {};
    status.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1; // Must actively force v2.
    mavlinkPackOdometry(&msg, &status, &s);
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const auto size = mavlink_msg_to_send_buffer(bytes, &msg);
    auto received = decode(std::vector<uint8_t>(bytes, bytes+size));
    EXPECT_EQ(331u, received.msgid);
    mavlink_odometry_t out;
    mavlink_msg_odometry_decode(&received, &out);
    return out;
}

TEST(MavlinkOdometry, WireUnitsFramesAndUnknownCovariance)
{
    auto s = sample();
    auto o = roundtrip(s);
    EXPECT_EQ(MAV_FRAME_LOCAL_NED, o.frame_id);
    EXPECT_EQ(MAV_FRAME_BODY_FRD, o.child_frame_id);
    EXPECT_EQ(s.timeUs, o.time_usec);
    EXPECT_FLOAT_EQ(23, o.x); EXPECT_FLOAT_EQ(12, o.y); EXPECT_FLOAT_EQ(-34, o.z);
    EXPECT_FLOAT_EQ(3, o.vx); EXPECT_FLOAT_EQ(2, o.vy); EXPECT_FLOAT_EQ(-4, o.vz);
    EXPECT_NEAR(M_PI/2, o.rollspeed, 1e-6);
    EXPECT_NEAR(-M_PI, o.pitchspeed, 1e-6);
    EXPECT_NEAR(M_PI/4, o.yawspeed, 1e-6);
    EXPECT_EQ(0x34, o.reset_counter);
    EXPECT_EQ(0, o.quality);
    EXPECT_TRUE(std::isnan(o.pose_covariance[0]));
    EXPECT_TRUE(std::isnan(o.velocity_covariance[0]));
}

TEST(MavlinkOdometry, FixedHeadingsAndBodyVelocity)
{
    auto s = sample();
    s.velocity[0] = 5; s.velocity[1] = s.velocity[2] = 0;
    // East heading: internal yaw -90, wire yaw +90. East motion is forward.
    s.quaternion[0] = std::sqrt(.5f); s.quaternion[3] = -std::sqrt(.5f);
    auto o = roundtrip(s);
    EXPECT_NEAR(5, o.vx, 1e-5); EXPECT_NEAR(0, o.vy, 1e-5); EXPECT_NEAR(0, o.vz, 1e-5);
    EXPECT_NEAR(std::sqrt(.5f), o.q[3], 1e-6);
    // South heading, East motion is left.
    s.quaternion[0] = 0; s.quaternion[3] = -1;
    o = roundtrip(s);
    EXPECT_NEAR(0, o.vx, 1e-5); EXPECT_NEAR(-5, o.vy, 1e-5);
    // West heading, East motion is backwards.
    s.quaternion[0] = std::sqrt(.5f); s.quaternion[3] = std::sqrt(.5f);
    o = roundtrip(s);
    EXPECT_NEAR(-5, o.vx, 1e-5); EXPECT_NEAR(0, o.vy, 1e-5);
}

TEST(MavlinkOdometry, RollPitchAndRosFluRecovery)
{
    auto s = sample();
    // BF roll +90 -> FRD roll +90; Up velocity becomes body left.
    s.quaternion[0] = s.quaternion[1] = std::sqrt(.5f);
    s.velocity[0] = s.velocity[1] = 0; s.velocity[2] = 1;
    auto o = roundtrip(s);
    EXPECT_NEAR(std::sqrt(.5f), o.q[1], 1e-6);
    EXPECT_NEAR(-1, o.vy, 1e-5); EXPECT_NEAR(0, o.vz, 1e-5);
    // BF pitch +90 -> FRD pitch -90; Up velocity is backwards.
    s.quaternion[1] = 0; s.quaternion[2] = std::sqrt(.5f);
    o = roundtrip(s);
    EXPECT_NEAR(-std::sqrt(.5f), o.q[2], 1e-6);
    EXPECT_NEAR(-1, o.vx, 1e-5); EXPECT_NEAR(0, o.vz, 1e-5);
    // Existing ROS receiver's NED/FRD -> ENU/FLU quaternion transform.
    s = sample(); o = roundtrip(s);
    const float k = std::sqrt(.5f);
    const float rosQ[] = {k*(o.q[0]+o.q[3]), k*(o.q[1]+o.q[2]),
                         k*(o.q[1]-o.q[2]), k*(o.q[0]-o.q[3])};
    EXPECT_NEAR(k, rosQ[0], 1e-6); EXPECT_NEAR(k, rosQ[3], 1e-6); // North = ENU yaw +90.
    EXPECT_FLOAT_EQ(s.velocity[1], o.vx);
    EXPECT_FLOAT_EQ(-s.velocity[0], -o.vy); // ROS FLU left = -FRD right.
}

TEST(MavlinkOdometry, NormalizesAndRejectsUnavailableState)
{
    auto s = sample(); s.quaternion[0] = 2;
    auto o = roundtrip(s); EXPECT_FLOAT_EQ(1, o.q[0]);
    s.quaternion[0] = 0; o = roundtrip(s);
    EXPECT_EQ(-1, o.quality); EXPECT_TRUE(std::isnan(o.q[0]));
    s = sample(); s.xyValid = false;
    EXPECT_EQ(-1, roundtrip(s).quality);
    s = sample(); s.position[0] = NAN;
    EXPECT_EQ(-1, roundtrip(s).quality);
    s = sample(); s.quaternion[1] = INFINITY;
    EXPECT_EQ(-1, roundtrip(s).quality);
    s = sample(); s.bodyRates[2] = NAN;
    EXPECT_EQ(-1, roundtrip(s).quality);
}

static uint64_t le(const uint8_t *p, unsigned n)
{
    uint64_t value = 0;
    for (unsigned i = 0; i < n; ++i) value |= uint64_t(p[i]) << (8*i);
    return value;
}
static float leFloat(const uint8_t *p)
{
    uint32_t bits = le(p, 4); float f; memcpy(&f, &bits, 4); return f;
}
TEST(MavlinkHealth, VersionedLittleEndianLayoutAndUnknowns)
{
    auto s = sample(); mavlink_status_t status = {}; mavlink_message_t msg;
    mavlinkPackEstimatorHealth(&msg, &status, &s);
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN]; auto n = mavlink_msg_to_send_buffer(bytes, &msg);
    auto received = decode(std::vector<uint8_t>(bytes, bytes+n));
    mavlink_v2_extension_t h; mavlink_msg_v2_extension_decode(&received, &h);
    EXPECT_EQ(0xBF01, h.message_type);
    auto p = h.payload;
    EXPECT_EQ(1, p[0]); EXPECT_EQ(68, p[1]); EXPECT_EQ(0xf7, le(p+2, 2));
    EXPECT_EQ(s.sequence, le(p+4, 4)); EXPECT_EQ(s.resetCounter, le(p+8, 4));
    EXPECT_EQ(s.timeUs, le(p+12, 8)); EXPECT_EQ(123456u, le(p+20, 4));
    EXPECT_EQ(UINT32_MAX, le(p+24, 4)); EXPECT_FLOAT_EQ(.25f, leFloat(p+28));
    EXPECT_FLOAT_EQ(.5f, leFloat(p+32)); EXPECT_EQ(17u, le(p+36, 4)); EXPECT_EQ(3, p[40]);
    for (int i=41; i<44; ++i) EXPECT_EQ(0, p[i]);
    for (int i=0; i<3; ++i) {
        EXPECT_FLOAT_EQ(s.positionVariance[i], leFloat(p+44+4*i));
        EXPECT_FLOAT_EQ(s.velocityVariance[i], leFloat(p+56+4*i));
    }
    for (int i=68; i<249; ++i) EXPECT_EQ(0, p[i]);
    s.velocityVariance[0] = INFINITY;
    s.positionVariance[2] = -1;
    mavlinkPackEstimatorHealth(&msg, &status, &s);
    mavlink_msg_v2_extension_decode(&msg, &h);
    EXPECT_TRUE(std::isnan(leFloat(h.payload+56)));
    EXPECT_TRUE(std::isnan(leFloat(h.payload+52)));
    s.present = false; mavlinkPackEstimatorHealth(&msg, &status, &s);
    mavlink_msg_v2_extension_decode(&msg, &h);
    EXPECT_EQ(0u, le(h.payload+2, 2)); EXPECT_EQ(UINT32_MAX, le(h.payload+20, 4));
    EXPECT_TRUE(std::isnan(leFloat(h.payload+44)));
}

TEST(MavlinkScheduler, DeadlinesWrapAndSkipWithoutCatchup)
{
    uint32_t deadline = UINT32_MAX-4;
    EXPECT_FALSE(mavlinkStreamDue(UINT32_MAX-5, &deadline, 10));
    EXPECT_TRUE(mavlinkStreamDue(UINT32_MAX-4, &deadline, 10)); EXPECT_EQ(5u, deadline);
    EXPECT_TRUE(mavlinkStreamDue(1005, &deadline, 10)); EXPECT_EQ(1015u, deadline);
    EXPECT_FALSE(mavlinkStreamDue(1005, &deadline, 10));
    EXPECT_FALSE(mavlinkStreamDue(1005, &deadline, UINT32_MAX));
}

class MavlinkTransport : public ::testing::Test {
protected:
    serialPort_t port = {};
    void SetUp() override {
        nowUs = 0; freeBytes = 256; writes.clear(); state = sample();
        state.sequence = 0; state.present = false;
        mavlinkTestInit(&port);
    }
    void tick(unsigned ms, bool update = true) {
        nowUs = ms*1000; freeBytes = 256;
        if (update && ms%10 == 0) { state.present = true; ++state.sequence; state.timeUs = mavlinkTimeUs(nowUs); }
        mavlinkTestProcess();
    }
};
TEST_F(MavlinkTransport, Actual100And50HzAndOnlyTwoStreams)
{
    for (unsigned ms=0; ms<=1000; ++ms) tick(ms);
    const auto stats = mavlinkGetTelemetryStats();
    EXPECT_EQ(100u, stats->odometryFrames); EXPECT_EQ(49u, stats->healthFrames); // First health deadline 25ms.
    EXPECT_EQ(0u, stats->droppedFrames);
    unsigned odo=0, health=0;
    uint64_t previous=0; int previousSeq=-1;
    for (const auto &wire : writes) {
        auto msg=decode(wire);
        if (previousSeq >= 0) { EXPECT_EQ(uint8_t(previousSeq+1), msg.seq); }
        previousSeq=msg.seq;
        if (msg.msgid == 331) {
            mavlink_odometry_t o; mavlink_msg_odometry_decode(&msg, &o);
            EXPECT_GT(o.time_usec, previous); previous=o.time_usec; ++odo;
        } else { EXPECT_EQ(248u, msg.msgid); ++health; }
    }
    EXPECT_EQ(100u, odo); EXPECT_EQ(49u, health);
    for (auto id : {0,1,24,30,33,35,65,74,105,137,147}) EXPECT_FALSE(mavlinkTestSetInterval(id,10));
    EXPECT_TRUE(mavlinkTestSetInterval(331, UINT32_MAX));
    const auto count=stats->odometryFrames;
    for (unsigned ms=1001; ms<=1100; ++ms) tick(ms);
    EXPECT_EQ(count, stats->odometryFrames);
    EXPECT_TRUE(mavlinkTestSetInterval(331,0));
    for (unsigned ms=1101; ms<=1200; ++ms) tick(ms);
    EXPECT_EQ(count+10, stats->odometryFrames);
}
TEST_F(MavlinkTransport, CongestionDropsWholeFrameAndDoesNotReplay)
{
    state.present=true; state.sequence=1000; nowUs=10000; state.timeUs=mavlinkTimeUs(nowUs);
     freeBytes=10; mavlinkTestProcess();
    EXPECT_TRUE(writes.empty()); EXPECT_EQ(1u, mavlinkGetTelemetryStats()->droppedFrames);
    nowUs=20000; freeBytes=256; mavlinkTestProcess();
    EXPECT_TRUE(writes.empty()); // Same sample is not retried.
    nowUs=30000; ++state.sequence; state.timeUs=mavlinkTimeUs(nowUs); mavlinkTestProcess();
    ASSERT_EQ(1u,writes.size()); EXPECT_EQ(331u,decode(writes[0]).msgid);
}
TEST_F(MavlinkTransport, StaleStateNeverGetsNewTimestamp)
{
    state.present=true; state.sequence=2000; nowUs=50000; state.timeUs=mavlinkTimeUs(nowUs)-50000;
     freeBytes=256; mavlinkTestProcess();
    EXPECT_EQ(1u,mavlinkGetTelemetryStats()->staleStates);
    EXPECT_EQ(0u,mavlinkGetTelemetryStats()->odometryFrames);
    EXPECT_EQ(uint32_t(0),uint32_t(state.timeUs));
}
TEST_F(MavlinkTransport, TimesyncUsesSameBootEpochAndEchoesRequest)
{
    mavlink_message_t req; mavlink_msg_timesync_pack(42,11,&req,0,123456789,1,1);
    nowUs=123456; freeBytes=256;
    const auto expected=mavlinkTimeUs(nowUs);
    mavlinkTestTimesync(&req);
    ASSERT_EQ(1u,writes.size()); auto msg=decode(writes[0]);
    mavlink_timesync_t ts; mavlink_msg_timesync_decode(&msg,&ts);
    EXPECT_EQ(expected*1000,uint64_t(ts.tc1)); EXPECT_EQ(123456789,ts.ts1);
    EXPECT_EQ(42,ts.target_system); EXPECT_EQ(11,ts.target_component);
    mavlinkTestTimesync(&msg); EXPECT_EQ(1u,writes.size());
}

TEST_F(MavlinkTransport, ProducerJustAfterDeadlineDoesNotLoseTenMillisecondSlot)
{
    // Alternate phase around deadlines: a sample appears at 11ms, then 20ms,
    // then 31ms. Holding the due slot must preserve each distinct sample.
    state.sequence=3000;
    for (unsigned ms=1; ms<=1001; ++ms) {
        nowUs=ms*1000; freeBytes=256;
        if (ms%20==11 || ms%20==0) {
            state.present=true; ++state.sequence; state.timeUs=mavlinkTimeUs(nowUs);
        }
        mavlinkTestProcess();
    }
    EXPECT_EQ(100u,mavlinkGetTelemetryStats()->odometryFrames);
}

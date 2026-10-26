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

/*
 * telemetry_mavlink.c
 *
 * Author: Konstantin Sharlaimov
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"
#include "common/float_bits.h"

#if defined(USE_TELEMETRY_MAVLINK)

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/flight_plan_nav.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"

#include "rx/rx.h"
#include "rx/mavlink.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "telemetry/telemetry.h"
#include "telemetry/mavlink.h"
#include "telemetry/mavlink_time.h"
#include "telemetry/mavlink_odometry.h"
#if ENABLE_TELEMETRY_MAVLINK_MISSION
#include "telemetry/mavlink_mission.h"
#endif

#include "build/debug.h"
#include "build/version.h"

// mavlink library uses unnamed unions that causes GCC to complain if -Wpedantic is used
// until this is resolved in mavlink library - ignore -Wpedantic for mavlink code
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#define TELEMETRY_MAVLINK_INITIAL_PORT_MODE MODE_RXTX

#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID MAV_COMP_ID_AUTOPILOT1
extern uint16_t rssi; // FIXME dependency on mw.c

static serialPort_t *mavlinkPort = NULL;
static const serialPortConfig_t *portConfig;

static bool mavlinkTelemetryEnabled =  false;
static portSharing_e mavlinkPortSharing;

static mavlink_message_t mavRxMsg;
static mavlink_status_t mavRxStatus;
static bool mavlinkPortOwned = false;

// Betaflight-specific MAVLink custom_mode values. Stable enumeration emitted in
// HEARTBEAT.custom_mode and advertised via AVAILABLE_MODES; consumed in T3 by
// MAV_CMD_DO_SET_MODE handling.
typedef enum {
    BF_MAV_MODE_ACRO = 0,
    BF_MAV_MODE_ANGLE,
    BF_MAV_MODE_HORIZON,
    BF_MAV_MODE_ALT_HOLD,
    BF_MAV_MODE_POS_HOLD,
    BF_MAV_MODE_AUTOPILOT,
    BF_MAV_MODE_RTL,
    BF_MAV_MODE_FAILSAFE,
    BF_MAV_MODE_COUNT,
} bfMavMode_e;

typedef struct {
    const char *name;
    uint8_t standardMode;
    uint32_t properties;
} bfMavModeDescriptor_t;

static const bfMavModeDescriptor_t bfMavModeDescriptors[BF_MAV_MODE_COUNT] = {
    [BF_MAV_MODE_ACRO]      = { "Acro",      MAV_STANDARD_MODE_NON_STANDARD,  0 },
    [BF_MAV_MODE_ANGLE]     = { "Angle",     MAV_STANDARD_MODE_NON_STANDARD,  0 },
    [BF_MAV_MODE_HORIZON]   = { "Horizon",   MAV_STANDARD_MODE_NON_STANDARD,  0 },
    [BF_MAV_MODE_ALT_HOLD]  = { "Alt Hold",  MAV_STANDARD_MODE_ALTITUDE_HOLD, 0 },
    [BF_MAV_MODE_POS_HOLD]  = { "Pos Hold",  MAV_STANDARD_MODE_POSITION_HOLD, 0 },
    [BF_MAV_MODE_AUTOPILOT] = { "Autopilot", MAV_STANDARD_MODE_MISSION,       0 },
    [BF_MAV_MODE_RTL]       = { "RTL",       MAV_STANDARD_MODE_SAFE_RECOVERY, 0 },
    [BF_MAV_MODE_FAILSAFE]  = { "Failsafe",  MAV_STANDARD_MODE_SAFE_RECOVERY, MAV_MODE_PROPERTY_NOT_USER_SELECTABLE },
};

static mavlink_message_t mavMsg;
static uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN];

static mavlinkTelemetryStats_t txStats;

const mavlinkTelemetryStats_t *mavlinkGetTelemetryStats(void)
{
    return &txStats;
}

static uint64_t mavlinkBootTimeUs(void)
{
    return mavlinkTimeUs((uint32_t)micros());
}

static bool mavlinkSerialWrite(uint8_t *buf, uint16_t length)
{
    // Single cooperative writer per serial port. Reserve the complete frame;
    // never block on DMA/IRQ drain and never put a partial packet on the wire.
    if (!mavlinkPort || serialTxBytesFree(mavlinkPort) < length) {
        ++txStats.droppedFrames;
        return false;
    }
    serialWriteBuf(mavlinkPort, buf, length);
    ++txStats.sentFrames;
    return true;
}

// Pack-to-buffer + write helper shared with telemetry/mavlink_mission so the
// mission module reuses this file's static mavBuffer/mavlinkPort path.
void mavlinkSendMessage(mavlink_message_t *msg)
{
    if (!mavlinkPort) {
        return;
    }
    // Mission messages are packed in another translation unit. Re-finalize with
    // this link's TX sequence so command replies and both streams share it.
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msg->msgid);
    if (!entry) {
        return;
    }
    mavlink_set_proto_version(MAVLINK_COMM_0, 2);
    mavlink_finalize_message_buffer(msg, msg->sysid, msg->compid,
        mavlink_get_channel_status(MAVLINK_COMM_0), entry->min_msg_len,
        entry->max_msg_len, entry->crc_extra);
    const uint16_t length = mavlink_msg_to_send_buffer(mavBuffer, msg);
    mavlinkSerialWrite(mavBuffer, length);
}

static void mavlinkSendSystemTime(void)
{
    uint64_t timeUnixUsec = 0;
#ifdef USE_RTC_TIME
    rtcTime_t rtcMs;
    if (rtcHasTime() && rtcGet(&rtcMs) && rtcMs > 0) {
        timeUnixUsec = (uint64_t)rtcMs * 1000ULL;
    }
#endif

    mavlink_msg_system_time_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        timeUnixUsec, millis());
    const uint16_t msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

static void handleHeartbeatRx(const mavlink_message_t *msg)
{
    UNUSED(msg);
    // Stub for now; later tiers will track GCS liveness and target replies by sysid/compid.
}

static void handlePing(const mavlink_message_t *msg)
{
    mavlink_ping_t ping;
    mavlink_msg_ping_decode(msg, &ping);

    // PING request per spec has target_system == 0 && target_component == 0;
    // non-zero target fields indicate a response, which we must not echo back.
    if (ping.target_system != 0 || ping.target_component != 0) {
        return;
    }

    mavlink_msg_ping_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        ping.time_usec,
        ping.seq,
        msg->sysid,
        msg->compid);
    const uint16_t msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

static void handleTimesync(const mavlink_message_t *msg)
{
    mavlink_timesync_t timesync;
    mavlink_msg_timesync_decode(msg, &timesync);

    // tc1 == 0 marks a request; non-zero is a response we never solicited.
    if (timesync.tc1 != 0) {
        return;
    }

    // Accept broadcast or directed-to-us; ignore requests addressed to other systems.
    if ((timesync.target_system != 0 && timesync.target_system != MAVLINK_SYSTEM_ID) ||
        (timesync.target_component != 0 && timesync.target_component != MAVLINK_COMPONENT_ID)) {
        return;
    }

    mavlink_msg_timesync_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        (int64_t)mavlinkBootTimeUs() * 1000LL,
        timesync.ts1,
        msg->sysid,
        msg->compid);
    const uint16_t msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

static void mavlinkSendCommandAck(uint16_t command, uint8_t result,
    uint8_t targetSystem, uint8_t targetComponent)
{
    mavlink_msg_command_ack_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        command, result, 0, 0, targetSystem, targetComponent);
    const uint16_t msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

static void mavlinkSendAutopilotVersion(void)
{
    const uint64_t capabilities = MAV_PROTOCOL_CAPABILITY_MAVLINK2;
    const uint32_t flightSwVersion =
        ((uint32_t)(FC_VERSION_YEAR - FC_CALVER_BASE_YEAR) << 24) |
        ((uint32_t)FC_VERSION_MONTH << 16) |
        ((uint32_t)FC_VERSION_PATCH_LEVEL << 8);

    const uint8_t emptyCustom[8] = {0};
    uint8_t uid2[18] = {0};
    const uint32_t uidWords[3] = { U_ID_0, U_ID_1, U_ID_2 };
    memcpy(uid2, uidWords, sizeof(uidWords));

    mavlink_msg_autopilot_version_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        capabilities,
        flightSwVersion,
        0,
        0,
        0,
        emptyCustom,
        emptyCustom,
        emptyCustom,
        0,
        0,
        ((uint64_t)U_ID_0 << 32) | U_ID_1,
        uid2);
    const uint16_t msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

static void mavlinkSendProtocolVersion(void)
{
    const uint8_t emptyHash[8] = {0};
    mavlink_msg_protocol_version_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        200, 100, 200, emptyHash, emptyHash);
    const uint16_t msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

static void mavlinkSendComponentInformation(void)
{
    mavlink_msg_component_information_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        millis(), 0, "", 0, "");
    const uint16_t msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

static uint32_t mavlinkComputeCustomMode(void)
{
    if (FLIGHT_MODE(FAILSAFE_MODE) || failsafeIsActive()) {
        return BF_MAV_MODE_FAILSAFE;
    }
    if (FLIGHT_MODE(GPS_RESCUE_MODE) || flightPlanNavIsRescuePlanActive()) {
        return BF_MAV_MODE_RTL;
    }
    if (FLIGHT_MODE(AUTOPILOT_MODE)) {
        return BF_MAV_MODE_AUTOPILOT;
    }
    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        return BF_MAV_MODE_POS_HOLD;
    }
    if (FLIGHT_MODE(ALT_HOLD_MODE)) {
        return BF_MAV_MODE_ALT_HOLD;
    }
    if (FLIGHT_MODE(HORIZON_MODE)) {
        return BF_MAV_MODE_HORIZON;
    }
    if (FLIGHT_MODE(ANGLE_MODE)) {
        return BF_MAV_MODE_ANGLE;
    }
    return BF_MAV_MODE_ACRO;
}

static void mavlinkSendAvailableMode(uint8_t modeIndex)
{
    if (modeIndex < 1 || modeIndex > BF_MAV_MODE_COUNT) {
        return;
    }
    const uint8_t idx0 = modeIndex - 1;
    const bfMavModeDescriptor_t *desc = &bfMavModeDescriptors[idx0];

    mavlink_msg_available_modes_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        BF_MAV_MODE_COUNT,
        modeIndex,
        desc->standardMode,
        idx0,
        desc->properties,
        desc->name);
    const uint16_t msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

static void mavlinkSendAvailableModesMonitor(void)
{
    mavlink_msg_available_modes_monitor_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg, 1);
    const uint16_t msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

// COMMAND_LONG params arrive as floats from an external GCS. Casting NaN/inf or
// out-of-range values to integer types is undefined behaviour, so validate first.
static bool cmdParamToUint32(float f, uint32_t maxValue, uint32_t *out)
{
    if (!float32IsFinite(f) || f < 0.0f) {
        return false;
    }
    if ((double)f > (double)maxValue) {
        return false;
    }
    *out = (uint32_t)f;
    return true;
}

static int32_t getMessageUpdateInterval(uint32_t messageId);
static void mavlinkSendMessageInterval(uint32_t messageId)
{
    uint16_t msgLength;
    int32_t updateIntervalUs;

    updateIntervalUs = getMessageUpdateInterval(messageId);
    mavlink_msg_message_interval_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg, messageId, updateIntervalUs);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

static void mavlinkSendHeartbeat(void);

static void handleRequestMessage(const mavlink_command_long_t *cmd,
    uint8_t targetSystem, uint8_t targetComponent)
{
    uint32_t messageId;
    if (!cmdParamToUint32(cmd->param1, UINT32_MAX, &messageId)) {
        mavlinkSendCommandAck(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_DENIED, targetSystem, targetComponent);
        return;
    }

    uint8_t result;
    switch (messageId) {
#if ENABLE_TELEMETRY_MAVLINK_MISSION
    case MAVLINK_MSG_ID_MISSION_CURRENT:
        mavMissionSendCurrent();
        result = MAV_RESULT_ACCEPTED;
        break;
#endif
    case MAVLINK_MSG_ID_SYSTEM_TIME:
        mavlinkSendSystemTime();
        result = MAV_RESULT_ACCEPTED;
        break;
    case MAVLINK_MSG_ID_HEARTBEAT:
        mavlinkSendHeartbeat();
        result = MAV_RESULT_ACCEPTED;
        break;
    case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
        mavlinkSendAutopilotVersion();
        result = MAV_RESULT_ACCEPTED;
        break;
    case MAVLINK_MSG_ID_PROTOCOL_VERSION:
        mavlinkSendProtocolVersion();
        result = MAV_RESULT_ACCEPTED;
        break;
    case MAVLINK_MSG_ID_COMPONENT_INFORMATION:
        mavlinkSendComponentInformation();
        result = MAV_RESULT_ACCEPTED;
        break;
    case MAVLINK_MSG_ID_AVAILABLE_MODES: {
        uint32_t modeIndex;
        if (!cmdParamToUint32(cmd->param2, BF_MAV_MODE_COUNT, &modeIndex) || modeIndex < 1) {
            result = MAV_RESULT_DENIED;
            break;
        }
        mavlinkSendAvailableMode((uint8_t)modeIndex);
        result = MAV_RESULT_ACCEPTED;
        break;
    }
    case MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR:
        mavlinkSendAvailableModesMonitor();
        result = MAV_RESULT_ACCEPTED;
        break;
    case MAVLINK_MSG_ID_MESSAGE_INTERVAL: {
        uint32_t msgId;
        if (!cmdParamToUint32(cmd->param2, UINT32_MAX, &msgId)) {
            result = MAV_RESULT_DENIED;
            break;
        }
        mavlinkSendMessageInterval(msgId);
        result = MAV_RESULT_ACCEPTED;
        break;
    }
    default:
        result = MAV_RESULT_UNSUPPORTED;
        break;
    }
    mavlinkSendCommandAck(MAV_CMD_REQUEST_MESSAGE, result, targetSystem, targetComponent);
}

static bool setMessageUpdateInterval(uint32_t id, uint32_t intervalMs);
static void handleSetMessageInterval(const mavlink_command_long_t *cmd, uint8_t targetSystem, uint8_t targetComponent)
{
    uint32_t msgId;
    bool success = false;

    if (!cmdParamToUint32(cmd->param1, UINT32_MAX, &msgId)) {
        mavlinkSendCommandAck(cmd->command, MAV_RESULT_DENIED, targetSystem, targetComponent);
        return;
    }

    if (!float32IsFinite(cmd->param2) || cmd->param2 > (float)INT32_MAX) {
        mavlinkSendCommandAck(cmd->command, MAV_RESULT_DENIED, targetSystem, targetComponent);
        return;
    }
    if (cmd->param2 < 0.0f) {
        // Disable message per MAVLink spec (param2 < 0 means disable)
        success = setMessageUpdateInterval(msgId, UINT32_MAX);
    } else if (cmd->param2 == 0.0f) {
        // param2 == 0 means "request default rate" per MAVLink spec
        success = setMessageUpdateInterval(msgId, 0);
    } else {
        uint32_t intervalMs = (cmd->param2 >= 1000.0f) ? (uint32_t)(cmd->param2 / 1000.0f) : 1;
        success = setMessageUpdateInterval(msgId, intervalMs);
    }
    mavlinkSendCommandAck(cmd->command, success ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED, targetSystem, targetComponent);
}

static void handleCommandLong(const mavlink_message_t *msg)
{
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(msg, &cmd);

    // Accept broadcast or directed-to-us; ignore commands addressed to other systems.
    if ((cmd.target_system != 0 && cmd.target_system != MAVLINK_SYSTEM_ID) ||
        (cmd.target_component != 0 && cmd.target_component != MAVLINK_COMPONENT_ID)) {
        return;
    }

    switch (cmd.command) {
    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
        // Spec: param1 is MAV_BOOL — 1 = send AUTOPILOT_VERSION, 0 = ignore.
        uint32_t request;
        if (!cmdParamToUint32(cmd.param1, 1, &request)) {
            mavlinkSendCommandAck(cmd.command, MAV_RESULT_DENIED, msg->sysid, msg->compid);
            break;
        }
        if (request == 1) {
            mavlinkSendAutopilotVersion();
        }
        mavlinkSendCommandAck(cmd.command, MAV_RESULT_ACCEPTED, msg->sysid, msg->compid);
        break;
    }
    case MAV_CMD_REQUEST_MESSAGE:
        handleRequestMessage(&cmd, msg->sysid, msg->compid);
        break;
    case MAV_CMD_SET_MESSAGE_INTERVAL:
        handleSetMessageInterval(&cmd, msg->sysid, msg->compid);
        break;
    default:
        mavlinkSendCommandAck(cmd.command, MAV_RESULT_UNSUPPORTED, msg->sysid, msg->compid);
        break;
    }
}

static void mavlinkDispatch(const mavlink_message_t *msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        handleHeartbeatRx(msg);
        break;
    case MAVLINK_MSG_ID_PING:
        handlePing(msg);
        break;
    case MAVLINK_MSG_ID_TIMESYNC:
        handleTimesync(msg);
        break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
        handleCommandLong(msg);
        break;
    default:
#if ENABLE_TELEMETRY_MAVLINK_MISSION
        mavMissionHandleMessage(msg);
#endif
        break;
    }
}

static void mavlinkProcessIncoming(void)
{
    if (!mavlinkPortOwned || !mavlinkPort) {
        return;
    }
    // Bound the drain so a flooded link cannot starve the telemetry task.
    uint16_t rxBudget = 64;
    uint32_t rxBytesWaiting = serialRxBytesWaiting(mavlinkPort);
    while (rxBudget-- && rxBytesWaiting-- > 0) {
        const uint8_t c = serialRead(mavlinkPort);
        if (mavlink_parse_char(MAVLINK_COMM_1, c, &mavRxMsg, &mavRxStatus) == MAVLINK_FRAMING_OK) {
            mavlinkDispatch(&mavRxMsg);
        }
    }
}

#ifdef USE_SERIALRX_MAVLINK
static void mavlinkProcessQueueMessages(void)
{
    uint8_t rxBudget = MAVLINK_RX_QUEUE_SIZE;
    while (rxBudget-- && mavlinkGetNextQueueMessage(&mavRxMsg)) {
        mavlinkDispatch(&mavRxMsg);
    }
}
#endif

void freeMAVLinkTelemetryPort(void)
{
    closeSerialPort(mavlinkPort);
    mavlinkPort = NULL;
    mavlinkPortOwned = false;
    mavlinkTelemetryEnabled = false;
}

void initMAVLinkTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_MAVLINK);
    mavlinkPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_MAVLINK);
}

void configureMAVLinkTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        // Dedicated fused-state link default.
        baudRateIndex = BAUD_921600;
    }

    mavlinkPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MAVLINK, NULL, NULL, baudRates[baudRateIndex], TELEMETRY_MAVLINK_INITIAL_PORT_MODE, telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);

    if (!mavlinkPort) {
        return;
    }

    mavlinkPortOwned = true;
    mavlink_reset_channel_status(MAVLINK_COMM_1);
    mavlinkTelemetryEnabled = true;
#if ENABLE_TELEMETRY_MAVLINK_MISSION
    mavMissionInit();
#endif
}

static void mavlinkSendHeartbeat(void)
{
    uint16_t msgLength;

    uint8_t mavModes = MAV_MODE_MANUAL_DISARMED;
    if (ARMING_FLAG(ARMED))
        mavModes |= MAV_MODE_MANUAL_ARMED;

    uint8_t mavSystemType;
    switch (mixerConfig()->mixerMode)
    {
        case MIXER_TRI:
            mavSystemType = MAV_TYPE_TRICOPTER;
            break;
        case MIXER_QUADP:
        case MIXER_QUADX:
        case MIXER_Y4:
        case MIXER_VTAIL4:
            mavSystemType = MAV_TYPE_QUADROTOR;
            break;
        case MIXER_Y6:
        case MIXER_HEX6:
        case MIXER_HEX6X:
            mavSystemType = MAV_TYPE_HEXAROTOR;
            break;
        case MIXER_OCTOX8:
        case MIXER_OCTOX8P:
        case MIXER_OCTOFLATP:
        case MIXER_OCTOFLATX:
            mavSystemType = MAV_TYPE_OCTOROTOR;
            break;
        case MIXER_FLYING_WING:
        case MIXER_AIRPLANE:
        case MIXER_CUSTOM_AIRPLANE:
            mavSystemType = MAV_TYPE_FIXED_WING;
            break;
        case MIXER_HELI_120_CCPM:
        case MIXER_HELI_90_DEG:
            mavSystemType = MAV_TYPE_HELICOPTER;
            break;
        default:
            mavSystemType = MAV_TYPE_GENERIC;
            break;
    }

    const uint32_t mavCustomMode = mavlinkComputeCustomMode();
    if (mavCustomMode != BF_MAV_MODE_ACRO) {
        mavModes |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    uint8_t mavSystemState = 0;
    if (ARMING_FLAG(ARMED)) {
        if (failsafeIsActive()) {
            mavSystemState = MAV_STATE_CRITICAL;
        }
        else {
            mavSystemState = MAV_STATE_ACTIVE;
        }
    }
    else {
        mavSystemState = MAV_STATE_STANDBY;
    }

    mavlink_msg_heartbeat_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &mavMsg,
        // type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        mavSystemType,
        // autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        MAV_AUTOPILOT_GENERIC,
        // base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
        mavModes,
        // custom_mode A bitfield for use for autopilot-specific flags.
        mavCustomMode,
        // system_status System status flag, see MAV_STATE ENUM
        mavSystemState);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    // Packets transmit counter to debug actual data rate
    static uint32_t transmitCounter = 0;
    DEBUG_SET(DEBUG_MAVLINK_TELEMETRY, 6, transmitCounter);
    transmitCounter = (transmitCounter + 1) % 100;
}

static bool mavlinkSendOdometry(void)
{
    const fcStateSnapshot_t *state = positionEstimatorGetSnapshot();
    static uint32_t lastSequence;
    const uint64_t now = mavlinkBootTimeUs();
    if (!state->present || state->sequence == lastSequence) {
        ++txStats.noNewState;
        return false;
    }
    lastSequence = state->sequence; // Even a congested/stale sample is never queued.
    if (now < state->timeUs || now - state->timeUs > BF_STATE_MAX_AGE_US) {
        ++txStats.staleStates;
        return true;
    }
    mavlinkPackOdometry(&mavMsg, mavlink_get_channel_status(MAVLINK_COMM_0), state);
    const uint16_t length = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    if (mavlinkSerialWrite(mavBuffer, length)) {
        ++txStats.odometryFrames;
        const uint32_t latency = mavlinkBootTimeUs() - state->timeUs;
        txStats.lastStateLatencyUs = latency;
        txStats.maxStateLatencyUs = MAX(txStats.maxStateLatencyUs, latency);
    }
    return true;
}

static bool mavlinkSendEstimatorHealth(void)
{
    mavlinkPackEstimatorHealth(&mavMsg, mavlink_get_channel_status(MAVLINK_COMM_0), positionEstimatorGetSnapshot());
    const uint16_t length = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    if (mavlinkSerialWrite(mavBuffer, length)) {
        ++txStats.healthFrames;
    }
    return true;
}

// The only periodic streams. Removed sensor messages cannot be re-enabled by
// SET_MESSAGE_INTERVAL. Capability/mission/command replies stay on demand.
static mavlinkTelemetryOutputMessage_t mavTelemetryOutputMessages[] = {
    { .id = MAVLINK_MSG_ID_ODOMETRY, .stream = 0, .updateInterval = BF_ODOMETRY_INTERVAL_MS,
      .updateTime = 0, .sendMessageFunc = mavlinkSendOdometry },
    { .id = MAVLINK_MSG_ID_V2_EXTENSION, .stream = 0, .updateInterval = BF_HEALTH_INTERVAL_MS,
      .updateTime = 0, .sendMessageFunc = mavlinkSendEstimatorHealth },
};
#define TELEMETRIES_OUTPUT_MESSAGES_COUNT ARRAYLEN(mavTelemetryOutputMessages)

static uint32_t defaultInterval(uint32_t id)
{
    return id == MAVLINK_MSG_ID_ODOMETRY ? BF_ODOMETRY_INTERVAL_MS : BF_HEALTH_INTERVAL_MS;
}

static void configureMAVLinkOutputMessagesIntervals(void)
{
    const timeMs_t now = millis();
    mavlink_set_proto_version(MAVLINK_COMM_0, 2);
    for (unsigned i = 0; i < TELEMETRIES_OUTPUT_MESSAGES_COUNT; ++i) {
        mavTelemetryOutputMessages[i].updateInterval = defaultInterval(mavTelemetryOutputMessages[i].id);
        // Stagger health by 5ms; a 245-byte odometry frame fits a 256-byte UART.
        mavTelemetryOutputMessages[i].updateTime = now + mavTelemetryOutputMessages[i].updateInterval + 5*i;
    }
}

// A return value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
static int32_t getMessageUpdateInterval(uint32_t messageId)
{
    int32_t updateIntervalUs = 0;
    for (uint16_t i = 0; i < TELEMETRIES_OUTPUT_MESSAGES_COUNT; i++) {
        if (mavTelemetryOutputMessages[i].id == messageId) {
            uint32_t updateIntervalMs = mavTelemetryOutputMessages[i].updateInterval;
            if (updateIntervalMs == UINT32_MAX) {
                updateIntervalUs = -1;
            } else {
                updateIntervalUs = updateIntervalMs * 1000 <= INT32_MAX ? (int32_t)(updateIntervalMs * 1000) : INT32_MAX;
            }
            break;
        }
    }
    return updateIntervalUs;
}

// Set intervalMs update interval for MAVLink message
// If intervalMs == 0, then set default value from config
// If intervalMs == UINT32_MAX, then switch off message send
static bool setMessageUpdateInterval(uint32_t messageId, uint32_t intervalMs)
{
    for (uint16_t i = 0; i < TELEMETRIES_OUTPUT_MESSAGES_COUNT; i++) {
        if (mavTelemetryOutputMessages[i].id == messageId) {
            if (intervalMs == 0) { // Set default value
                intervalMs = defaultInterval(messageId);
            }
            if (intervalMs < defaultInterval(messageId)) {
                intervalMs = defaultInterval(messageId);
            }
            mavTelemetryOutputMessages[i].updateInterval = intervalMs;
            mavTelemetryOutputMessages[i].updateTime = millis() + (intervalMs == UINT32_MAX ? 0 : intervalMs);
            return true;
        }
    }
    return false;
}

static void processMAVLinkTelemetry(void)
{
    const timeMs_t now = millis();
    for (unsigned i = 0; i < TELEMETRIES_OUTPUT_MESSAGES_COUNT; ++i) {
        mavlinkTelemetryOutputMessage_t *stream = &mavTelemetryOutputMessages[i];
        uint32_t nextDeadline = stream->updateTime;
        if (mavlinkStreamDue(now, &nextDeadline, stream->updateInterval)) {
            // If the 100Hz producer is just behind this task, poll on the next
            // 1ms tick instead of losing a full 10ms slot. Never resend a sample.
            if (stream->sendMessageFunc()) {
                stream->updateTime = nextDeadline;
            }
        }
    }
}

void checkMAVLinkTelemetryState(void)
{
    if (portConfig && telemetryCheckRxPortShared(portConfig, rxRuntimeState.serialrxProvider)) {
        if (!mavlinkTelemetryEnabled && telemetrySharedPort != NULL) {
            mavlinkPort = telemetrySharedPort;
            mavlinkPortOwned = false;
            mavlinkTelemetryEnabled = true;
            configureMAVLinkOutputMessagesIntervals();
        }
    } else {
        bool newTelemetryEnabledValue = telemetryDetermineEnabledState(mavlinkPortSharing);

        if (newTelemetryEnabledValue == mavlinkTelemetryEnabled) {
            return;
        }

        if (newTelemetryEnabledValue) {
            configureMAVLinkTelemetryPort();
            configureMAVLinkOutputMessagesIntervals();
        } else {
            freeMAVLinkTelemetryPort();
        }
    }
}

void handleMAVLinkTelemetry(void)
{
    (void)mavlinkBootTimeUs(); // Keep the epoch alive even while the link is disabled.
    mavlink_set_proto_version(MAVLINK_COMM_0, 2);
    if (!mavlinkTelemetryEnabled || !mavlinkPort) {
        return;
    }

#ifndef USE_SERIALRX_MAVLINK
    mavlinkProcessIncoming();
#else
    if (telemetrySharedPort != NULL && rxRuntimeState.serialrxProvider == SERIALRX_MAVLINK) {
        mavlinkProcessQueueMessages();
    } else {
        mavlinkProcessIncoming();
    }
#endif
#if ENABLE_TELEMETRY_MAVLINK_MISSION
    mavMissionUpdate(millis());
#endif

    processMAVLinkTelemetry();
}

#ifdef UNIT_TEST
void mavlinkTestInit(serialPort_t *port)
{
    mavlinkPort = port;
    txStats = (mavlinkTelemetryStats_t){0};
    configureMAVLinkOutputMessagesIntervals();
}
void mavlinkTestProcess(void) { processMAVLinkTelemetry(); }
bool mavlinkTestSetInterval(uint32_t id, uint32_t ms) { return setMessageUpdateInterval(id, ms); }
void mavlinkTestTimesync(const mavlink_message_t *msg) { handleTimesync(msg); }
#endif

#endif

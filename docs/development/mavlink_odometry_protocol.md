# Betaflight fused-state MAVLink protocol

Version: BF_ESTIMATOR_HEALTH_V1, 2026-10-26.
Base: `2026.6.1` / `6dbc4218fd6bc33bf16ea32c670304d4f89321d5`.
Local branch: `mavlink_odometry`.
Selective reference: `mavlink-HZ-prompt` / `c213bef857c8a5921c5c538c9c51995e66273411`.

## Link and periodic messages

Dedicated UART: **921600 baud, 8N1, MAVLink 2**, sysid 1, component AUTOPILOT1.
BAUD_AUTO selects 921600; existing explicit baud rates remain respected. Configure
the actual telemetry UART using CLI `serial` with its board-specific identifier.
The CLI permits 921600 for telemetry. This work does not connect/configure a board.

| ID | Message | Default | Minimum requested interval |
|---|---|---|---|
| 331 | ODOMETRY | 100 Hz | 10000 us |
| 248 | V2_EXTENSION, message_type 0xBF01 | 50 Hz | 20000 us |

`MAV_CMD_SET_MESSAGE_INTERVAL` supports only these two IDs. Negative disables; zero
restores default; positive selects an interval clamped to the minimum. Positive
intervals above the minimum are rounded down to whole milliseconds. Nonfinite and
excessive values are denied. Legacy stream settings remain in PG/CLI for config
compatibility but do not control this link. Old IDs cannot be re-enabled.

TIMESYNC/PING, COMMAND_ACK, capability/version/mode queries, and existing command/
Mission processing remain. HEARTBEAT, SYSTEM_TIME and MISSION_CURRENT are available
through MAV_CMD_REQUEST_MESSAGE. MISSION_SET_CURRENT still responds with the cursor;
Mission transfer retries and waypoint-reached events remain. MISSION_CURRENT has
no periodic broadcast. Generic GCS discovery requiring periodic HEARTBEAT is not
guaranteed on this dedicated link. MSP control is retained.

Removed periodic senders: SYS_STATUS, RC_CHANNELS_RAW, GPS_RAW_INT,
GLOBAL_POSITION_INT, GPS_GLOBAL_ORIGIN, HOME_POSITION, ATTITUDE, HEARTBEAT, VFR_HUD,
BATTERY_STATUS and MISSION_CURRENT. Reference raw HIGHRES_IMU/SCALED_PRESSURE and
GPS snapshot hooks/configuration were not ported. Sensor drivers are retained.

## ODOMETRY fields

| Field | Meaning |
|---|---|
| time_usec | 64-bit FC boot us, start of the completed position-estimator update |
| frame_id | MAV_FRAME_LOCAL_NED (1) |
| child_frame_id | MAV_FRAME_BODY_FRD (12) |
| x,y,z | North, East, Down position, m |
| q[4] | Normalized w,x,y,z, BODY_FRD → LOCAL_NED rotation |
| vx,vy,vz | Fused velocity rotated into BODY_FRD, m/s |
| rollspeed,pitchspeed,yawspeed | Filtered controller gyro, BODY_FRD rad/s |
| reset_counter | Low 8 bits of full reset count |
| estimator_type | MAV_ESTIMATOR_TYPE_AUTOPILOT |
| quality | 0 when snapshot, XY, Z, heading and finite state are available; -1 otherwise |
| pose_covariance / velocity_covariance | Entire matrix unknown: element 0 NaN, remaining elements also NaN |

There is no calibrated 1–100 quality score. Trust is not calibrated accuracy. The
estimator lacks attitude/rate covariance, so no complete 6x6 covariance can be
reported. Actual translational diagonals are provided in health. This uses the
[MAVLink ODOMETRY unknown covariance/quality semantics](https://mavlink.io/en/messages/common.html#ODOMETRY).

Position and velocity exclusively come from `positionEstimatorGetEstimate()` after
the original Kalman update, scaled by 0.01 from cm/cm/s. ENU→NED is `[N,E,-U]`.
No raw GPS values substitute for the posterior. `getQuaternion()` gives BF body→NWU;
conjugating its rotation with diag(1,-1,-1) gives wire `[w,x,-y,-z]`. This matches
reference ATTITUDE (roll,-pitch,heading). Velocity is `R_NED_FRD^T * [vN,vE,-vU]`,
not world-frame NED velocity. Filtered `gyro.gyroADCf` maps `[gx,-gy,-gz]` and converts
degrees/s→rad/s. The headfree/display quaternion is not used.

ROS recovers ENU position `[wire.y,wire.x,-wire.z]`, and BODY_FLU velocity/rates
`[wire.x,-wire.y,-wire.z]`. Rotate body velocity with the recovered attitude when
world ENU velocity is needed. Existing ROS quaternion convention is:

```
k = sqrt(0.5)
q_ENU_FLU = [k*(w+z), k*(x+y), k*(x-y), k*(w-z)]
```

Tests cover North/East/South/West, roll/pitch, fixed motion directions, rates and
normalization. The ROS source was read for convention verification only: the current
raw-sensor receiver still needs a new ODOMETRY/health decoding path before MPC use.

## Snapshot and clock boundaries

`positionEstimatorUpdate()` remains at TASK_ALTITUDE_RATE_HZ=100. It records the
update epoch before prediction/correction and publishes a snapshot at completion.
Position, velocity, covariance, trust and validity belong to that update. Attitude
and filtered gyro are the latest scheduler samples read at completion, **not
interpolated or hardware synchronized** with each other or GPS/barometer/IMU. V1
does not encode their individual sensor ages. Time denotes an estimator epoch,
not GPS/UTC/hardware-sample time or UART send time.

Producer/consumer run in the cooperative FC scheduler, not interrupts. The read-only
snapshot is valid until the next update/reset; concurrent readers require additional
synchronization. The MAVLink task never runs filters or resamples sensors. Each
ODOMETRY sequence is attempted once; absent, duplicate or >20ms-old state is skipped.
A congested frame is dropped, never queued. Health continues with the same sequence/
time if no state update occurs. Receivers must reject old state timestamps.

`mavlinkTimeUs()` extends unsigned 32-bit micros to a shared 64-bit boot epoch.
Estimator and TIMESYNC use the same clock. TIMESYNC tc1 is FC nanoseconds; ts1 echoes
the request. Cooperative sampling must occur at least once every 2^32 us (~71.6min),
as the estimator/active telemetry task normally provide. Power cycles reset the
clock. Tests cover repeated micros wraps and exact TIMESYNC epoch agreement.

ROS must track a boot session, reset timesync on reboot/reconnect/time regression,
and clear references when reset_counter changes. A reconnect starts a new receiver
session even if a reboot cannot be distinguished from cable interruption. Counters
are not unique boot identifiers.

## BF_ESTIMATOR_HEALTH_V1

Standard [V2_EXTENSION](https://mavlink.io/en/messages/common.html#V2_EXTENSION),
message_type **0xBF01**, target network/system/component zero. Payload array: 249
bytes, first 68 used, remainder zero. MAVLink2 trims trailing zeros on wire; generated
decoders restore zeros. Verify BOTH message_type and inner version/length. Integers
and IEEE-754 float32 are little endian, serialized without unaligned struct casts.

| Offset | Type | Field |
|---|---|---|
| 0 | uint8 | version=1 |
| 1 | uint8 | payload_length=68 |
| 2 | uint16 | flags |
| 4 | uint32 | state_sequence |
| 8 | uint32 | reset_counter |
| 12 | uint64 | state_time_usec |
| 20 | uint32 | xy_observation_age_us |
| 24 | uint32 | z_observation_age_us |
| 28 | float32 | trust_xy |
| 32 | float32 | trust_z |
| 36 | uint32 | source_flags |
| 40 | uint8 | gps_fix_type |
| 41 | uint8[3] | reserved=0 |
| 44 | float32[3] | position_variance_m2, East/North/Up |
| 56 | float32[3] | velocity_variance_m2ps2, East/North/Up |

Flags: bit0 xy_valid, bit1 z_valid, bit2 heading_valid, bit3 mag_heading_fresh,
bit4 armed, bit5 gps_fix, bit6 xy_fusion_enabled, bit7 state_snapshot_present.
Bits8–15 zero. Without a snapshot, all flags zero, ages UINT32_MAX, trust/variances
NaN. Nonfinite or negative variances also encode NaN.

Heading validity uses existing `imuIsHeadingValid()` (including GPS heading).
Mag freshness separately requires a finite recent magnetic yaw update, a successful
mag sample <=100ms old, trusted/calibrated compass, and no active mag calibration.
Base `compassEnabledAndCalibrated()` and attitude/control eligibility are unchanged.

source_flags are measurements incorporated **in this estimator update**: bit0 GPS
XY, bit1 GPS Z, bit2 barometer Z, bit3 rangefinder Z, bit4 optical-flow XY; other bits
zero. These do not declare sensor health/accuracy. gps_fix_type is 0 if no GPS sensor,
1 if no fix, 3 if base GPS_FIX is set. The base solution does not retain RTK/2D type;
no such precision classification is invented.

Ages track the original `lastXYMeasurementUs`/`lastZMeasurementUs` Kalman-update
events. Seen flags handle t=0; 64-bit bookkeeping avoids wrapping stale ages into
fresh ones. Never observed/saturated age is UINT32_MAX. Age is **at state_time_usec**;
receivers add elapsed time since that epoch. Existing fusion can reuse the latest
barometer/rangefinder/flow value every update: these are accepted-fusion-update ages,
**not independent physical-sensor freshness proof**. GPS uses gpsHasNewData().
Those algorithms are unchanged.

XY validity retains its original 2-second timeout and is false when fusion is
disabled (normally disarmed). ROS independently checks state/observation ages,
heading policy, flags, finite values, covariance and boot/reset session. Valid flags
and quality=0 alone do not establish high-rate control suitability. Variances are
actual P00/P11 times 0.0001 (cm²→m² and (cm/s)²→(m/s)²), not inferred from trust.

## Resets

The uint32 counter starts at zero in boot memory; estimator initialization increments
once. Explicit XY reset, explicit Z reset and disabled→enabled XY reinitialization
each increment once. Immediate GPS-origin capture inside an XY reset belongs to
that event, not a second count. A later first GPS-origin capture increments
separately. Arming can reset Z and reinitialize XY (two actual filter events).
Continuous barometer offset cross-calibration is not a discrete reset. Reset call
order/filter logic are unchanged. Reset invalidates the old snapshot immediately;
next completed update carries the new count. Counters use modular uint32 arithmetic.

## Capacity and diagnostics

MAVLink telemetry task: 1000Hz when configured. Estimator remains 100Hz. Deadlines
handle millis wrap, advance from previous deadlines to avoid accumulated jitter,
and skip missed slots without catch-up bursts. If a state is not yet available,
the due slot waits for the next 1ms poll rather than consuming a 10ms interval. Health is phased 5ms after ODOMETRY.

ODOMETRY <=233 payload+12 framing=245 bytes. Used health payload is <=73+12=85 bytes
on wire (266 bytes if all 249 extension payload bytes were used). Default bound:
`245*100 + 85*50 = 28750 bytes/s`, **287500 bit/s or 31.2% of 921600** at 8N1.
Full 266-byte health would yield 41.0%. These are theoretical capacity bounds.
Signing is not enabled; its 13 bytes would make ODOMETRY exceed a 256-byte TX buffer.

UART default TX buffer: 256 bytes, or 1280 with USE_MSP_DISPLAYPORT. Before writing,
check room for the full encoded frame. No waiting for drain and no partial frame. Submission counters cannot confirm
physical delivery or detect a driver suppressing output due to TX pin monitoring.
Command/Mission responses can also drop when congested; retain protocol retries,
no unbounded queue. A 245-byte frame takes ~2.66ms at 921600, beyond enqueue latency.

`mavlinkGetTelemetryStats()` exposes sentFrames, droppedFrames, odometryFrames,
healthFrames, noNewState, staleStates, lastStateLatencyUs, maxStateLatencyUs for a
debugger. noNewState counts due-slot polls without a fresh sample. Latency is state epoch→successful enqueue, not host arrival. Use CLI
`tasks`/`status` plus counters for future hardware CPU and UART checks. SITL TCP TX
is drained every 1ms solely from the dyad event thread, avoiding cross-thread access.

## MSP override

Ported msp_override_timeout_ms with PG layout compatibility, CLI read/write, channel
coverage and timeout regression tests. Default remains **300ms**, preserving the
reference branch behavior. For the requested flight watchdog explicitly set and
read back `set msp_override_timeout_ms = 50` / `get msp_override_timeout_ms` on the
intended deployed config. Zero disables override; >1000 is rejected by freshness
checking. Tests verify fresh at 50ms, stale at 51ms, including millis wrap. No board
configuration was changed here.

## Reproduction

Default host clang could not find C++ standard headers. GCC works; isolate outputs
to avoid mixing clang/GCC coverage objects. The ordinary SITL build omits telemetry;
the explicit flags below are necessary.

```sh
make -C src/test CC=gcc CXX=g++ OBJECT_DIR=/tmp/bf-odometry-release-tests \
  test_mavlink_odometry_unittest test_position_estimator_unittest \
  test_mavlink_time_unittest test_flight_imu_unittest \
  test_rx_msp_override_unittest test_pg_unittest \
  test_telemetry_mavlink_mission_unittest -j4
CCACHE_DIR=/tmp/bf-odometry-ccache CCACHE_TEMPDIR=/tmp LC_ALL=C \
  make CONFIG=GEPRC_TAKER_H743 -j4
CCACHE_DIR=/tmp/bf-odometry-ccache CCACHE_TEMPDIR=/tmp LC_ALL=C \
  make TARGET=SITL EXTRA_FLAGS='-DUSE_TELEMETRY -DUSE_TELEMETRY_MAVLINK' -j4
python3 src/test/sitl/mavlink_odometry_harness.py obj/main/betaflight_SITL.elf
git diff --check
```

SITL requires pymavlink and permission for localhost TCP/UDP. It supplies simulated
sensor inputs to the real estimator. Codec tests use synthetic snapshots. These
checks do not verify physical UART throughput, CPU margin, sensor latency or flight
performance. ROS subscriber integration and real hardware validation remain separate.

## Per-file review

| File | Change / interface / selective port |
|---|---|
| `docs/development/mavlink_odometry_protocol.md` | Protocol, timing/validity boundaries, capacity, per-file review, validation and publication instructions. |
| `mk/source.mk` | Register shared clock and ODOMETRY/health codec sources. |
| `src/main/cli/cli.c` | Port telemetry baud acceptance through 921600. |
| `src/main/cli/settings.c` | Expose msp_override_timeout_ms for CLI read/write. |
| `src/main/common/float_bits.h` | Add IEEE-754 finite check that survives -ffast-math/-Ofast; used only for telemetry validation/health. |
| `src/main/drivers/serial_tcp.c` | Port single-event-thread TCP TX drain for SITL. |
| `src/main/fc/tasks.c` | Port 1000Hz MAVLink telemetry scheduling; estimator frequency unchanged. |
| `src/main/flight/imu.c` | Port observational mag-update timestamp/finite flag and separate heading-health getter; attitude equations untouched. |
| `src/main/flight/imu.h` | Declare imuIsMagHeadingValid(). |
| `src/main/flight/position_estimator.c` | Capture completed posterior/attitude/gyro; record accepted observation times, source flags, covariances and reset events; KF calculations unchanged. |
| `src/main/flight/position_estimator.h` | Define fcStateSnapshot_t and read-only positionEstimatorGetSnapshot(). |
| `src/main/pg/rx.c` | Port timeout default 300ms without changing existing control defaults. |
| `src/main/pg/rx.h` | Append timeout configuration while preserving PG v4 byte layout. |
| `src/main/rx/msp.c` | Port configurable freshness check, including explicit 50ms setting and zero/invalid handling. |
| `src/main/sensors/compass.c` | Track successful mag read time and add separate compassIsHeadingFresh(); retain original control predicate. |
| `src/main/sensors/compass.h` | Declare read-only compassIsHeadingFresh(). |
| `src/main/telemetry/mavlink.c` | Replace old periodic senders with two streams; preserve command/Mission handling; on-demand discovery; shared v2 TX sequence, TIMESYNC epoch, whole-frame TX/drop/latency metrics, interval validation. |
| `src/main/telemetry/mavlink.h` | Declare TX diagnostics and pending-sample send callback contract. |
| `src/main/telemetry/mavlink_mission.c` | Remove periodic MISSION_CURRENT; retain on-demand cursor, transfer retries and reached events. |
| `src/main/telemetry/mavlink_mission.h` | Expose mavMissionSendCurrent() for query response. |
| `src/main/telemetry/mavlink_odometry.c` | Normalize/transform fused state, pack MAVLink2 ODOMETRY and fixed little-endian health; wrap-safe deadline helper. |
| `src/main/telemetry/mavlink_odometry.h` | Define protocol/rate constants and pack/scheduling interfaces. |
| `src/main/telemetry/mavlink_time.c` | Port unsigned micros extension and add shared estimator/TIMESYNC clock. |
| `src/main/telemetry/mavlink_time.h` | Document and declare clock contracts. |
| `src/platform/SIMULATOR/sitl.c` | Port 1ms dyad tick/update timeout. |
| `src/test/Makefile` | Register new test sources and per-test flags; compile actual codec/transport with -Ofast; isolate uncalled hardware functions. |
| `src/test/sitl/mavlink_odometry_harness.py` | Add simulated-input end-to-end wire, frequency, source-rate, load, response and stream-restriction checks. |
| `src/test/unit/flight_imu_unittest.cc` | Port magnetic heading health tests and mock freshness without changing controller tests. |
| `src/test/unit/mavlink_odometry_unittest.cc` | Add v2 byte/CRC decode, frames, rates, quaternion, health layout/unknowns, actual scheduler/TX/clock and congestion tests. |
| `src/test/unit/mavlink_time_unittest.cc` | Port repeated micros-wrap test. |
| `src/test/unit/pg_unittest.cc` | Port RX PG legacy-layout/default/extension preservation regression. |
| `src/test/unit/position_estimator_unittest.cc` | Add actual fused-posterior-to-ODOMETRY consistency, reset, GPS outage/recovery, unknown-age and disarmed tests. |
| `src/test/unit/rx_msp_override_unittest.cc` | Port 50/51ms cutoff, wrap, short-frame and zero/invalid timeout tests. |
| `src/test/unit/telemetry_mavlink_mission_unittest.cc` | Retain Mission behavior tests, test on-demand cursor and prohibit periodic cursor; C++ compatibility for existing C static assert. |


## Git state and publication

Before: clean `mavlink-HZ-prompt`, HEAD c213bef857c8a5921c5c538c9c51995e66273411;
no pre-existing uncommitted changes. The target branch did not exist. No applicable
AGENTS.md was found in the checkout or ancestor directories. Created local
mavlink_odometry directly at base 6dbc4218fd6bc33bf16ea32c670304d4f89321d5;
no cherry-pick/commit was used. Reference branch still points to its original HEAD.
After: implementation files modified/untracked, no staged changes. No commit, push,
firmware flash, or real-aircraft connection. All files remain for review in the
original clean checkout; no worktree was needed.

After reviewing these exact files, publication commands (not executed):

```sh
git switch mavlink_odometry
git diff --check
git add -- \
  docs/development/mavlink_odometry_protocol.md \
  mk/source.mk \
  src/main/cli/cli.c \
  src/main/cli/settings.c \
  src/main/common/float_bits.h \
  src/main/drivers/serial_tcp.c \
  src/main/fc/tasks.c \
  src/main/flight/imu.c \
  src/main/flight/imu.h \
  src/main/flight/position_estimator.c \
  src/main/flight/position_estimator.h \
  src/main/pg/rx.c \
  src/main/pg/rx.h \
  src/main/rx/msp.c \
  src/main/sensors/compass.c \
  src/main/sensors/compass.h \
  src/main/telemetry/mavlink.c \
  src/main/telemetry/mavlink.h \
  src/main/telemetry/mavlink_mission.c \
  src/main/telemetry/mavlink_mission.h \
  src/main/telemetry/mavlink_odometry.c \
  src/main/telemetry/mavlink_odometry.h \
  src/main/telemetry/mavlink_time.c \
  src/main/telemetry/mavlink_time.h \
  src/platform/SIMULATOR/sitl.c \
  src/test/Makefile \
  src/test/sitl/mavlink_odometry_harness.py \
  src/test/unit/flight_imu_unittest.cc \
  src/test/unit/mavlink_odometry_unittest.cc \
  src/test/unit/mavlink_time_unittest.cc \
  src/test/unit/pg_unittest.cc \
  src/test/unit/position_estimator_unittest.cc \
  src/test/unit/rx_msp_override_unittest.cc \
  src/test/unit/telemetry_mavlink_mission_unittest.cc
git commit -m "Add fused-state MAVLink2 odometry and estimator health"
git push -u origin mavlink_odometry
```

## Final validation record (2026-10-26)

All commands in Reproduction were executed (GCC outputs in
`/tmp/bf-odometry-release-tests`). Results:

| Check | Result |
|---|---|
| mavlink_odometry_unittest | 11 passed; actual codec/transport C compiled with -Ofast |
| position_estimator_unittest | 8 passed, including actual posterior→ODOMETRY fields |
| mavlink_time_unittest | 1 passed, repeated wrap and duplicate clock ticks |
| flight_imu_unittest | 18 passed |
| rx_msp_override_unittest | 2 passed, including 50/51ms timeout and wrap |
| pg_unittest | 3 passed, legacy RX layout preserved |
| telemetry_mavlink_mission_unittest | 10 passed, no unsolicited periodic cursor |
| Total unit tests | **53 passed** |
| H743 GEPRC_TAKER_H743 firmware | Build passed, no compiler errors/warnings in final log |
| MAVLink-enabled SITL | Build and end-to-end harness passed |
| git diff --check | Passed |

Final SITL measurement uses **FC boot state time**, not host wall-clock rate:

- ODOMETRY **98.492Hz**; producer sequence rate **98.491Hz**; health **50.046Hz**.
- 494 ODOMETRY and 251 health messages in the sample window; 250 health epochs
  matched ODOMETRY in that window (window edges/task phase need not pair every frame).
- ODOMETRY state-time gaps: 10000–12263us. No duplicate/reversed state timestamps,
  malformed/CRC-failed frames, or extra periodic message types observed.
- Simulator-reported MSP scheduler load: 0%. This is not a useful H743 CPU estimate.
- Disable/restore, all tested legacy stream denials, capabilities, on-demand
  HEARTBEAT/MISSION_CURRENT, NaN/Inf interval denial and TIMESYNC passed.
- Source generation and output rates match within measurement resolution. A
  deterministic scheduler test emits exactly 100 new ODOMETRY samples per second;
  **this SITL run does not demonstrate exact sustained 100Hz** because the existing
  estimator task runs slightly below its nominal rate. No estimator/control timing
  or filter equations were altered to manufacture the target rate.

Earlier failures and resolutions: default clang lacked C++ headers; switching to
GCC required isolated objects to avoid mixed coverage runtimes. An initial localhost
startup attempt failed before a later launch succeeded. Integration found the old
Mission periodic cursor (removed in favor of on-demand queries), then exposed
`-ffast-math` folding finite checks (fixed using IEEE-754 exponent bits and tested
under -Ofast). No acceptance threshold was loosened to mask those failures.

Final artifacts (not flashed):

| Artifact | SHA-256 |
|---|---|
| obj/betaflight_2026.6.1_STM32H743_GEPRC_TAKER_H743.hex | 57dd20f971c729fc447be2288ef3512479d5f2dfa1f66b92e34ef457e3418573 |
| obj/main/betaflight_SITL.elf | cb7e314466c8941c2c60e455f947d42af3485380ad2c7a5f86ec65d6f0f98126 |

H743 link report: FLASH1 579411/1835008 bytes (31.58%), ITCM 18576/65536 (28.34%),
DTCM 17528/131072 (13.37%), RAM 103700/524288 (19.78%). Logs are retained in ignored
`obj/mavlink_odometry_validation/`: unit-tests.log, sitl.log, build-h743.log,
build-sitl.log. The production symbol `txStats.lto_priv.0` exists in both ELF files;
LTO can inline/remove the unused public getter, so inspect the symbol with a debugger.

Remaining boundaries: physical UART frames/baud/congestion and board CPU margin
are **not verified**; no hardware was connected. Tests establish v2 wire bytes in
the real software TX path and SITL TCP stream, not physical UART delivery. Enqueue
latency counters exist, but hardware latency distribution is unmeasured. Snapshot
state epochs are preserved; attitude/gyro are latest scheduler samples rather than
strictly synchronized measurements. True physical-sensor freshness, actual fusion
accuracy, ROS subscriber integration and flight behavior still require their own
validation. Default MSP timeout remains 300ms; a deployed 50ms watchdog requires
explicit configuration/readback as documented above.

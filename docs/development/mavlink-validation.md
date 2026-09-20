# MAVLink sensor integration validation — 2026-09-16

## Delivered build

- Board: standard `GEPRC_TAKER_H743` (not V2 or MINI).
- Firmware: `obj/betaflight_2026.6.1_STM32H743_GEPRC_TAKER_H743.hex`
- SHA-256: `c4e7a3034b42193d45583d5609f92349395504639ba110ea010ae6e6f09793c8`
- Build: `CCACHE_DISABLE=1 LC_ALL=C make CONFIG=GEPRC_TAKER_H743 -j4`
- ROS 2 Humble: `agi_ros2/scripts/build.sh` passed, including generated Heading interface.
- No physical flight controller was opened or flashed.

## Passed checks

- MAVLink time unit test: one test, repeated 32-bit microsecond wraps and duplicate ticks.
- ROS pseudo-UART integration: eight tests passed. Tests run the actual compiled node.
  - Dedicated UART selection, symlink alias rejection, hardware-only launch.
  - Fragmented frames, CRC/source rejection, duplicate samples, units/axes/timestamps.
  - Delayed TIMESYNC rejection and synchronization establishment.
  - Extended boot time beyond the old microsecond boundary.
  - Heading conversion: north/east/south/west, 35999 boundary, 36000/65535 invalid values.
  - No-fix, unavailable velocity/accuracy/height and repeated GPS solutions.
  - GNSS versus RTK float/fixed diagnostics and valid ellipsoid altitude.
  - Flight-controller reboot and physical pseudo-UART disconnect/reconnect.
- Final Betaflight SITL telemetry run (rates measured in FC time):
  - Configured 250 Hz IMU: 250.4 Hz.
  - Runtime 500 Hz IMU: 500.1 Hz; restore 250 Hz: 250.6 Hz.
  - ATTITUDE: 100.0 Hz; fresh GPS solutions: 9.8 Hz.
  - Deliberately reduced 50 Hz sensor source: 48.8 Hz unique IMU output.
  - Packet decoding, units, sensor timestamps, stream disable/restore and deduplication passed.
- Both repositories pass `git diff --check`; pinned MAVLink header checksums match.

## Existing control regression caveat

The existing `agi_ros2/test/test_node_pipeline.py` was run without changing its
control/fusion/MSP implementations or relaxing deadlines. Full-suite results were
not stable: the first run passed 9/10; a later run passed 8/10. Failures were in
startup/warm-up or MSP override timing, rather than the new sensor node (which is
not launched by that suite).

The RC-gap warm-up and simulated-clock cases passed isolated reruns. The final
isolated `test_msp_pseudo_uart_aetr_only_and_kill` still failed at the assertion
that override is active; reported RC ROS-stamp ages were roughly 0.10–0.17 s,
exceeding the existing 0.1 s deadline while transport_healthy remained true.
No root cause is claimed from this observation, and no safety gates were weakened.
The original MSP regression therefore remains an unresolved validation failure.
Do not treat the successful sensor tests as proof that the whole flight pipeline
has passed regression or is ready for physical closed-loop operation.

## Not verified

Actual H743 CPU/UART load, GNSS module behavior, sensor/filter latency, PPS or
hardware sampling synchronization, physical ROS-clock steps, and flight operation.
GPS/IMU timestamps describe FC update times, not hardware measurement epochs.
Heading is an FC estimate from GLOBAL_POSITION_INT.hdg, not independent RTK heading.

#!/usr/bin/env python3
"""Check raw barometer MAVLink telemetry from SITL.

Requires pymavlink and free SITL ports 5761/5762/9003. Run after building SITL:
    python3 src/test/sitl/mavlink_baro_harness.py obj/main/betaflight_SITL.elf

The virtual barometer completes a read about every 12 ms, faster than the
MS5611's 25 ms cycle. Check the 40 Hz default separately from the new-sample
limit; neither SITL rate constitutes a measurement of the physical MS5611.
Rates are measured in FC time because simulator speed depends on the host.
"""

import math
import struct
import sys
import tempfile
import time

from pymavlink import mavutil

sys.dont_write_bytecode = True
from mavlink_telemetry_harness import HighRateFdmFeed, collect, rate
from sitl_harness import Sitl, FDM_PORT, HOME_LAT, HOME_LON, HOME_ALT_M


PRESSURE_ID = mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE
IMU_ID = mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU


def pressure_pa(altitude_m):
    # Match sitl.c: Gazebo derives pressure from height; the legacy bridge reads
    # the pressure field directly. Supply matching values for either build.
    return int(101325.0 * (1.0 - 2.25577e-5 * altitude_m) ** 5.25588)


class BaroFdmFeed(HighRateFdmFeed):
    altitude_m = HOME_ALT_M

    def run(self):
        while self.running:
            altitude = self.altitude_m
            packet = struct.pack('<18d', time.monotonic() - self.t0,
                0, 0, 0, 0, 0, -9.80665,
                math.sqrt(.5), 0, 0, math.sqrt(.5),
                0, 0, 0, HOME_LON, HOME_LAT, altitude, pressure_pa(altitude))
            self.sock.sendto(packet, ('127.0.0.1', FDM_PORT))
            time.sleep(self.period)


def set_interval(link, message_id, interval_us):
    link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                              0, message_id, interval_us, 0, 0, 0, 0, 0)
    settling = collect(link, .5)
    acknowledgements = [message for message in settling['COMMAND_ACK']
                        if message.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL]
    assert acknowledgements, f'No SET_MESSAGE_INTERVAL acknowledgement for {message_id}'
    assert all(message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
               for message in acknowledgements), acknowledgements


def check_other_streams(data, expect_imu=True):
    gps_rate = rate(data['GLOBAL_POSITION_INT'], 'time_boot_ms', 1e3)
    assert 9 <= gps_rate <= 11, f'GPS changed to {gps_rate:.1f} Hz'
    if expect_imu:
        imu_rate = rate(data['HIGHRES_IMU'], 'time_usec', 1e6)
        assert 225 <= imu_rate <= 260, f'IMU changed to {imu_rate:.1f} Hz'
    else:
        assert not data['HIGHRES_IMU'], 'IMU was not disabled independently'


def check_pressure(data, altitude_m, minimum_rate, maximum_rate):
    messages = data['SCALED_PRESSURE']
    actual = rate(messages, 'time_boot_ms', 1e3)
    assert minimum_rate <= actual <= maximum_rate, f'Unexpected pressure rate {actual:.1f} Hz'
    expected_hpa = pressure_pa(altitude_m) / 100.0
    for message in messages:
        assert math.isclose(message.press_abs, expected_hpa, abs_tol=.001), message
        assert message.temperature == 2500, message
        assert message.press_diff == 0, message
    stamps = [message.time_boot_ms for message in messages]
    assert stamps[0] > 0
    # The virtual barometer needs 10 ms for conversion alone. A high requested
    # output rate must not relabel the same sample with successive send times.
    assert all(second - first >= 10 for first, second in zip(stamps, stamps[1:])), stamps
    # Pressure timestamps must be boot-relative, on the same clock as IMU/GPS.
    # Do not compare ATTITUDE: it uses millis(), and the existing SITL millis
    # and micros clocks integrate simRate independently and can drift apart.
    # The sub-millisecond pressure precision is intentionally lost.
    newest_gps = data['GLOBAL_POSITION_INT'][-1].time_boot_ms
    assert abs(stamps[-1] - newest_gps) < 200, (stamps[-1], newest_gps)
    if data['HIGHRES_IMU']:
        newest_imu_ms = data['HIGHRES_IMU'][-1].time_usec / 1000
        assert abs(stamps[-1] - newest_imu_ms) < 200, (stamps[-1], newest_imu_ms)
    return actual


def main():
    if len(sys.argv) != 2:
        raise SystemExit(f'Usage: {sys.argv[0]} obj/main/betaflight_SITL.elf')
    with tempfile.TemporaryDirectory(prefix='bf-mavlink-baro-') as directory:
        sitl = Sitl(sys.argv[1], directory)
        feed = BaroFdmFeed()
        link = None
        try:
            sitl.provision(['feature TELEMETRY', 'feature GPS',
                            'serial 1 512 115200 115200 921600 115200',
                            'set mavlink_imu_rate = 250',
                            'set mavlink_pos_rate = 10',
                            'set mavlink_extra1_rate = 100'])
            feed.start()
            sitl.start()
            link = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
            collect(link, 2)
            initial = collect(link)
            actual = check_pressure(initial, feed.altitude_m, 36, 42)
            check_other_streams(initial)
            print(f'Default pressure: {actual:.1f} Hz, {initial["SCALED_PRESSURE"][-1].press_abs:.2f} hPa')

            # A height step must change absolute pressure, without waiting for
            # flight-controller ground-reference/altitude calibration.
            feed.altitude_m += 100
            collect(link, .5)
            changed = collect(link)
            check_pressure(changed, feed.altitude_m, 36, 42)
            assert changed['SCALED_PRESSURE'][-1].press_abs < initial['SCALED_PRESSURE'][-1].press_abs
            check_other_streams(changed)

            # Configure pressure independently, disable it, restore the 40 Hz
            # default, then request faster output than the sensor can produce.
            for interval, low, high in [(50000, 18, 21), (-1, 0, 0),
                                        (0, 36, 42), (2000, 60, 100)]:
                set_interval(link, PRESSURE_ID, interval)
                data = collect(link)
                check_other_streams(data)
                if interval < 0:
                    assert not data['SCALED_PRESSURE'], 'Pressure did not stop'
                    print('Pressure disabled; GPS and IMU continue')
                else:
                    actual = check_pressure(data, feed.altitude_m, low, high)
                    print(f'Pressure interval {interval} us: {actual:.1f} Hz')

            set_interval(link, PRESSURE_ID, 0)
            set_interval(link, IMU_ID, -1)
            data = collect(link)
            check_pressure(data, feed.altitude_m, 36, 42)
            check_other_streams(data, expect_imu=False)
            set_interval(link, IMU_ID, 0)

            # At this test altitude, the virtual sensor returns zero Pa. It
            # must not be published as a valid measurement or replay the last
            # good sample indefinitely. Restore valid pressure afterwards.
            saved_altitude = feed.altitude_m
            feed.altitude_m = 44000
            assert pressure_pa(feed.altitude_m) == 0
            collect(link, 1)
            invalid = collect(link)
            assert not invalid['SCALED_PRESSURE'], 'Invalid/stale pressure was published'
            check_other_streams(invalid)
            feed.altitude_m = saved_altitude
            collect(link, .5)
            recovered = collect(link)
            check_pressure(recovered, feed.altitude_m, 36, 42)
            check_other_streams(recovered)
            print('PASS: pressure units, temperature, boot/sample timestamps, default rate, '
                  'rate changes, deduplication, independent streams, invalid-data suppression and recovery')
        finally:
            if link:
                link.close()
            feed.shutdown()
            if feed.ident is not None:
                feed.join(timeout=1)
            feed.sock.close()
            sitl.stop()


if __name__ == '__main__':
    main()

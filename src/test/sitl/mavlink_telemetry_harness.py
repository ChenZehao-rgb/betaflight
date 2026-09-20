#!/usr/bin/env python3
"""Check MAVLink telemetry from SITL. Requires pymavlink and free ports 5761/5762.

Run: python3 src/test/sitl/mavlink_telemetry_harness.py obj/main/betaflight_SITL.elf
Rates below are measured in FC time: simulator wall-clock speed is host-dependent.
"""
import collections

import math
import struct
import sys
import tempfile
import time

from pymavlink import mavutil
sys.dont_write_bytecode = True
from sitl_harness import FdmFeed, Sitl, FDM_PORT, HOME_LAT, HOME_LON, HOME_ALT_M


class HighRateFdmFeed(FdmFeed):
    # A static level vehicle, sampled fast enough to test 500 distinct IMU updates/s.
    period = .0005

    def run(self):
        while self.running:
            pkt = struct.pack('<18d', time.monotonic() - self.t0,
                0, 0, 0, 0, 0, -9.80665,
                math.sqrt(.5), 0, 0, math.sqrt(.5),
                0, 0, 0, HOME_LON, HOME_LAT, HOME_ALT_M, 101325)
            self.sock.sendto(pkt, ('127.0.0.1', FDM_PORT))
            time.sleep(self.period)


def collect(link, seconds=3):
    result = collections.defaultdict(list)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        msg = link.recv_match(blocking=True, timeout=0.1)
        if msg:
            assert msg.get_type() != 'BAD_DATA', msg
            result[msg.get_type()].append(msg)
    return result


def rate(messages, field, scale):
    assert len(messages) > 5, 'Missing telemetry'
    return (len(messages) - 1) * scale / (getattr(messages[-1], field) - getattr(messages[0], field))


def main():
    with tempfile.TemporaryDirectory(prefix='bf-mavlink-') as directory:
        sitl = Sitl(sys.argv[1], directory)
        feed = HighRateFdmFeed()
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
            collect(link, 2)  # Let sensor initialization and calibration settle.
            data = collect(link)
            imu_rate = rate(data['HIGHRES_IMU'], 'time_usec', 1e6)
            attitude_rate = rate(data['ATTITUDE'], 'time_boot_ms', 1e3)
            gps_rate = rate(data['GLOBAL_POSITION_INT'], 'time_boot_ms', 1e3)
            print(f'FC-time rates: IMU {imu_rate:.1f}, attitude {attitude_rate:.1f}, GPS {gps_rate:.1f} Hz')
            assert 240 <= imu_rate <= 255
            assert 95 <= attitude_rate <= 105
            assert 9 <= gps_rate <= 11
            imu = data['HIGHRES_IMU'][-1]
            assert abs(imu.xacc) < 0.5 and abs(imu.yacc) < 0.5
            assert math.isclose(imu.zacc, -9.80665, abs_tol=0.5), imu
            assert max(abs(imu.xgyro), abs(imu.ygyro), abs(imu.zgyro)) < 0.1
            assert imu.fields_updated == 63
            assert abs(data['GLOBAL_POSITION_INT'][-1].time_boot_ms - data['ATTITUDE'][-1].time_boot_ms) < 200
            # Runtime rate change, disable, then restore the configured default.
            for interval, expected in [(2000, 500), (-1, 0), (0, 250)]:
                link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                           0, 105, interval, 0, 0, 0, 0, 0)
                collect(link, 0.5)
                messages = collect(link)['HIGHRES_IMU']
                if expected:
                    actual = rate(messages, 'time_usec', 1e6)
                    print(f'IMU requested {expected}, measured {actual:.1f} Hz')
                    assert expected * 0.95 <= actual <= expected * 1.02
                else:
                    assert not messages
            # A 50 Hz source must not be relabelled as 250 Hz new measurements.
            feed.period = .02
            collect(link, .5)
            slow = collect(link)
            actual = rate(slow['HIGHRES_IMU'], 'time_usec', 1e6)
            print(f'50 Hz sensor source: {actual:.1f} Hz unique IMU output')
            assert 40 <= actual <= 55
            for kind, field in [('HIGHRES_IMU', 'time_usec'), ('GPS_RAW_INT', 'time_usec')]:
                stamps = [getattr(m, field) for m in slow[kind]]
                assert len(stamps) == len(set(stamps))
            print('PASS: decoded units, sample timestamps, rates, deduplication, disable and restore')
        finally:
            if link:
                link.close()
            feed.shutdown()
            sitl.stop()


if __name__ == '__main__':
    main()

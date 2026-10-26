#!/usr/bin/env python3
"""Protocol/scheduling validation using simulated sensor inputs, NOT flight validation.

Requires pymavlink and the MAVLink-enabled SITL build documented in the protocol.
Rates use FC boot time; localhost TCP does not certify a physical 921600 UART.
"""
import collections
import math
import os
import struct
import sys
import tempfile
import time

os.environ['MAVLINK20'] = '1'
sys.dont_write_bytecode = True
from pymavlink import mavutil
from sitl_harness import FdmFeed, Sitl, FDM_PORT, HOME_LAT, HOME_LON, HOME_ALT_M


class StaticSensorFeed(FdmFeed):
    def run(self):
        while self.running:
            packet = struct.pack('<18d', time.monotonic() - self.t0,
                0, 0, 0, 0, 0, -9.80665, math.sqrt(.5), 0, 0, math.sqrt(.5),
                0, 0, 0, HOME_LON, HOME_LAT, HOME_ALT_M, 101325)
            self.sock.sendto(packet, ('127.0.0.1', FDM_PORT))
            time.sleep(.0005)


def collect(link, seconds=3):
    data = collections.defaultdict(list)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        msg = link.recv_match(blocking=True, timeout=.1)
        if msg:
            assert msg.get_type() != 'BAD_DATA', msg
            assert msg.get_msgbuf()[0] == 0xfd, msg
            data[msg.get_type()].append(msg)
    return data


def health(msg):
    assert msg.message_type == 0xBF01
    p = bytes(msg.payload)
    assert p[0:2] == b'\x01\x44'
    assert not any(p[41:44]) and not any(p[68:])
    return dict(flags=struct.unpack_from('<H', p, 2)[0],
                sequence=struct.unpack_from('<I', p, 4)[0],
                reset=struct.unpack_from('<I', p, 8)[0],
                time=struct.unpack_from('<Q', p, 12)[0],
                xy_age=struct.unpack_from('<I', p, 20)[0])


def rate(times):
    assert len(times) > 10, f'Insufficient samples: {len(times)}'
    return (len(times)-1)*1e6/(times[-1]-times[0])


def command(link, command_id, *params):
    params = list(params) + [0]*(7-len(params))
    link.mav.command_long_send(1, 1, command_id, 0, *params)
    responses = collect(link, .35)
    acks = [m for m in responses['COMMAND_ACK'] if m.command == command_id]
    assert acks, f'No ACK for {command_id}'
    return acks[-1], responses


def main():
    with tempfile.TemporaryDirectory(prefix='bf-odometry-') as directory:
        sitl = Sitl(sys.argv[1], directory)
        feed = StaticSensorFeed()
        link = None
        try:
            sitl.provision(['feature TELEMETRY', 'feature GPS',
                'serial 1 512 115200 115200 921600 115200',
                'set msp_override_timeout_ms = 50'])
            feed.start()
            sitl.start()
            link = mavutil.mavlink_connection('tcp:127.0.0.1:5762', dialect='common')
            collect(link, 2)
            data = collect(link, 5)
            assert set(data) == {'ODOMETRY', 'V2_EXTENSION'}, set(data)
            odo = data['ODOMETRY']
            h = [health(m) for m in data['V2_EXTENSION']]
            times = [m.time_usec for m in odo]
            assert all(b > a for a, b in zip(times, times[1:]))
            odo_rate, health_rate = rate(times), rate([m['time'] for m in h])
            producer_rate = (h[-1]['sequence']-h[0]['sequence'])*1e6/(h[-1]['time']-h[0]['time'])
            status = sitl.msp.request(101)
            load = struct.unpack_from('<H', status, 11)[0]
            gaps = [b-a for a,b in zip(times,times[1:])]
            print(f'FC-time rates: ODOMETRY={odo_rate:.3f}Hz HEALTH={health_rate:.3f}Hz ESTIMATOR={producer_rate:.3f}Hz', flush=True)
            print(f'SITL scheduler reported load={load}%, ODOMETRY state gaps min/max={min(gaps)}/{max(gaps)}us', flush=True)
            assert 95 <= odo_rate <= 101
            assert 48 <= health_rate <= 52
            for o in odo:
                assert o.frame_id == 1 and o.child_frame_id == 12
                assert o.quality == -1  # Disarmed horizontal fusion is unavailable.
                assert abs(sum(v*v for v in o.q)-1) < 1e-5
                assert math.isnan(o.pose_covariance[0]) and math.isnan(o.velocity_covariance[0])
                assert all(math.isfinite(v) for v in [o.x,o.y,o.z,o.vx,o.vy,o.vz])
            assert all(m['flags'] & 128 and not m['flags'] & 1 for m in h)
            assert all(m['xy_age'] == 0xffffffff for m in h)
            bytime = {m.time_usec: m for m in odo}
            paired = [m for m in h if m['time'] in bytime]
            assert len(paired) > .8*len(h)
            assert all(bytime[m['time']].reset_counter == m['reset']%256 for m in paired)
            print(f'Validated {len(odo)} ODOMETRY, {len(h)} health, {len(paired)} paired state epochs', flush=True)
            for old_id in [0,1,24,30,33,35,65,74,105,137,147]:
                ack, _ = command(link, 511, old_id, 10000)
                assert ack.result == mavutil.mavlink.MAV_RESULT_DENIED, old_id
            for msg_id, name in [(331,'ODOMETRY'),(248,'V2_EXTENSION')]:
                ack, _ = command(link,511,msg_id,-1)
                assert ack.result == 0
                assert not collect(link,.4)[name]
                ack, _ = command(link,511,msg_id,0)
                assert ack.result == 0
                assert len(collect(link,.5)[name]) > 10
            ack, responses = command(link,512,148) # AUTOPILOT_VERSION
            assert ack.result == 0 and responses['AUTOPILOT_VERSION']
            ack, responses = command(link,512,0) # On-demand HEARTBEAT
            assert ack.result == 0 and responses['HEARTBEAT']
            ack, responses = command(link,512,42) # MISSION_CURRENT on demand
            assert ack.result == 0 and responses['MISSION_CURRENT']
            for interval in [float('nan'),float('inf')]:
                ack, _ = command(link,511,331,interval)
                assert ack.result == mavutil.mavlink.MAV_RESULT_DENIED
            request_time = time.monotonic_ns()
            link.mav.timesync_send(0,request_time)
            responses = collect(link,.5)
            ts = [m for m in responses['TIMESYNC'] if m.ts1 == request_time]
            assert ts and ts[-1].tc1 > 0
            first = responses['ODOMETRY'][0].time_usec
            assert abs(ts[-1].tc1/1000-first) < 100000
            print('PASS: MAVLink2 CRC decode, state timestamps, rates, health pairing, old-stream denial, disable/restore, capabilities, HEARTBEAT and TIMESYNC', flush=True)
        except Exception:
            log_path = os.path.join(directory, 'sitl.log')
            if os.path.exists(log_path):
                print(open(log_path).read()[-12000:], flush=True)
            raise
        finally:
            if link: link.close()
            feed.shutdown()
            sitl.stop()


if __name__ == '__main__':
    main()

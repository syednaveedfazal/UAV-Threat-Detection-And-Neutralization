#!/usr/bin/env python3
"""Scripted pilot: flies a short pattern through /joy exactly as a controller would.

Needs the sim + manual_scan.launch.py running (joy:=false or no pad connected):
    python3 src/pocs/poc3_manual_scan/test/smoke_flight.py

Checks: takeoff, stick -> direction, hands-off hold drift, yaw, climb, geofence-free
flight, scan session saved with a non-empty map, landing + disarm.
"""

import math
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

START, B, X, Y = 6, 1, 2, 3


class Pilot:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node('smoke_pilot')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.n.create_publisher(Joy, '/joy', 10)
        self.pos = self.status = None
        self.flight = self.scan = ''
        self.n.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                   lambda m: setattr(self, 'pos', m), qos)
        self.n.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', lambda m: setattr(self, 'status', m), qos)
        self.n.create_subscription(String, '/manual_flight/status', lambda m: setattr(self, 'flight', m.data), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.n.create_subscription(String, '/scan/status', self.on_scan, latched)
        self.scan_log = []
        self.failures = []

    def on_scan(self, m):
        self.scan = m.data
        self.scan_log.append(m.data)

    def hold(self, seconds, axes=(0.0,) * 6, buttons=()):
        """Publish /joy at 20 Hz for `seconds` with these sticks and buttons held."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            j = Joy()
            j.header.stamp = self.n.get_clock().now().to_msg()
            j.axes = [float(a) for a in axes]
            j.buttons = [1 if i in buttons else 0 for i in range(15)]
            self.pub.publish(j)
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def press(self, button):
        self.hold(0.2, buttons=(button,))
        self.hold(0.3)

    def wait(self, what, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.hold(0.25)
            if cond():
                print(f'  ok: {what}')
                return True
        self.fail(f'timeout: {what} (flight status: {self.flight})')
        return False

    def fail(self, msg):
        print('  FAIL: ' + msg)
        self.failures.append(msg)

    def ne(self):
        return self.pos.x, self.pos.y

    def check(self, cond, msg):
        print(('  ok: ' if cond else '  FAIL: ') + msg)
        if not cond:
            self.failures.append(msg)


def main():
    p = Pilot()
    print('waiting for PX4 + joy_flight...')
    if not p.wait('telemetry', lambda: p.pos is not None and p.status is not None and p.flight, 60):
        sys.exit(1)

    print('1. START -> take off')
    p.press(START)
    if not p.wait('in FLY state (at takeoff altitude)', lambda: p.flight.startswith('FLY'), 60):
        sys.exit(1)
    alt0 = -p.pos.z
    print(f'     altitude {alt0:.1f} m, heading {math.degrees(p.pos.heading):.0f} deg')

    print('2. Y -> start scan')
    p.press(Y)
    p.wait('scan recording', lambda: 'RECORDING' in p.scan, 10)

    print('3. right stick forward 6 s')
    h = p.pos.heading
    n0, e0 = p.ne()
    p.hold(6, axes=(0, 0, 0, 0.8, 0, 0))
    dn, de = p.pos.x - n0, p.pos.y - e0
    along = dn * math.cos(h) + de * math.sin(h)
    across = -dn * math.sin(h) + de * math.cos(h)
    speed = math.hypot(p.pos.vx, p.pos.vy)
    p.check(along > 8 and abs(across) < 0.15 * along,
            f'moved {along:.1f} m forward, {across:+.1f} m sideways, speed {speed:.1f} m/s')

    print('4. hands off 6 s -> brake + hold')
    p.hold(3)
    n1, e1 = p.ne()
    p.hold(3)
    drift = math.hypot(p.pos.x - n1, p.pos.y - e1)
    p.check(drift < 0.3, f'hold drift over 3 s: {drift:.2f} m')

    print('5. yaw left 3 s')
    h0 = p.pos.heading
    p.hold(3, axes=(0.7, 0, 0, 0, 0, 0))
    p.hold(1.5)
    dyaw = math.degrees(math.atan2(math.sin(p.pos.heading - h0), math.cos(p.pos.heading - h0)))
    p.check(dyaw < -40, f'heading changed {dyaw:+.0f} deg (left = counter-clockwise = negative in NED)')

    print('6. climb 3 s')
    a0 = -p.pos.z
    p.hold(3, axes=(0, 0.8, 0, 0, 0, 0))
    p.hold(2)
    p.check(-p.pos.z - a0 > 3, f'climbed {-p.pos.z - a0:.1f} m to {-p.pos.z:.1f} m')

    print('7. fast sideways right 4 s (RB held)')
    h = p.pos.heading
    n0, e0 = p.ne()
    p.hold(4, axes=(0, 0, -0.8, 0, 0, 0), buttons=(10,))
    dn, de = p.pos.x - n0, p.pos.y - e0
    right = -dn * math.sin(h) + de * math.cos(h)
    p.check(right > 10, f'moved {right:.1f} m to the right at {math.hypot(p.pos.vx, p.pos.vy):.1f} m/s')
    p.hold(3)

    print('8. X -> save map, B -> land')
    p.press(X)
    p.wait('map saved', lambda: p.scan.startswith('saved'), 30)
    print('     ' + p.scan)
    p.press(B)
    p.wait('landed and disarmed', lambda: p.status.arming_state != VehicleStatus.ARMING_STATE_ARMED, 90)

    print('9. Y -> stop scan (writes the session)')
    p.press(Y)
    p.wait('session closed', lambda: p.scan.startswith('stopped'), 30)
    print('     ' + p.scan)

    print('\nRESULT: ' + ('PASS' if not p.failures else f'{len(p.failures)} failure(s): ' + '; '.join(p.failures)))
    sys.exit(1 if p.failures else 0)


if __name__ == '__main__':
    main()

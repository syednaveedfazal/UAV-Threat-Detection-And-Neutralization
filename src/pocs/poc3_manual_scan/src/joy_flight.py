#!/usr/bin/env python3
"""Fly the PX4 drone with a game controller (offboard velocity over uXRCE-DDS).

Mode 2 sticks, like an RC transmitter:
    left stick   up/down = climb/descend      left/right = yaw
    right stick  up/down = forward/back       left/right = sideways
    LB held = precise (slow)    RB held = fast
    START = arm + take off      B = land
    Y = start/stop scan recording    X = save the map now

Sticks are velocity commands in the drone's heading frame. Let go of a stick
and that axis brakes, then holds its position (per axis), so the drone hovers
in place hands-off - the behaviour of PX4's Position mode, rebuilt here
because offboard setpoints bypass PX4's own stick handling.

Safety, mirroring what a real flight controller does:
    * geofence: the site's area (scene manifest) minus a margin, 2..120 m
      altitude (120 m is the EU open-category limit); the drone slows down
      towards the fence and cannot cross it
    * controller lost (no /joy for 0.5 s): hold position; after
      joy_lost_land_s: land - like an RC-loss failsafe
    * if anyone else changes the flight mode (QGroundControl, PX4 failsafe),
      this node stops asking for OFFBOARD until START is pressed again:
      it never fights another pilot or the autopilot's own safety logic

The button/axis numbers below are SDL GameController indices, which
game_controller_node uses for every supported pad (Xbox, PlayStation, 8BitDo...).
Check your pad with:  ros2 run poc3_manual_scan joy_flight.py --ros-args -p show_inputs:=true
"""

import json
import math
import time
from pathlib import Path

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

import flight_math as fm

NAN = float('nan')
IDLE, TAKEOFF, FLY, LANDING = 'IDLE', 'TAKEOFF', 'FLY', 'LANDING'


class JoyFlight(Node):
    def __init__(self):
        super().__init__('joy_flight')
        P = self.declare_parameter
        P('status_topic', '/fmu/out/vehicle_status_v4')
        P('local_position_topic', '/fmu/out/vehicle_local_position_v1')
        P('rate_hz', 20.0)
        # controller layout (SDL GameController indices, ROS sign: up/left = +1)
        P('axis_yaw', 0); P('axis_throttle', 1); P('axis_roll', 2); P('axis_pitch', 3)
        P('button_takeoff', 6); P('button_land', 1); P('button_record', 3); P('button_save', 2)
        P('button_slow', 9); P('button_fast', 10)
        P('deadzone', 0.08); P('expo', 0.4)
        P('show_inputs', False)
        # speeds
        P('max_speed_xy', 4.0); P('max_speed_z', 2.0); P('max_yaw_rate_deg', 60.0)
        P('slow_factor', 0.3); P('fast_factor', 2.5)
        P('max_accel_xy', 3.0); P('max_accel_z', 2.0)
        P('takeoff_altitude', 8.0)
        # safety
        P('manifest', str(Path.home() / 'UAV/data/bonn_poppelsdorf/world/scene/scene_manifest.json'))
        P('fence_region', 'full'); P('fence_margin', 10.0)
        P('alt_min', 2.0); P('alt_max', 120.0)
        P('joy_timeout_s', 0.5); P('joy_lost_land_s', 15.0)
        self.p = {n: v.value for n, v in self.get_parameters_by_prefix('').items()}

        self.fence = self.load_fence()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.pub_scan = self.create_publisher(String, '/scan/command', 10)
        self.pub_status = self.create_publisher(String, '/manual_flight/status', 10)
        self.create_subscription(VehicleStatus, self.p['status_topic'], self.on_status, qos)
        self.create_subscription(VehicleLocalPosition, self.p['local_position_topic'], self.on_pos, qos)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)

        self.status: VehicleStatus | None = None
        self.pos: VehicleLocalPosition | None = None
        self.joy: Joy | None = None
        self.prev_buttons: list[int] = []
        self.last_joy = -1e9
        self.state = IDLE
        self.state_since = time.monotonic()
        # True from START until PX4 is in OFFBOARD and armed. Only then do we
        # request modes; afterwards any mode change by someone else is respected.
        self.want_offboard = False
        self.takeoff_target = None
        self.v = [0.0, 0.0, 0.0]              # commanded NED velocity (after slew)
        self.hold_xy = None
        self.hold_z = None
        self.hold_yaw = None
        self.tick = 0
        self.dt = 1.0 / self.p['rate_hz']
        self.create_timer(self.dt, self.loop)
        self.get_logger().info(
            f"joy_flight ready. START = take off, B = land, Y = record scan, X = save map. "
            f"Geofence N {self.fence.north_min:.0f}..{self.fence.north_max:.0f} m, "
            f"E {self.fence.east_min:.0f}..{self.fence.east_max:.0f} m, "
            f"alt {self.fence.alt_min:.0f}..{self.fence.alt_max:.0f} m")

    def load_fence(self) -> fm.Geofence:
        try:
            region = json.loads(Path(self.p['manifest']).read_text())['regions'][self.p['fence_region']]
            return fm.Geofence.from_enu_region(region, self.p['fence_margin'], self.p['alt_min'], self.p['alt_max'])
        except (OSError, KeyError, ValueError) as e:
            self.get_logger().warn(f'no site geofence ({e}); using +-200 m around home')
            return fm.Geofence(-200, 200, -200, 200, self.p['alt_min'], self.p['alt_max'])

    # ------------------------------------------------------------- inputs --
    def on_status(self, m):
        self.status = m

    def on_pos(self, m):
        self.pos = m

    def on_joy(self, m: Joy):
        if self.p['show_inputs']:
            self.show_inputs(m)
        pressed = [i for i, b in enumerate(m.buttons)
                   if b and (i >= len(self.prev_buttons) or not self.prev_buttons[i])]
        self.prev_buttons = list(m.buttons)
        self.joy, self.last_joy = m, time.monotonic()
        for b in pressed:
            self.on_button(b)

    def show_inputs(self, m: Joy):
        axes = ' '.join(f'{i}:{a:+.2f}' for i, a in enumerate(m.axes) if abs(a) > 0.2)
        btns = ' '.join(str(i) for i, b in enumerate(m.buttons) if b)
        if axes or btns:
            self.get_logger().info(f'axes [{axes}]  buttons [{btns}]', throttle_duration_sec=0.2)

    def on_button(self, b: int):
        p = self.p
        if b == p['button_takeoff']:
            if self.state in (IDLE, LANDING):
                if self.pos is None or not (self.pos.xy_valid and self.pos.z_valid):
                    self.get_logger().warn('no position estimate from PX4 yet - cannot take off')
                    return
                alt = -self.pos.z
                if self.armed and alt > 1.0:
                    self.set_state(FLY, 'taking control in the air, holding here')
                    self.want_offboard = True
                    return
                self.takeoff_target = (self.pos.x, self.pos.y, -(max(alt, 0.0) + p['takeoff_altitude']),
                                       self.pos.heading)
                self.set_state(TAKEOFF, f"take off to {p['takeoff_altitude']:.0f} m")
            self.want_offboard = True
        elif b == p['button_land'] and self.state in (TAKEOFF, FLY):
            self.set_state(LANDING, 'landing')
            self.want_offboard = False
            self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        elif b == p['button_record']:
            self.pub_scan.publish(String(data='toggle'))
        elif b == p['button_save']:
            self.pub_scan.publish(String(data='save'))

    # -------------------------------------------------------------- PX4 io --
    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def command(self, cmd, p1=0.0, p2=0.0):
        m = VehicleCommand(command=cmd, param1=float(p1), param2=float(p2), target_system=1, target_component=1,
                           source_system=1, source_component=1, from_external=True, timestamp=self.now_us())
        self.pub_cmd.publish(m)

    def send_setpoint(self, pos=(NAN, NAN, NAN), vel=(NAN, NAN, NAN), yaw=NAN, yawspeed=NAN):
        # NaN = "not controlled on this axis": PX4 then uses whichever of
        # position / velocity is finite per axis - that is how one axis can
        # hold position while another follows the stick.
        self.pub_mode.publish(OffboardControlMode(position=True, velocity=True, timestamp=self.now_us()))
        # /fmu/in/trajectory_setpoint IS PX4's internal trajectory_setpoint topic,
        # the one its own modes (Land, Position, Hold...) write to. Publishing it
        # outside OFFBOARD fights the active mode - e.g. Land never sees low
        # throttle and never disarms. So setpoints go out only in OFFBOARD, or
        # in the lead-in PX4 requires right before switching to it.
        if not (self.offboard or self.want_offboard):
            return
        self.pub_sp.publish(TrajectorySetpoint(position=[float(v) for v in pos], velocity=[float(v) for v in vel],
                                               acceleration=[NAN] * 3, jerk=[NAN] * 3,
                                               yaw=float(yaw), yawspeed=float(yawspeed), timestamp=self.now_us()))

    @property
    def armed(self):
        return self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    @property
    def offboard(self):
        return self.status is not None and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def set_state(self, s, why=''):
        if s != self.state:
            self.get_logger().info(f'{self.state} -> {s}' + (f': {why}' if why else ''))
        self.state, self.state_since = s, time.monotonic()
        if s in (TAKEOFF, FLY):
            self.v = [0.0, 0.0, 0.0]
            self.hold_xy = self.hold_z = self.hold_yaw = None

    # ---------------------------------------------------------- main loop --
    def loop(self):
        self.tick += 1
        if self.pos is None or self.status is None:
            if self.tick % (5 * int(self.p['rate_hz'])) == 1:
                self.get_logger().info('waiting for PX4 telemetry (is MicroXRCEAgent running?)')
            return
        once_per_s = self.tick % int(self.p['rate_hz']) == 0

        # Someone else took over (QGC mode switch, PX4 failsafe): stand down.
        in_air_states = (TAKEOFF, FLY)
        if self.state in in_air_states and self.armed and not self.offboard and not self.want_offboard \
                and time.monotonic() - self.state_since > 3.0:
            self.set_state(IDLE, f'flight mode changed outside this node (nav_state {self.status.nav_state}); '
                                 'standing down - press START to take control again')
        if self.state == LANDING and not self.armed:
            self.set_state(IDLE, 'landed and disarmed')

        if self.state == TAKEOFF:
            n, e, d, yaw = self.takeoff_target
            self.send_setpoint(pos=(n, e, d), yaw=yaw)
            if self.offboard and self.armed and abs(self.pos.z - d) < 0.5:
                self.set_state(FLY, 'at altitude - sticks are live')
        elif self.state == FLY:
            self.fly()
        else:
            # Heartbeat only (send_setpoint drops the setpoint outside OFFBOARD):
            # keeps the offboard link "alive" for PX4 without touching other modes.
            self.send_setpoint(pos=(self.pos.x, self.pos.y, self.pos.z), yaw=self.pos.heading)

        # Ask for OFFBOARD and arming only while the pilot wants it, retried at 1 Hz.
        if self.want_offboard and self.state in in_air_states and once_per_s:
            if not self.offboard:
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)   # custom mode 6 = OFFBOARD
            elif not self.armed:
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            else:
                self.want_offboard = False      # done: in OFFBOARD and armed

        if once_per_s:
            self.report()

    def fly(self):
        p, pos, joy = self.p, self.pos, self.joy
        age = time.monotonic() - self.last_joy
        if age > p['joy_lost_land_s']:
            self.get_logger().error(f'controller lost for {age:.0f} s - landing (RC-loss failsafe)')
            self.set_state(LANDING, 'controller lost')
            self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            return
        sticks = [0.0] * 4
        factor = 1.0
        if joy is not None and age <= p['joy_timeout_s']:
            ax = lambda i: joy.axes[i] if i < len(joy.axes) else 0.0              # noqa: E731
            bt = lambda i: i < len(joy.buttons) and joy.buttons[i]               # noqa: E731
            sticks = [fm.shape(ax(p[k]), p['deadzone'], p['expo'])
                      for k in ('axis_pitch', 'axis_roll', 'axis_throttle', 'axis_yaw')]
            factor = p['slow_factor'] if bt(p['button_slow']) else p['fast_factor'] if bt(p['button_fast']) else 1.0
        elif age > p['joy_timeout_s']:
            self.get_logger().warn('no controller input - holding position', throttle_duration_sec=2.0)
        fwd, left, up, yaw_left = sticks
        alt = -pos.z

        # target velocity: body frame -> NED, then the geofence
        vn, ve = fm.body_to_ned(fwd * p['max_speed_xy'] * factor, -left * p['max_speed_xy'] * factor, pos.heading)
        vup = up * p['max_speed_z'] * min(factor, 1.5)
        vn, ve, vup = self.fence.limit(pos.x, pos.y, alt, vn, ve, vup)
        if not self.fence.contains(pos.x, pos.y, alt):
            self.get_logger().warn('outside the geofence - only movement back inside is allowed',
                                   throttle_duration_sec=3.0)

        # acceleration limit, so stick jabs do not become attitude jumps
        a_xy, a_z = p['max_accel_xy'] * self.dt, p['max_accel_z'] * self.dt
        self.v = [fm.slew(self.v[0], vn, a_xy), fm.slew(self.v[1], ve, a_xy), fm.slew(self.v[2], -vup, a_z)]

        # per-axis: stick active -> velocity; released -> brake, then hold position
        sp_pos, sp_vel = [NAN, NAN, NAN], list(self.v)
        if fwd == 0 and left == 0:
            if self.hold_xy is None and math.hypot(pos.vx, pos.vy) < 0.3 and math.hypot(*self.v[:2]) < 0.05:
                self.hold_xy = (pos.x, pos.y)
            if self.hold_xy is not None:
                sp_pos[0], sp_pos[1] = self.hold_xy
                sp_vel[0] = sp_vel[1] = NAN
        else:
            self.hold_xy = None
        if up == 0:
            if self.hold_z is None and abs(pos.vz) < 0.2 and abs(self.v[2]) < 0.05:
                self.hold_z = max(pos.z, -p['alt_max'])
            if self.hold_z is not None:
                sp_pos[2], sp_vel[2] = self.hold_z, NAN
        else:
            self.hold_z = None

        if yaw_left == 0:
            if self.hold_yaw is None:
                self.hold_yaw = pos.heading
            self.send_setpoint(sp_pos, sp_vel, yaw=self.hold_yaw)
        else:
            self.hold_yaw = None
            # NED yaw is clockwise-positive; stick left (+) must turn left
            self.send_setpoint(sp_pos, sp_vel, yawspeed=-yaw_left * math.radians(p['max_yaw_rate_deg']) * factor)

    def report(self):
        pos = self.pos
        speed = math.hypot(pos.vx, pos.vy)
        txt = (f'{self.state:8s} {"ARMED" if self.armed else "disarmed"} nav {self.status.nav_state:2d}  '
               f'N {pos.x:7.1f} E {pos.y:7.1f} alt {-pos.z:5.1f} m  speed {speed:4.1f} m/s  '
               f'heading {math.degrees(pos.heading) % 360:5.1f}')
        self.pub_status.publish(String(data=txt))
        if self.state != IDLE and self.tick % (5 * int(self.p['rate_hz'])) == 0:
            self.get_logger().info(txt)


def main():
    rclpy.init()
    node = JoyFlight()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

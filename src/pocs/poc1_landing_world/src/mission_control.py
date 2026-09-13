#!/usr/bin/env python3
"""Offboard waypoint mission for the PX4 x500 in the landing_mission world.

PX4 will only accept OFFBOARD mode after it has already been receiving a
stream of OffboardControlMode *and* TrajectorySetpoint messages, so this node
streams setpoints first and only then asks for the mode change and arming.
Both commands are retried at ~1 Hz instead of being re-sent every tick.

Topic names are versioned in PX4 v1.16+ (vehicle_status_v4,
vehicle_local_position_v1). They are exposed as parameters so the node can be
pointed at a different PX4 release without editing the source:

    ros2 run poc1_landing_world mission_control.py \
        --ros-args -p status_topic:=/fmu/out/vehicle_status_v4
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)

# Mission phases
WAIT_TELEMETRY = 0
STREAM_SETPOINTS = 1
REQUEST_OFFBOARD = 2
ARM = 3
NAVIGATE = 4
LAND = 5
DONE = 6


class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')

        # PX4 publishes with BEST_EFFORT + TRANSIENT_LOCAL over uXRCE-DDS;
        # a subscription must not request stronger guarantees than that.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        status_topic = self.declare_parameter(
            'status_topic', '/fmu/out/vehicle_status_v4').value
        local_position_topic = self.declare_parameter(
            'local_position_topic', '/fmu/out/vehicle_local_position_v1').value
        self.takeoff_altitude = float(self.declare_parameter(
            'takeoff_altitude', 10.0).value)

        # Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.create_subscription(
            VehicleStatus, status_topic, self.vehicle_status_callback, qos_profile)
        self.create_subscription(
            VehicleLocalPosition, local_position_topic,
            self.vehicle_local_position_callback, qos_profile)

        self.vehicle_status = None
        self.vehicle_local_position = None

        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        self.phase = WAIT_TELEMETRY
        self.tick = 0
        self.phase_tick = 0
        self.waypoint_index = 0

        alt = -abs(self.takeoff_altitude)  # NED: negative is up
        # Airborne waypoints in the local NED frame. The descent is handled by
        # PX4's LAND mode rather than by a z=0 setpoint, which would otherwise
        # command the vehicle into the ground.
        self.waypoints = [
            [0.0, 0.0, alt],    # Takeoff / climb
            [5.0, 5.0, alt],    # Base station (landing pad 0)
            [-4.0, 2.0, alt],   # Outpost (landing pad 1)
            [0.0, 0.0, alt],    # Return home
        ]
        self.error_threshold = 0.5  # acceptance radius, metres

        self.get_logger().info(
            f"mission_control: status='{status_topic}' "
            f"position='{local_position_topic}' altitude={self.takeoff_altitude}m")

    # ---------------- callbacks ----------------

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    # ---------------- command helpers ----------------

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def engage_offboard_mode(self):
        # base mode 1 = custom, custom main mode 6 = OFFBOARD
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = 0.0  # NED: 0 rad points north
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    # ---------------- state helpers ----------------

    def set_phase(self, phase, message=None):
        self.phase = phase
        self.phase_tick = 0
        if message:
            self.get_logger().info(message)

    def is_armed(self):
        return (self.vehicle_status is not None
                and self.vehicle_status.arming_state
                == VehicleStatus.ARMING_STATE_ARMED)

    def is_offboard(self):
        return (self.vehicle_status is not None
                and self.vehicle_status.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def once_per_second(self):
        return self.phase_tick % 10 == 0

    def distance_to_target(self, target):
        p = self.vehicle_local_position
        return math.sqrt(
            (target[0] - p.x) ** 2 + (target[1] - p.y) ** 2 + (target[2] - p.z) ** 2)

    # ---------------- main loop ----------------

    def cmdloop_callback(self):
        self.tick += 1
        self.phase_tick += 1

        if self.phase == WAIT_TELEMETRY:
            ready = (self.vehicle_status is not None
                     and self.vehicle_local_position is not None
                     and self.vehicle_local_position.xy_valid
                     and self.vehicle_local_position.z_valid)
            if ready:
                self.set_phase(STREAM_SETPOINTS,
                               'Telemetry and EKF ready; streaming setpoints')
            elif self.once_per_second():
                self.get_logger().info('Waiting for PX4 telemetry / EKF...')
            return

        # From here on PX4 must see an uninterrupted setpoint stream.
        self.publish_offboard_control_mode()
        target = self.waypoints[min(self.waypoint_index, len(self.waypoints) - 1)]
        if self.phase != LAND and self.phase != DONE:
            self.publish_trajectory_setpoint(*target)

        if self.phase == STREAM_SETPOINTS:
            # PX4 needs roughly a second of setpoints before it will accept
            # the OFFBOARD mode request.
            if self.phase_tick >= 15:
                self.set_phase(REQUEST_OFFBOARD, 'Requesting OFFBOARD mode')

        elif self.phase == REQUEST_OFFBOARD:
            if self.is_offboard():
                self.set_phase(ARM, 'OFFBOARD engaged; arming')
            elif self.once_per_second():
                self.engage_offboard_mode()

        elif self.phase == ARM:
            if self.is_armed():
                self.set_phase(NAVIGATE, 'Armed; flying waypoints')
            elif self.once_per_second():
                self.arm()

        elif self.phase == NAVIGATE:
            if not self.is_offboard():
                self.set_phase(REQUEST_OFFBOARD, 'Lost OFFBOARD; re-requesting')
                return
            if self.distance_to_target(target) < self.error_threshold:
                self.get_logger().info(
                    f'Reached waypoint {self.waypoint_index + 1}/{len(self.waypoints)}')
                self.waypoint_index += 1
                if self.waypoint_index >= len(self.waypoints):
                    self.set_phase(LAND, 'Waypoints complete; landing')

        elif self.phase == LAND:
            if self.once_per_second():
                self.land()
            if not self.is_armed() and self.phase_tick > 20:
                self.set_phase(DONE, 'Landed and disarmed; mission complete')

        elif self.phase == DONE:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

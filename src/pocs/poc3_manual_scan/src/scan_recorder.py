#!/usr/bin/env python3
"""Build a 3D map from the drone's LiDAR while you fly, and record the raw scans.

Commands on /scan/command (std_msgs/String), sent by joy_flight's buttons:
    toggle  start / stop a scan session        save  write the map now

Each session goes to <out_dir>/<YYYYmmdd_HHMMSS>/:
    map.pcd          accumulated map, gz_world (ENU, origin = launch pad), voxel-thinned
    trajectory.csv   sim time, drone position and orientation at every keyframe
    raw/             rosbag2 of the raw LiDAR, pose, IMU and TF - the input for
                     LiDAR-inertial odometry (FAST-LIO, phase L2) later
    session.json     settings and counts

How the map is built - and the caveat: every scan is placed in the world with
Gazebo's GROUND-TRUTH drone pose. That gives a perfect reference map (phase L0)
and is the yardstick odometry will be measured against, but a real drone has no
ground truth: there the pose comes from FAST-LIO. Only the pose source changes.

Only keyframes go into the map (every keyframe_dist m, keyframe_angle deg or
keyframe_period s), because hovering would otherwise add the same scan 10x/s.

Also publishes TF gz_world -> base_link from the same pose, so rviz2 can show
the live scan in place (the static base_link -> lidar_link comes from
sensors.launch.py), and the map on /scan/map for rviz2 (latched).
"""

import csv
import json
import math
import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import TransformBroadcaster

import scan_map as sm


def stamp_s(h: Header) -> float:
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class ScanRecorder(Node):
    def __init__(self):
        super().__init__('scan_recorder')
        P = self.declare_parameter
        P('cloud_topic', '/lidar_3d/points')
        P('pose_topic', '/model/x500_threat_scanner_0/pose')
        P('world_frame', 'gz_world'); P('base_frame', 'base_link')
        P('lidar_xyz', [0.0, 0.0, 0.13]); P('lidar_rpy', [0.0, 0.0, 0.0])     # base_link -> lidar_link (model.sdf)
        P('map_voxel', 0.15); P('scan_voxel', 0.15)
        P('min_range', 1.0); P('max_range', 90.0)
        P('keyframe_dist', 0.5); P('keyframe_angle_deg', 10.0); P('keyframe_period', 1.0)
        P('out_dir', str(Path.home() / 'UAV/data/scans'))
        P('record_bag', True)
        P('bag_topics', ['/lidar_3d/points', '/model/x500_threat_scanner_0/pose', '/imu', '/clock',
                         '/tf', '/tf_static', '/fmu/out/vehicle_odometry'])
        P('publish_tf', True)
        P('map_publish_period', 2.0); P('display_max_points', 1_500_000)
        self.p = {n: v.value for n, v in self.get_parameters_by_prefix('').items()}

        self.lidar_pos = np.array(self.p['lidar_xyz'], float)
        self.lidar_q = sm.rpy_to_quat(*self.p['lidar_rpy'])
        self.poses = sm.PoseBuffer()
        self.map = sm.VoxelMap(self.p['map_voxel'])
        self.recording = False
        self.session: Path | None = None
        self.bag: subprocess.Popen | None = None
        self.traj: list[tuple] = []
        self.last_kf = None                  # (t, pos, yaw)
        self.n_scans = self.n_keyframes = self.n_no_pose = 0

        self.tf = TransformBroadcaster(self) if self.p['publish_tf'] else None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_map = self.create_publisher(PointCloud2, '/scan/map', latched)
        self.pub_status = self.create_publisher(String, '/scan/status', latched)
        self.create_subscription(PoseStamped, self.p['pose_topic'], self.on_pose, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.p['cloud_topic'], self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(String, '/scan/command', self.on_command, 10)
        self.create_timer(self.p['map_publish_period'], self.publish_map)
        self.status('ready - press Y on the controller to start scanning')

    # ---------------------------------------------------------------- input --
    def on_pose(self, m: PoseStamped):
        p, q = m.pose.position, m.pose.orientation
        self.poses.add(stamp_s(m.header), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = m.header.stamp, self.p['world_frame'], self.p['base_frame']
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
            t.transform.rotation = q
            self.tf.sendTransform(t)

    def on_cloud(self, m: PointCloud2):
        self.n_scans += 1
        if not self.recording:
            return
        t = stamp_s(m.header)
        pose = self.poses.at(t)
        if pose is None:
            self.n_no_pose += 1
            self.get_logger().warn('scan without a matching pose (is the pose bridge running?)',
                                   throttle_duration_sec=5.0)
            return
        pos, q = pose
        yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        if self.last_kf is not None:
            t0, p0, y0 = self.last_kf
            moved = np.linalg.norm(pos - p0) >= self.p['keyframe_dist']
            turned = abs(math.atan2(math.sin(yaw - y0), math.cos(yaw - y0))) >= math.radians(self.p['keyframe_angle_deg'])
            if not (moved or turned or t - t0 >= self.p['keyframe_period']):
                return
        self.last_kf = (t, pos, yaw)

        xyz = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        r = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(r >= self.p['min_range']) & (r <= self.p['max_range'])]    # drops self-hits and no-returns
        xyz = sm.voxel_downsample(xyz, self.p['scan_voxel'])
        # lidar -> base_link -> world
        lidar_world_pos = pos + sm.quat_to_matrix(q) @ self.lidar_pos
        self.map.add(sm.transform(xyz, lidar_world_pos, sm.quat_mul(q, self.lidar_q)))
        self.traj.append((t, *pos, *q))
        self.n_keyframes += 1

    def on_command(self, m: String):
        cmd = m.data.strip().lower()
        if cmd == 'toggle':
            self.stop() if self.recording else self.start()
        elif cmd == 'start' and not self.recording:
            self.start()
        elif cmd == 'stop' and self.recording:
            self.stop()
        elif cmd == 'save':
            self.save()
        else:
            self.get_logger().warn(f'unknown or redundant scan command: {m.data!r}')

    # ------------------------------------------------------------- sessions --
    def start(self):
        self.session = Path(self.p['out_dir']).expanduser() / datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session.mkdir(parents=True, exist_ok=True)
        self.map = sm.VoxelMap(self.p['map_voxel'])
        self.traj, self.last_kf = [], None
        self.n_keyframes = self.n_no_pose = 0
        self.recording = True
        if self.p['record_bag']:
            # A separate process: rosbag2's writer must not share this node's
            # executor, and a crash in either cannot take the other down.
            self.bag = subprocess.Popen(
                ['ros2', 'bag', 'record', '--use-sim-time', '-o', str(self.session / 'raw'),
                 '--max-cache-size', str(200 * 1024 * 1024), *self.p['bag_topics']],
                stdout=open(self.session / 'rosbag.log', 'w'), stderr=subprocess.STDOUT,
                start_new_session=True)
        self.status(f'RECORDING -> {self.session}')

    def stop(self):
        self.recording = False
        if self.bag is not None:
            os.killpg(self.bag.pid, signal.SIGINT)          # rosbag2 finalises its metadata on SIGINT
            try:
                self.bag.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.bag.pid, signal.SIGKILL)
            self.bag = None
        self.save()
        self.status(f'stopped - session saved in {self.session}')

    def save(self):
        if self.session is None:
            self.get_logger().warn('nothing to save yet - start a scan first (Y)')
            return
        t0 = time.monotonic()
        self.map.consolidate()
        sm.write_pcd(self.session / 'map.pcd', self.map.points)
        with open(self.session / 'trajectory.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['sim_time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            w.writerows(self.traj)
        length = float(np.sum(np.linalg.norm(np.diff(np.array([r[1:4] for r in self.traj]), axis=0), axis=1))) \
            if len(self.traj) > 1 else 0.0
        (self.session / 'session.json').write_text(json.dumps({
            'frame': f"{self.p['world_frame']} (ENU, metres, origin = launch pad; same frame as the scene manifest)",
            'pose_source': 'Gazebo ground truth (PosePublisher)',
            'map_points': len(self.map), 'map_voxel_m': self.p['map_voxel'],
            'keyframes': self.n_keyframes, 'scans_without_pose': self.n_no_pose,
            'flight_path_m': round(length, 1), 'recording': self.recording,
            'bag': 'raw/' if self.p['record_bag'] else None,
        }, indent=1))
        self.status(f'saved {len(self.map):,} points, {self.n_keyframes} keyframes, {length:.0f} m flown '
                    f'-> {self.session / "map.pcd"} ({time.monotonic() - t0:.1f} s)')

    # ---------------------------------------------------------------- output --
    def publish_map(self):
        if self.map.consolidate() == 0 and not self.recording:
            return
        pts = self.map.points
        if len(pts) > self.p['display_max_points']:
            pts = pts[:: int(math.ceil(len(pts) / self.p['display_max_points']))]
        h = Header(frame_id=self.p['world_frame'], stamp=self.get_clock().now().to_msg())
        self.pub_map.publish(point_cloud2.create_cloud_xyz32(h, pts))
        if self.recording:
            self.get_logger().info(f'scanning: {len(self.map):,} map points, {self.n_keyframes} keyframes',
                                   throttle_duration_sec=10.0)

    def status(self, txt: str):
        self.get_logger().info(txt)
        if rclpy.ok():                       # also called from shutdown, after Ctrl+C
            self.pub_status.publish(String(data=txt))

    def close(self):
        if self.recording:
            self.stop()


def main():
    rclpy.init()
    node = ScanRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()                 # never lose a session on Ctrl+C
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

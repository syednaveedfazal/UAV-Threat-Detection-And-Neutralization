#!/usr/bin/env python3
"""ROS 2 side of the Unity camera bridge (protocol: docs/UNITY_BRIDGE.md).

Forwards the drone's Gazebo ground-truth pose to Unity over TCP and publishes
what Unity renders back as ordinary ROS 2 camera topics:

    /unity_cam/image_raw     sensor_msgs/Image (rgb8)
    /unity_cam/camera_info   sensor_msgs/CameraInfo
    TF  gz_world -> unity_cam_optical

Each image is stamped with the sim time of the pose it was rendered at, and the
TF carries the camera pose Unity actually rendered with, so image, TF and
labels always agree even though Unity is not lockstepped with Gazebo.

Run with use_sim_time:=true (the launch file does).
"""

import array
import json
import math
import socket
import threading

import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster

import unity_bridge_protocol as proto


def to_stamp(t: float) -> Time:
    sec = math.floor(t)
    nsec = min(int(round((t - sec) * 1e9)), 999_999_999)
    return Time(sec=int(sec), nanosec=nsec)


class UnityBridge(Node):
    def __init__(self):
        super().__init__('unity_bridge')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 5700)
        self.declare_parameter('pose_topic', '/model/x500_threat_scanner_0/pose')
        self.declare_parameter('image_topic', '/unity_cam/image_raw')
        self.declare_parameter('info_topic', '/unity_cam/camera_info')
        self.declare_parameter('world_frame', 'gz_world')
        self.declare_parameter('camera_frame', 'unity_cam_optical')
        self.declare_parameter('stats_period_s', 5.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.world_frame, self.camera_frame = p('world_frame'), p('camera_frame')

        out_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub_image = self.create_publisher(Image, p('image_topic'), out_qos)
        self.pub_info = self.create_publisher(CameraInfo, p('info_topic'), out_qos)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, p('pose_topic'), self.on_pose, qos_profile_sensor_data)

        # connection state, shared between the ROS thread and the socket threads
        self._lock = threading.Lock()
        self._conn: socket.socket | None = None
        self._pose_msg: bytes | None = None          # newest POSE, not yet sent
        self._pose_ready = threading.Condition(self._lock)
        self._newest_pose_t = float('nan')
        self._info: CameraInfo | None = None

        # stats since the last report
        self._n_frames = 0
        self._lag = []                                # newest pose t - frame t (sim s)
        self._n_poses = 0
        self.create_timer(float(p('stats_period_s')), self.report)

        self._server = socket.create_server((p('host'), int(p('port'))), reuse_port=False)
        self._server.settimeout(1.0)
        self._running = True
        threading.Thread(target=self.accept_loop, daemon=True).start()
        threading.Thread(target=self.send_loop, daemon=True).start()
        self.get_logger().info(
            f"listening on {p('host')}:{p('port')} for Unity; pose from {p('pose_topic')}")

    # ------------------------------------------------------------- ROS in --
    def on_pose(self, msg: PoseStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos, q = msg.pose.position, msg.pose.orientation
        packed = proto.pack_pose(t, (pos.x, pos.y, pos.z), (q.x, q.y, q.z, q.w))
        with self._lock:
            self._newest_pose_t = t
            self._n_poses += 1
            self._pose_msg = packed                  # older unsent pose is dropped
            self._pose_ready.notify()

    # ------------------------------------------------------------ sockets --
    def accept_loop(self):
        while self._running:
            try:
                conn, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                old, self._conn = self._conn, conn
            if old is not None:
                self.get_logger().warn('new Unity connection replaces the old one')
                old.close()
            self.get_logger().info(f'Unity connected from {addr[0]}:{addr[1]}')
            threading.Thread(target=self.read_loop, args=(conn,), daemon=True).start()

    def send_loop(self):
        """Send only the newest pose; never queue behind a slow Unity."""
        while self._running:
            with self._lock:
                self._pose_ready.wait_for(lambda: self._pose_msg is not None or not self._running,
                                          timeout=1.0)
                msg, self._pose_msg = self._pose_msg, None
                conn = self._conn
            if msg is None or conn is None:
                continue
            try:
                conn.sendall(msg)
            except OSError:
                self.drop(conn, 'send failed')

    def read_loop(self, conn: socket.socket):
        reader = proto.StreamReader()
        try:
            while True:
                data = conn.recv(1 << 20)
                if not data:
                    break
                for msg_type, payload in reader.feed(data):
                    if msg_type == proto.HELLO:
                        self.on_hello(json.loads(payload))
                    elif msg_type == proto.FRAME:
                        self.on_frame(proto.unpack_frame(payload))
                    else:
                        self.get_logger().warn(f'ignoring message type {msg_type}')
        except (OSError, ValueError) as e:
            self.drop(conn, str(e))
            return
        self.drop(conn, 'Unity disconnected')

    def drop(self, conn: socket.socket, why: str):
        with self._lock:
            if self._conn is not conn:
                return                                  # already replaced
            self._conn = None
        conn.close()
        self.get_logger().warn(f'{why}; waiting for Unity to reconnect')

    # ------------------------------------------------------------ ROS out --
    def on_hello(self, hello: dict):
        c = hello['camera']
        info = CameraInfo()
        info.header.frame_id = self.camera_frame
        info.width, info.height = int(c['width']), int(c['height'])
        info.distortion_model = c.get('distortion_model', 'plumb_bob')
        info.d = [float(v) for v in c.get('d', [0.0] * 5)]
        fx, fy, cx, cy = (float(c[k]) for k in ('fx', 'fy', 'cx', 'cy'))
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info = info
        self.get_logger().info(
            f"camera from Unity ({hello.get('site', '?')}): {info.width}x{info.height} "
            f"fx={fx:.1f} hfov={c.get('hfov_deg', float('nan')):.1f} deg")

    def on_frame(self, f: proto.Frame):
        stamp = to_stamp(f.sim_time)
        h, w, _ = f.rgb.shape
        if self._info is None:
            self.get_logger().warn('FRAME before HELLO, dropped', throttle_duration_sec=5.0)
            return
        if (w, h) != (self._info.width, self._info.height):
            self.get_logger().warn(f'frame {w}x{h} does not match HELLO '
                                   f'{self._info.width}x{self._info.height}',
                                   throttle_duration_sec=5.0)

        img = Image()
        img.header.stamp, img.header.frame_id = stamp, self.camera_frame
        img.height, img.width, img.encoding, img.step = h, w, 'rgb8', w * 3
        data = array.array('B')
        data.frombytes(f.rgb.tobytes())                 # fast path; a list would be ~100x slower
        img.data = data
        self._info.header.stamp = stamp

        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id = stamp, self.world_frame
        tf.child_frame_id = self.camera_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = \
            (float(v) for v in f.cam_pos)
        qx, qy, qz, qw = proto.matrix_to_quat(f.axes)   # columns = optical axes in world
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = float(qx), float(qy), float(qz), float(qw)

        self.tf.sendTransform(tf)
        self.pub_info.publish(self._info)
        self.pub_image.publish(img)
        with self._lock:
            self._n_frames += 1
            self._lag.append(self._newest_pose_t - f.sim_time)

    def report(self):
        period = float(self.get_parameter('stats_period_s').value)
        with self._lock:
            n, lag, poses, connected = self._n_frames, self._lag, self._n_poses, self._conn is not None
            self._n_frames, self._lag, self._n_poses = 0, [], 0
        if not connected:
            self.get_logger().info(f'no Unity connection (pose in: {poses / period:.1f} Hz)')
            return
        lag = [v for v in lag if not math.isnan(v)]
        lag_txt = (f', frame behind newest pose: mean {1e3 * sum(lag) / len(lag):.0f} ms '
                   f'max {1e3 * max(lag):.0f} ms (sim)') if lag else ''
        self.get_logger().info(f'image out: {n / period:.1f} Hz, pose in: {poses / period:.1f} Hz'
                               + lag_txt)

    def close(self):
        self._running = False
        with self._lock:
            self._pose_ready.notify_all()
            conn, self._conn = self._conn, None
        if conn is not None:
            conn.close()
        self._server.close()


def main():
    rclpy.init()
    node = UnityBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

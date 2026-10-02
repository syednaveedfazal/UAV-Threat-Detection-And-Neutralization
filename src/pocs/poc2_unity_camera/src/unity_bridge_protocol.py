"""Wire protocol and camera math for the Unity camera bridge (docs/UNITY_BRIDGE.md).

Pure Python + NumPy, no ROS imports, so it can be unit-tested anywhere and
reused by the fake Unity client.
"""

from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass

import numpy as np

MAGIC = b"GRD1"
HEADER = struct.Struct("<4sB3xI")          # magic, type, 3 reserved, payload length
HELLO, POSE, FRAME = 1, 2, 3
POSE_BODY = struct.Struct("<8d")           # sim_time, x, y, z, qx, qy, qz, qw
FRAME_FIXED = struct.Struct("<d3d9d3I")    # sim_time, cam pos, axes (right, down, fwd), w, h, enc
ENC_RGB8_TOP_FIRST, ENC_RGB8_BOTTOM_FIRST = 0, 1
MAX_PAYLOAD = 64 * 1024 * 1024


def pack(msg_type: int, payload: bytes) -> bytes:
    return HEADER.pack(MAGIC, msg_type, len(payload)) + payload


def pack_hello(camera: dict, site: str = "") -> bytes:
    return pack(HELLO, json.dumps({"protocol": 1, "site": site, "camera": camera}).encode())


def pack_pose(sim_time: float, pos, quat_xyzw) -> bytes:
    return pack(POSE, POSE_BODY.pack(sim_time, *pos, *quat_xyzw))


def unpack_pose(payload: bytes) -> tuple[float, np.ndarray, np.ndarray]:
    v = POSE_BODY.unpack(payload)
    return v[0], np.array(v[1:4]), np.array(v[4:8])


def pack_frame(sim_time: float, cam_pos, right, down, forward, rgb: np.ndarray,
               encoding: int = ENC_RGB8_TOP_FIRST) -> bytes:
    h, w, _ = rgb.shape
    fixed = FRAME_FIXED.pack(sim_time, *cam_pos, *right, *down, *forward, w, h, encoding)
    return pack(FRAME, fixed + np.ascontiguousarray(rgb, np.uint8).tobytes())


@dataclass
class Frame:
    sim_time: float
    cam_pos: np.ndarray          # ENU
    axes: np.ndarray             # 3x3, columns = right, down, forward (ENU)
    rgb: np.ndarray              # (h, w, 3) uint8, top row first


def unpack_frame(payload: bytes) -> Frame:
    v = FRAME_FIXED.unpack_from(payload)
    w, h, enc = v[13], v[14], v[15]
    data = np.frombuffer(payload, np.uint8, w * h * 3, FRAME_FIXED.size)
    rgb = data.reshape(h, w, 3)
    if enc == ENC_RGB8_BOTTOM_FIRST:
        rgb = rgb[::-1]          # GPU readback order -> image order
    elif enc != ENC_RGB8_TOP_FIRST:
        raise ValueError(f"unknown frame encoding {enc}")
    axes = np.array(v[4:13]).reshape(3, 3).T
    return Frame(v[0], np.array(v[1:4]), axes, rgb)


class StreamReader:
    """Reassembles messages from a TCP byte stream (which has no message borders)."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self._buf += data
        out = []
        while len(self._buf) >= HEADER.size:
            magic, msg_type, length = HEADER.unpack_from(self._buf)
            if magic != MAGIC:
                raise ValueError("stream out of sync (bad magic)")
            if length > MAX_PAYLOAD:
                raise ValueError(f"payload too large: {length}")
            end = HEADER.size + length
            if len(self._buf) < end:
                break
            out.append((msg_type, bytes(self._buf[HEADER.size:end])))
            del self._buf[:end]
        return out


# ------------------------------------------------------------------ camera --

def intrinsics(hfov_deg: float, width: int, height: int) -> dict:
    """Pinhole with square pixels; ROS convention (pixel centres at integers)."""
    fx = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    return {"width": width, "height": height, "fx": fx, "fy": fx,
            "cx": (width - 1) / 2, "cy": (height - 1) / 2, "hfov_deg": hfov_deg,
            "distortion_model": "plumb_bob", "d": [0.0, 0.0, 0.0, 0.0, 0.0]}


def project(points_enu, cam_pos, axes, cam: dict) -> np.ndarray:
    """World points -> pixel (u, v). axes: columns right, down, forward (ENU)."""
    p = np.atleast_2d(np.asarray(points_enu, float)) - np.asarray(cam_pos, float)
    c = p @ axes                                   # camera (optical) coordinates
    return np.column_stack([cam["fx"] * c[:, 0] / c[:, 2] + cam["cx"],
                            cam["fy"] * c[:, 1] / c[:, 2] + cam["cy"]])


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(m: np.ndarray) -> np.ndarray:
    """Rotation matrix -> (x, y, z, w), w >= 0 (Shepperd's method)."""
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
        q = [0.0] * 4
        q[i] = 0.25 * s
        q[j] = (m[j, i] + m[i, j]) / s
        q[k] = (m[k, i] + m[i, k]) / s
        q[3] = (m[k, j] - m[j, k]) / s
    q = np.array(q)
    return q if q[3] >= 0 else -q


def gimbal_camera(drone_pos, drone_quat_xyzw, mount_flu, pitch_deg: float):
    """Reference gimbal geometry (see docs/UNITY_BRIDGE.md). Returns (cam_pos, axes).

    Unity's GimbalCamera implements the same definition; the fake client uses
    this one. Both are tested against docs/frame_test_vectors.json.
    """
    r = quat_to_matrix(drone_quat_xyzw)
    cam = np.asarray(drone_pos, float) + r @ np.asarray(mount_flu, float)
    fwd_body = r[:, 0]
    psi = math.atan2(fwd_body[1], fwd_body[0])
    p = math.radians(pitch_deg)
    forward = np.array([math.cos(psi) * math.cos(p), math.sin(psi) * math.cos(p), math.sin(p)])
    right = np.array([math.sin(psi), -math.cos(psi), 0.0])
    down = np.cross(forward, right)
    return cam, np.column_stack([right, down, forward])

"""Point-cloud map building blocks: pose interpolation, voxel map, PCD writer. No ROS imports.

Frames: ROS/ENU world (gz_world), quaternions (x, y, z, w).
"""

from __future__ import annotations

import bisect
import math
from pathlib import Path

import numpy as np


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def nlerp(q0, q1, f: float) -> np.ndarray:
    """Quaternion interpolation; nlerp equals slerp to <0.1 deg over the 20 ms between poses."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    if np.dot(q0, q1) < 0:
        q1 = -q1                                 # shortest path
    q = (1 - f) * q0 + f * q1
    return q / np.linalg.norm(q)


class PoseBuffer:
    """Recent stamped poses; returns the pose at any stamp inside the buffer.

    A LiDAR scan and the pose stream are not stamped at the same instants. At
    10 m/s, using the nearest of 50 Hz poses would misplace a scan by up to
    10 cm, so poses are interpolated to the scan's stamp.
    """

    def __init__(self, horizon_s: float = 5.0):
        self.horizon = horizon_s
        self.t: list[float] = []
        self.p: list[np.ndarray] = []
        self.q: list[np.ndarray] = []

    def add(self, t: float, pos, quat):
        if self.t and t <= self.t[-1]:
            if t < self.t[-1] - 1.0:             # sim time jumped back (world reset)
                self.t, self.p, self.q = [], [], []
            else:
                return
        self.t.append(t); self.p.append(np.asarray(pos, float)); self.q.append(np.asarray(quat, float))
        while self.t and self.t[0] < t - self.horizon:
            self.t.pop(0); self.p.pop(0); self.q.pop(0)

    def at(self, t: float, max_extrapolation: float = 0.05):
        """(pos, quat) at t, or None if t is outside the buffered range."""
        if not self.t:
            return None
        if t >= self.t[-1]:
            return (self.p[-1], self.q[-1]) if t - self.t[-1] <= max_extrapolation else None
        if t < self.t[0]:
            return None
        i = bisect.bisect_right(self.t, t)
        t0, t1 = self.t[i - 1], self.t[i]
        f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return self.p[i - 1] + f * (self.p[i] - self.p[i - 1]), nlerp(self.q[i - 1], self.q[i], f)


def voxel_keys(points: np.ndarray, voxel: float) -> np.ndarray:
    """One int64 per voxel: 21 bits per axis, +-104 km at 10 cm voxels."""
    idx = np.floor(points / voxel).astype(np.int64) + (1 << 20)
    return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]


def voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    """Centroid of the points in each occupied voxel."""
    if len(points) == 0:
        return points.reshape(0, 3)
    _, inv, cnt = np.unique(voxel_keys(points, voxel), return_inverse=True, return_counts=True)
    out = np.zeros((len(cnt), 3))
    np.add.at(out, inv, points)
    return out / cnt[:, None]


class VoxelMap:
    """Accumulated map, one point per voxel (first point wins: keeps it cheap).

    Scans are queued by add() and merged in batches by consolidate(), because a
    merge sorts the whole map and should not run for every 10 Hz scan.
    """

    def __init__(self, voxel: float):
        self.voxel = voxel
        self.keys = np.empty(0, np.int64)
        self.points = np.empty((0, 3), np.float32)
        self._pending: list[np.ndarray] = []

    def add(self, points_world: np.ndarray):
        if len(points_world):
            self._pending.append(points_world.astype(np.float32))

    def consolidate(self) -> int:
        """Merge queued scans; returns the number of new voxels."""
        if not self._pending:
            return 0
        new = np.concatenate(self._pending)
        self._pending = []
        keys = np.concatenate([self.keys, voxel_keys(new.astype(np.float64), self.voxel)])
        pts = np.concatenate([self.points, new])
        before = len(self.keys)
        self.keys, first = np.unique(keys, return_index=True)    # existing voxels come first, so they win
        self.points = pts[first]
        return len(self.keys) - before

    def __len__(self):
        return len(self.keys)


def transform(points: np.ndarray, pos, quat) -> np.ndarray:
    return points @ quat_to_matrix(quat).T + np.asarray(pos, float)


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy])


def quat_mul(a, b) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def write_pcd(path: Path, xyz: np.ndarray, viewpoint=(0, 0, 0)):
    """Binary PCD (opens in CloudCompare, Open3D, PCL, rviz2 via pcl_ros)."""
    xyz = np.ascontiguousarray(xyz, np.float32)
    header = (f"# .PCD v0.7 - written by poc3_manual_scan\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
              f"TYPE F F F\nCOUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\n"
              f"VIEWPOINT {viewpoint[0]} {viewpoint[1]} {viewpoint[2]} 1 0 0 0\n"
              f"POINTS {len(xyz)}\nDATA binary\n")
    path = Path(path)
    tmp = path.with_suffix(".tmp")
    with open(tmp, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(xyz.tobytes())
    tmp.replace(path)                    # never leave a half-written map behind


def read_pcd(path: Path) -> np.ndarray:
    data = Path(path).read_bytes()
    end = data.index(b"DATA binary\n") + len(b"DATA binary\n")
    n = int([ln for ln in data[:end].decode().splitlines() if ln.startswith("POINTS")][0].split()[1])
    return np.frombuffer(data, np.float32, n * 3, end).reshape(n, 3)

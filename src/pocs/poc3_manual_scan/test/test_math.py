"""python3 -m pytest src/pocs/poc3_manual_scan/test -q   (no ROS needed)"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import flight_math as fm  # noqa: E402
import scan_map as sm  # noqa: E402


# ------------------------------------------------------------------ flight --

def test_shape_deadzone_and_endpoints():
    assert fm.shape(0.04, 0.05, 0.4) == 0.0
    assert fm.shape(1.0, 0.05, 0.4) == pytest.approx(1.0)
    assert fm.shape(-1.0, 0.05, 0.4) == pytest.approx(-1.0)
    assert fm.shape(0.0501, 0.05, 0.0) == pytest.approx(0.0001 / 0.95, abs=1e-9)   # no jump at the edge
    assert 0 < fm.shape(0.5, 0.05, 0.4) < fm.shape(0.5, 0.05, 0.0)                 # expo softens the centre


@pytest.mark.parametrize("heading,fwd,right,exp", [
    (0.0, 1, 0, (1, 0)),                 # facing north, forward = north
    (math.pi / 2, 1, 0, (0, 1)),         # facing east, forward = east
    (0.0, 0, 1, (0, 1)),                 # facing north, right = east
    (math.pi / 2, 0, 1, (-1, 0)),        # facing east, right = south
])
def test_body_to_ned(heading, fwd, right, exp):
    assert fm.body_to_ned(fwd, right, heading) == pytest.approx(exp, abs=1e-12)


def test_slew():
    assert fm.slew(0.0, 5.0, 0.2) == pytest.approx(0.2)
    assert fm.slew(1.0, 1.1, 0.2) == pytest.approx(1.1)
    assert fm.slew(0.0, -5.0, 0.2) == pytest.approx(-0.2)


def test_geofence_slows_towards_wall_never_away():
    g = fm.Geofence(-100, 100, -50, 50, 2, 120, gain=0.8)
    assert g.limit(0, 0, 50, 5, 5, 1) == (5, 5, 1)                     # far from walls: untouched
    vn, ve, vu = g.limit(97.5, 0, 50, 5, 0, 0)
    assert vn == pytest.approx(2.0)                                     # 2.5 m left -> 2 m/s
    assert g.limit(101, 0, 50, 5, 0, 0)[0] == 0.0                       # outside: no further out
    assert g.limit(101, 0, 50, -5, 0, 0)[0] == -5                       # ...but always back in
    assert g.limit(0, 0, 2.0, 0, 0, -1)[2] == 0.0                       # floor
    assert g.limit(0, 0, 0.0, 0, 0, 1)[2] == 1                          # climbing from the ground is fine
    assert g.limit(0, 0, 119, 0, 0, 3)[2] == pytest.approx(0.8)         # ceiling


def test_geofence_from_manifest_region():
    g = fm.Geofence.from_enu_region({"min": [-140.1, -262.3], "max": [459.9, 337.7]}, 10, 2, 120)
    assert (g.north_min, g.north_max, g.east_min, g.east_max) == pytest.approx((-252.3, 327.7, -130.1, 449.9))


# -------------------------------------------------------------------- scan --

def test_pose_buffer_interpolates():
    b = sm.PoseBuffer()
    q0, q90 = [0, 0, 0, 1], [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
    b.add(1.0, [0, 0, 0], q0)
    b.add(1.02, [0.2, 0, 0], q90)
    p, q = b.at(1.01)
    assert p == pytest.approx([0.1, 0, 0])
    yaw = 2 * math.atan2(q[2], q[3])
    assert math.degrees(yaw) == pytest.approx(45, abs=0.5)
    assert b.at(0.5) is None and b.at(1.2) is None
    assert b.at(1.03)[0] == pytest.approx([0.2, 0, 0])                  # short extrapolation allowed


def test_pose_buffer_resets_when_sim_time_jumps_back():
    b = sm.PoseBuffer()
    b.add(100.0, [0, 0, 0], [0, 0, 0, 1])
    b.add(1.0, [1, 0, 0], [0, 0, 0, 1])
    assert b.t == [1.0]


def test_transform_and_mount():
    q_yaw90 = sm.rpy_to_quat(0, 0, math.pi / 2)
    p = sm.transform(np.array([[1.0, 0, 0]]), [10, 0, 5], q_yaw90)
    assert p[0] == pytest.approx([10, 1, 5])
    # chaining: world <- base (yaw 90) <- lidar (0, 0, 0.13 up)
    q = sm.quat_mul(q_yaw90, [0, 0, 0, 1])
    pos = np.array([10, 0, 5]) + sm.quat_to_matrix(q_yaw90) @ [0, 0, 0.13]
    assert sm.transform(np.array([[1.0, 0, 0]]), pos, q)[0] == pytest.approx([10, 1, 5.13])


def test_voxel_map_dedups_and_keeps_first():
    m = sm.VoxelMap(0.5)
    m.add(np.array([[0.1, 0.1, 0.1], [0.2, 0.2, 0.2], [1.1, 0, 0]]))
    assert m.consolidate() == 2
    m.add(np.array([[0.3, 0.3, 0.3], [5, 5, 5], [-0.1, 0, 0]]))     # first is a known voxel; -0.1 is a new one
    assert m.consolidate() == 2
    assert len(m) == 4
    assert any(np.allclose(p, [0.1, 0.1, 0.1]) for p in m.points)


def test_voxel_downsample_is_centroid():
    pts = np.array([[0.0, 0, 0], [0.2, 0, 0], [3, 3, 3]])
    out = sm.voxel_downsample(pts, 1.0)
    assert len(out) == 2 and any(np.allclose(p, [0.1, 0, 0]) for p in out)


def test_pcd_roundtrip(tmp_path):
    xyz = np.random.default_rng(1).normal(size=(1000, 3)).astype(np.float32)
    sm.write_pcd(tmp_path / "m.pcd", xyz)
    assert np.array_equal(sm.read_pcd(tmp_path / "m.pcd"), xyz)

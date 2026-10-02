"""Unit tests for the bridge protocol and camera math (no ROS needed).

    python3 -m pytest src/pocs/poc2_unity_camera/test -q
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
import unity_bridge_protocol as p  # noqa: E402

VECTORS = json.loads((HERE.parents[3] / "docs" / "frame_test_vectors.json").read_text())


def test_pose_roundtrip():
    msg = p.pack_pose(12.5, [1, 2, 3], [0, 0, 0.7071, 0.7071])
    (t, payload), = p.StreamReader().feed(msg)
    assert t == p.POSE
    st, pos, q = p.unpack_pose(payload)
    assert st == 12.5 and np.allclose(pos, [1, 2, 3]) and np.allclose(q, [0, 0, 0.7071, 0.7071])


def test_frame_roundtrip_and_bottom_first_flip():
    rgb = np.zeros((4, 6, 3), np.uint8)
    rgb[0] = 255                                        # top row white
    axes = np.eye(3)
    msg = p.pack_frame(3.25, [1, 2, 3], axes[:, 0], axes[:, 1], axes[:, 2], rgb[::-1],
                       p.ENC_RGB8_BOTTOM_FIRST)          # sent the way Unity reads back
    (_, payload), = p.StreamReader().feed(msg)
    f = p.unpack_frame(payload)
    assert f.sim_time == 3.25 and f.rgb.shape == (4, 6, 3)
    assert (f.rgb[0] == 255).all() and (f.rgb[-1] == 0).all(), "top row must come out on top"
    assert np.allclose(f.axes, axes)


def test_stream_reassembly_split_anywhere():
    msgs = p.pack_pose(1, [0, 0, 0], [0, 0, 0, 1]) + p.pack_hello({"width": 2}) + \
        p.pack_pose(2, [1, 1, 1], [0, 0, 0, 1])
    for cut in range(1, len(msgs)):
        r = p.StreamReader()
        got = r.feed(msgs[:cut]) + r.feed(msgs[cut:])
        assert [t for t, _ in got] == [p.POSE, p.HELLO, p.POSE]


def test_bad_magic_is_rejected():
    with pytest.raises(ValueError):
        p.StreamReader().feed(b"XXXX" + bytes(8))


@pytest.mark.parametrize("case", VECTORS["gimbal_camera"],
                         ids=lambda c: f"rpy{c['drone_rpy_deg']}")
def test_gimbal_geometry_matches_reference(case):
    cam, axes = p.gimbal_camera(case["drone_pos_enu"], case["drone_quat_xyzw"],
                                case["mount_flu"], case["gimbal_pitch_deg"])
    assert np.allclose(cam, case["camera_pos_enu"], atol=1e-6)
    assert np.allclose(axes[:, 0], case["axis_right_enu"], atol=1e-6)
    assert np.allclose(axes[:, 1], case["axis_down_enu"], atol=1e-6)
    assert np.allclose(axes[:, 2], case["axis_forward_enu"], atol=1e-6)


@pytest.mark.parametrize("case", VECTORS["gimbal_camera"],
                         ids=lambda c: f"rpy{c['drone_rpy_deg']}")
def test_projection_matches_reference(case):
    cam = p.intrinsics(case["hfov_deg"], case["width"], case["height"])
    assert cam["fx"] == pytest.approx(case["fx"]) and cam["cx"] == case["cx"]
    axes = np.column_stack([case["axis_right_enu"], case["axis_down_enu"], case["axis_forward_enu"]])
    uv = p.project(case["world_points_enu"], case["camera_pos_enu"], axes, cam)
    assert np.allclose(uv, case["pixels_uv"], atol=VECTORS["tolerance"]["pixel"])


def test_quaternion_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.normal(size=4); q /= np.linalg.norm(q); q = q if q[3] >= 0 else -q
        assert np.allclose(p.matrix_to_quat(p.quat_to_matrix(q)), q, atol=1e-9)

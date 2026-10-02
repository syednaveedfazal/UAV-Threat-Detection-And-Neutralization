#!/usr/bin/env python3
"""Generate docs/frame_test_vectors.json - shared ground truth for frame math.

Both sides of the Unity bridge convert between ROS/Gazebo (ENU: x=East,
y=North, z=Up, right-handed) and Unity (x=East, y=Up, z=North, left-handed).
Getting that wrong gives a mirrored or rotated world that still "looks fine",
so the expected values are computed here, independently of either
implementation, with plain rotation matrices:

    S = swap(y, z)          ENU vector v  ->  Unity vector S v
    R_unity = S R_enu S     (S is its own inverse; det S = -1 flips handedness)

Sun positions come from pvlib (NREL SPA), a well-tested reference.

The C# EditMode tests (unity/com.guardian.sim/Tests) and the Python bridge
tests both read this file. Regenerate with:
    ~/.venvs/geo/bin/python scripts/make_frame_test_vectors.py
"""

import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
import pvlib
from scipy.spatial.transform import Rotation

S = np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0]], float)
OUT = Path(__file__).resolve().parent.parent / "docs" / "frame_test_vectors.json"


def unity_quat(r_enu: Rotation) -> list[float]:
    """ROS/ENU rotation -> Unity quaternion (x, y, z, w), canonical w >= 0."""
    m = S @ r_enu.as_matrix() @ S
    # m has det +1 (S . R . S with det S = -1 twice), so it is a valid rotation
    q = Rotation.from_matrix(m).as_quat()      # x, y, z, w
    return [round(float(v), 9) for v in (q if q[3] >= 0 else -q)]


def gimbal_camera_cases() -> list[dict]:
    """Expected geometry of the stabilised gimbal camera, from first principles.

    Definition (shared by Unity's GimbalCamera and the ROS bridge):
      * camera position = drone position + R_body . mount   (mount in body FLU)
      * stabilised gimbal: only the drone's heading carries over (roll/pitch
        ignored); the camera is tilted by `pitch_deg` (negative = looking down)
      * heading psi = direction of the body x-axis projected onto the ground
      * optical frame (ROS): x = image right, y = image down, z = forward
      * pinhole, square pixels: fx = fy = (w/2) / tan(hfov/2),
        cx = (w-1)/2, cy = (h-1)/2  (ROS convention: pixel centres at integers)
    """
    cases = []
    specs = [
        # drone pos ENU, rpy deg, gimbal pitch deg
        ([0.0, 0.0, 30.0], (0, 0, 0), -35.0),
        ([12.0, -7.5, 25.0], (0, 0, 90), -35.0),
        ([-40.0, 20.0, 18.0], (6, -4, 222.5), -50.0),     # tilted airframe: gimbal must ignore roll/pitch
        ([5.0, 5.0, 10.0], (-3, 8, -40.2), -20.0),
    ]
    mount = np.array([0.10, 0.0, -0.08])
    w, h, hfov = 960, 540, 82.0
    fx = (w / 2) / math.tan(math.radians(hfov) / 2)
    cx, cy = (w - 1) / 2, (h - 1) / 2
    for pos, rpy, pitch in specs:
        r = Rotation.from_euler("xyz", rpy, degrees=True)
        pos = np.array(pos)
        cam = pos + r.apply(mount)
        fwd_body = r.apply([1, 0, 0])
        psi = math.atan2(fwd_body[1], fwd_body[0])
        p = math.radians(pitch)
        forward = np.array([math.cos(psi) * math.cos(p), math.sin(psi) * math.cos(p), math.sin(p)])
        right = np.array([math.sin(psi), -math.cos(psi), 0.0])
        down = np.cross(forward, right)
        # world points: straight ahead on the ground, plus offsets left/right/near
        pts, pix = [], []
        ground_ahead = cam + forward * (cam[2] / -forward[2])            # optical axis hits z = 0
        for d in ([0, 0, 0], [4, 0, 0], [0, 3, 0], [-2, -5, 1.5], [3, 2, 17.0]):
            wp = ground_ahead + np.array(d, float)
            c = np.array([right @ (wp - cam), down @ (wp - cam), forward @ (wp - cam)])
            if c[2] <= 0.5:
                continue
            pts.append([round(float(v), 9) for v in wp])
            pix.append([round(float(fx * c[0] / c[2] + cx), 6), round(float(fx * c[1] / c[2] + cy), 6)])
        q = r.as_quat()
        cases.append({
            "drone_pos_enu": [float(v) for v in pos], "drone_rpy_deg": list(rpy),
            "drone_quat_xyzw": [round(float(v), 12) for v in q],
            "mount_flu": mount.tolist(), "gimbal_pitch_deg": pitch,
            "width": w, "height": h, "hfov_deg": hfov,
            "fx": round(fx, 9), "fy": round(fx, 9), "cx": cx, "cy": cy,
            "camera_pos_enu": [round(float(v), 9) for v in cam],
            "axis_right_enu": [round(float(v), 9) for v in right],
            "axis_down_enu": [round(float(v), 9) for v in down],
            "axis_forward_enu": [round(float(v), 9) for v in forward],
            "world_points_enu": pts, "pixels_uv": pix})
    return cases


def main():
    points = [
        {"enu": [1, 0, 0], "unity": [1, 0, 0], "note": "East stays +X"},
        {"enu": [0, 1, 0], "unity": [0, 0, 1], "note": "North becomes +Z"},
        {"enu": [0, 0, 1], "unity": [0, 1, 0], "note": "Up becomes +Y"},
        {"enu": [-12.5, 40.25, 3.0], "unity": [-12.5, 3.0, 40.25], "note": "general point"},
    ]

    # Yaw (CCW from East, ENU) -> Unity Euler Y (degrees, clockwise seen from above).
    #  * objects whose length axis is local +X (fence panels, gates):  y = -yaw
    #  * objects that face local +Z (people, cameras, vehicles):        y = 90 - yaw
    yaws = []
    for deg in (0, 30, 90, -40.2, 180, 270):
        yaw = math.radians(deg)
        yaws.append({"yaw_enu_deg": deg,
                     "unity_euler_y_x_aligned": round(-deg, 6),
                     "unity_euler_y_z_forward": round(90 - deg, 6),
                     "direction_unity": [round(math.cos(yaw), 9), 0.0,
                                         round(math.sin(yaw), 9)]})

    # Full rotations: ROS quaternion (x,y,z,w) -> Unity quaternion (x,y,z,w).
    # Closed form this should equal: (-qx, -qz, -qy, qw).
    rots = []
    for rpy in ((0, 0, 90), (0, 0, -40.2), (10, 0, 0), (0, 30, 0), (10, -20, 135), (-5, 12, 250)):
        r = Rotation.from_euler("xyz", rpy, degrees=True)
        q_ros = r.as_quat()
        rots.append({"ros_rpy_deg": list(rpy),
                     "ros_quat_xyzw": [round(float(v), 9) for v in q_ros],
                     "unity_quat_xyzw": unity_quat(r),
                     "ros_x_axis_in_unity": [round(float(v), 9)
                                            for v in S @ r.apply([1, 0, 0])]})

    # Sun: Bonn Poppelsdorf launch pad, several local times.
    lat, lon = 50.725890272, 7.086701902
    times = pd.DatetimeIndex(["2025-06-20T14:00:00", "2025-06-20T06:00:00",
                              "2025-12-21T12:00:00", "2025-03-20T17:30:00"],
                             tz="Europe/Berlin")
    sp = pvlib.solarposition.get_solarposition(times, lat, lon, altitude=60.85)
    sun = [{"local_time": t.strftime("%Y-%m-%dT%H:%M:%S"), "timezone": "Europe/Berlin",
            "utc": t.tz_convert("UTC").strftime("%Y-%m-%dT%H:%M:%SZ"),
            "lat": lat, "lon": lon,
            "elevation_deg": round(float(row.apparent_elevation), 3),
            "azimuth_deg_from_north_cw": round(float(row.azimuth), 3)}
           for t, row in sp.iterrows()]

    cameras = gimbal_camera_cases()

    OUT.write_text(json.dumps({
        "description": "Shared frame-conversion and sun-position test values. "
                       "ENU: x=East,y=North,z=Up (right-handed). Unity: x=East,y=Up,"
                       "z=North (left-handed). Generated by scripts/make_frame_test_vectors.py",
        "tolerance": {"position_m": 1e-6, "quaternion": 1e-6, "angle_deg": 1e-6,
                      "sun_deg": 1.0,    # NOAA shortcut is good to ~0.5 deg; shadows need ~1 deg
                      "pixel": 0.5},
        "points": points, "yaw": yaws, "rotations": rots, "sun": sun,
        "gimbal_camera": cameras}, indent=1))
    print(f"wrote {OUT}")
    for s in sun:
        print(f"  sun {s['local_time']}: elevation {s['elevation_deg']:.2f}, "
              f"azimuth {s['azimuth_deg_from_north_cw']:.2f}")
    # the closed form Unity code will use must agree with the matrix method
    for r in rots:
        x, y, z, w = r["ros_quat_xyzw"]
        closed = np.array([-x, -z, -y, w]); closed = closed if closed[3] >= 0 else -closed
        assert np.allclose(closed, r["unity_quat_xyzw"], atol=1e-8), r
    print("  closed form (-qx, -qz, -qy, qw) matches the matrix method")


if __name__ == "__main__":
    main()

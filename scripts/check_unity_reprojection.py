#!/usr/bin/env python3
"""Phase 3 gate: does the Unity image line up with its CameraInfo + TF?

Takes one /unity_cam/image_raw frame with the camera_info and the
gz_world -> unity_cam_optical TF of the *same stamp*, projects the compound's
fence panels (3D boxes from the scene manifest) into it with nothing but those
ROS messages, and measures how far the projection is off the fence pixels
Unity actually drew: the projected panel masks are shifted over +-12 px and
the shift where they cover the most fence-coloured pixels is the error.
A correct chain peaks at (0, 0).

This deliberately uses only what any ROS consumer sees - it does not trust
Unity's or the bridge's own geometry code.

    source /opt/ros/jazzy/setup.bash
    python3 scripts/check_unity_reprojection.py            # grab live + check
    python3 scripts/check_unity_reprojection.py --npz f.npz  # re-check a saved grab

Writes <out>.png (frame + projected outlines) and <out>.npz next to it.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

MANIFEST = Path.home() / "UAV/data/bonn_poppelsdorf/world/scene/scene_manifest.json"


def grab(timeout: float) -> dict:
    import rclpy
    from sensor_msgs.msg import CameraInfo
    from sensor_msgs.msg import Image as RosImage
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node("check_unity_reprojection")
    imgs, tfs, info = {}, {}, {}
    key = lambda h: (h.stamp.sec, h.stamp.nanosec)  # noqa: E731
    node.create_subscription(RosImage, "/unity_cam/image_raw", lambda m: imgs.__setitem__(key(m.header), m), 5)
    node.create_subscription(CameraInfo, "/unity_cam/camera_info", lambda m: info.__setitem__("k", m), 5)

    def on_tf(m):
        for t in m.transforms:
            if t.child_frame_id == "unity_cam_optical":
                tfs[key(t.header)] = t
    node.create_subscription(TFMessage, "/tf", on_tf, 50)
    end = node.get_clock().now().nanoseconds + timeout * 1e9
    common = set()
    while not (common and "k" in info):
        if node.get_clock().now().nanoseconds > end:
            raise SystemExit("no synchronised image + TF + camera_info (is Unity streaming?)")
        rclpy.spin_once(node, timeout_sec=0.1)
        common = set(imgs) & set(tfs)
    k = max(common)
    m, t = imgs[k], tfs[k].transform
    rgb = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3).copy()
    node.destroy_node()
    rclpy.shutdown()
    return {"rgb": rgb, "K": np.array(info["k"].k).reshape(3, 3), "stamp": k[0] + k[1] * 1e-9,
            "t": np.array([t.translation.x, t.translation.y, t.translation.z]),
            "q": np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])}


def quat_matrix(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def box_corners(part):
    """Manifest part: centre (ENU), size [length along yaw, width, height], yaw."""
    (cx, cy, cz), (L, W, H), yaw = part["centre"], part["size"], part["yaw"]
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for a in (-L / 2, L / 2):
        for b in (-W / 2, W / 2):
            for h in (-H / 2, H / 2):
                out.append([cx + a * c - b * s, cy + a * s + b * c, cz + h])
    return np.array(out)


def hull(pts):
    """Convex hull, monotone chain."""
    pts = sorted(map(tuple, pts))
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])  # noqa: E731
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--npz", help="re-check a saved grab instead of grabbing live")
    ap.add_argument("--manifest", default=str(MANIFEST))
    ap.add_argument("--out", default=str(Path.home() / "UAV/data/bonn_poppelsdorf/world/scene/unity_reprojection"))
    ap.add_argument("--timeout", type=float, default=20)
    a = ap.parse_args()

    d = dict(np.load(a.npz)) if a.npz else grab(a.timeout)
    if not a.npz:
        np.savez(a.out + ".npz", **d)
    rgb, K, t, R = d["rgb"], d["K"], d["t"], quat_matrix(d["q"])   # R: optical axes in world
    h, w, _ = rgb.shape
    parts = [p for p in json.loads(Path(a.manifest).read_text())["perimeter"] if p["type"] in ("fence", "gate")]

    mask = Image.new("L", (w, h), 0)
    md = ImageDraw.Draw(mask)
    polys = []
    for p in parts:
        c = (box_corners(p) - t) @ R                          # world -> optical
        if (c[:, 2] < 1.0).any():
            continue
        uv = np.column_stack([K[0, 0] * c[:, 0] / c[:, 2] + K[0, 2], K[1, 1] * c[:, 1] / c[:, 2] + K[1, 2]])
        if uv[:, 0].max() < 0 or uv[:, 0].min() >= w or uv[:, 1].max() < 0 or uv[:, 1].min() >= h:
            continue
        poly = [(x + 0.5, y + 0.5) for x, y in hull(uv)]      # ROS pixel centres -> PIL pixel edges
        md.polygon(poly, fill=255)
        polys.append(poly)
    m = np.asarray(mask) > 0
    if m.sum() < 200:
        raise SystemExit(f"only {m.sum()} fence pixels projected into view - point the drone at the compound fence")

    # Fence placeholder colour is dark green (0.18, 0.30, 0.22); lighting changes brightness, not the hue order.
    f = rgb.astype(int)
    fence_like = (f[..., 1] > f[..., 0] + 6) & (f[..., 1] > f[..., 2]) & (f.max(-1) < 170)

    R_ = 12
    pad = np.pad(fence_like, R_)
    score = np.zeros((2 * R_ + 1, 2 * R_ + 1))
    for dy in range(-R_, R_ + 1):
        for dx in range(-R_, R_ + 1):
            # image shifted by (-dx, -dy) under the mask == mask shifted by (dx, dy)
            score[dy + R_, dx + R_] = pad[R_ + dy:R_ + dy + h, R_ + dx:R_ + dx + w][m].mean()
    by, bx = np.unravel_index(score.argmax(), score.shape)
    best = (bx - R_, by - R_)
    s0, sb = score[R_, R_], score.max()

    over = Image.fromarray(rgb)
    od = ImageDraw.Draw(over)
    for poly in polys:
        od.polygon(poly, outline=(255, 40, 40))
    over.save(a.out + ".png")

    ok = max(abs(best[0]), abs(best[1])) <= 2
    print(f"stamp {d['stamp']:.3f}  panels in view {len(polys)}  projected fence px {int(m.sum())}")
    print(f"fence-coloured inside projection: {100 * s0:.1f}% at (0,0); best {100 * sb:.1f}% at shift {best} px "
          f"(background {100 * fence_like[~m].mean():.1f}%)")
    print(("PASS" if ok else "FAIL") + f": reprojection offset {best} px (gate: <= 2 px)  overlay -> {a.out}.png")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Stand-in for Unity: speaks the bridge protocol and renders a test pattern.

Lets the ROS side be tested without Unity. It uses the reference gimbal
geometry (unity_bridge_protocol.gimbal_camera) and ray-casts a 5 m checkerboard
on the plane z = ground, so a correct bridge shows a ground grid that moves
with the drone and a sky above the horizon.

    python3 fake_unity_client.py --rate 15
"""

import argparse
import socket
import time

import numpy as np

import unity_bridge_protocol as proto


def render(cam_pos, axes, cam: dict, ground_z: float) -> np.ndarray:
    """RGB image, top row first: sky above the horizon, checkerboard below."""
    w, h = cam['width'], cam['height']
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    rays = np.stack([(u - cam['cx']) / cam['fx'], (v - cam['cy']) / cam['fy'], np.ones_like(u, float)], -1)
    d = rays @ axes.T                                   # optical -> ENU directions
    img = np.empty((h, w, 3), np.uint8)
    img[:] = (135, 180, 230)
    hit = d[..., 2] < -1e-6
    t = (ground_z - cam_pos[2]) / d[..., 2][hit]
    x = cam_pos[0] + t * d[..., 0][hit]
    y = cam_pos[1] + t * d[..., 1][hit]
    white = (np.floor(x / 5).astype(int) + np.floor(y / 5).astype(int)) % 2 == 0
    img[hit] = np.where(white[:, None], (200, 200, 190), (60, 90, 50))
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5700)
    ap.add_argument('--rate', type=float, default=15.0, help='frames per second (wall clock)')
    ap.add_argument('--width', type=int, default=960)
    ap.add_argument('--height', type=int, default=540)
    ap.add_argument('--hfov', type=float, default=82.0)
    ap.add_argument('--pitch', type=float, default=-35.0, help='gimbal pitch, negative = down')
    ap.add_argument('--mount', type=float, nargs=3, default=[0.10, 0.0, -0.08], help='FLU, m')
    ap.add_argument('--ground', type=float, default=0.0, help='checkerboard plane height (ENU z)')
    ap.add_argument('--frames', type=int, default=0, help='stop after N frames (0 = run forever)')
    a, _ = ap.parse_known_args()      # ros2 launch appends --ros-args ...

    cam = proto.intrinsics(a.hfov, a.width, a.height)
    sent = 0
    while True:
        try:
            s = socket.create_connection((a.host, a.port), timeout=2.0)
        except OSError:
            print('bridge not reachable, retrying in 1 s'); time.sleep(1.0); continue
        s.settimeout(0.0)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.sendall(proto.pack_hello(cam, site='fake'))
        print(f'connected to {a.host}:{a.port}')
        reader, pose = proto.StreamReader(), None
        next_t = time.monotonic()
        try:
            while True:
                try:
                    while True:
                        data = s.recv(1 << 16)
                        if not data:
                            raise ConnectionError('bridge closed the connection')
                        for t, payload in reader.feed(data):
                            if t == proto.POSE:
                                pose = proto.unpack_pose(payload)
                except BlockingIOError:
                    pass
                now = time.monotonic()
                if pose is not None and now >= next_t:
                    next_t = max(next_t + 1.0 / a.rate, now)
                    sim_t, pos, q = pose
                    cam_pos, axes = proto.gimbal_camera(pos, q, a.mount, a.pitch)
                    rgb = render(cam_pos, axes, cam, a.ground)
                    s.setblocking(True)
                    s.sendall(proto.pack_frame(sim_t, cam_pos, *axes.T, rgb[::-1].copy(),
                                               proto.ENC_RGB8_BOTTOM_FIRST))   # like Unity readback
                    s.setblocking(False)
                    sent += 1
                    if a.frames and sent >= a.frames:
                        return
                time.sleep(0.002)
        except (OSError, ConnectionError) as e:
            print(f'{e}; reconnecting')
            s.close()
            time.sleep(1.0)


if __name__ == '__main__':
    main()

# Unity camera bridge (Phase 3)

Gazebo flies the drone; Unity renders what the drone's gimbal camera sees; ROS 2
gets the images. This file is the contract between the two sides.

```
Gazebo  --/model/<drone>/pose (gz.msgs.Pose, sim time)-->  ros_gz_bridge
        --/model/<drone>/pose (geometry_msgs/PoseStamped)-->  unity_bridge (rclpy)
unity_bridge  --TCP 127.0.0.1:5700-->  Unity (renders the gimbal camera at that pose)
Unity         --TCP-->  unity_bridge  -->  /unity_cam/image_raw, /unity_cam/camera_info, TF
```

Why a custom bridge: neither ros2-for-unity nor Unity's ROS-TCP-Endpoint supports
ROS 2 Jazzy on Ubuntu 24.04. A socket plus one rclpy node has no dependency to
wait on, and Isaac Sim (the later target) has native ROS 2 anyway.

Why a PosePublisher on the drone: the world's `dynamic_pose/info` has the same
pose, but `ros_gz_bridge` drops entity names and stamps when converting it
(verified: empty frame ids, stamp 0), so the drone could not be picked out
reliably.

## Transport

* TCP, `unity_bridge` listens on `127.0.0.1:5700`; Unity connects and retries
  every second, so either side can start first.
* One connection at a time; a new connection replaces the old one.
* All numbers little-endian. Every message:

| bytes | field |
|---|---|
| 4 | magic `GRD1` |
| 1 | type |
| 3 | reserved, zero |
| 4 | payload length (uint32) |
| n | payload |

## Messages

**1 HELLO** (Unity -> bridge, once per connection). UTF-8 JSON describing the
camera Unity actually renders; the bridge publishes `CameraInfo` from it:

```json
{"protocol": 1, "site": "bonn_poppelsdorf",
 "camera": {"width": 960, "height": 540, "fx": 552.1, "fy": 552.1,
            "cx": 479.5, "cy": 269.5, "hfov_deg": 82.0,
            "distortion_model": "plumb_bob", "d": [0, 0, 0, 0, 0]}}
```

`d` is the distortion *actually rendered* - zeros until lens distortion is added
in Unity (Phase 4). Publishing the target camera's coefficients while rendering
a pinhole image would mislead every consumer.

**2 POSE** (bridge -> Unity, ~50 Hz). 8 float64:
`sim_time, x, y, z, qx, qy, qz, qw` - the drone's world pose, ROS/ENU
convention, from Gazebo ground truth.

**3 FRAME** (Unity -> bridge, camera rate). Fixed part (116 bytes):

| type | field |
|---|---|
| float64 | sim_time of the pose this frame was rendered at |
| float64 x3 | camera position, ENU |
| float64 x9 | optical-frame axes in ENU: right (x), down (y), forward (z) |
| uint32 x3 | width, height, encoding (0 = RGB8 top row first, 1 = RGB8 bottom row first) |

followed by `width * height * 3` bytes. Unity sends encoding 1 (GPU readback
order); the bridge flips it.

Unity reports the camera pose it *rendered with*, rather than the bridge
recomputing it from the drone pose, so TF can never disagree with the image.

## Gimbal camera geometry

Shared definition, tested on both sides against `docs/frame_test_vectors.json`
(`gimbal_camera`), generated from first principles by
`scripts/make_frame_test_vectors.py`:

* camera position = drone position + R_body * mount (mount in body FLU, from the
  manifest's `camera.mount_xyz`)
* stabilised: only the drone's heading is kept; roll and pitch are ignored, the
  camera is tilted by `camera.gimbal.pitch_deg` (negative = down)
* pinhole, square pixels: fx = fy = (w/2)/tan(hfov/2), cx = (w-1)/2, cy = (h-1)/2
  (ROS: pixel centres at integer coordinates)

## ROS 2 output

| topic / frame | type |
|---|---|
| `/unity_cam/image_raw` | `sensor_msgs/Image`, `rgb8`, stamp = sim time of the pose |
| `/unity_cam/camera_info` | `sensor_msgs/CameraInfo`, same stamp |
| TF `gz_world -> unity_cam_optical` | camera pose Unity rendered with, same stamp |

Frames are not lockstep: Unity renders the newest pose it has. Stamping each
image with *that pose's* sim time keeps image, TF and labels consistent.

## Coordinates

ROS/Gazebo ENU (x East, y North, z Up, right-handed) <-> Unity (x East, y Up,
z North, left-handed). Converted only in `unity/com.guardian.sim/Runtime/Frames.cs`
and, on the ROS side, only in `unity_bridge_protocol.py`.

## Running it

```bash
# everything: Gazebo headless + PX4 + bridge (+ Unity player if built)
./start_uav_sim.sh --world ~/UAV/data/bonn_poppelsdorf/world/bonn_poppelsdorf.sdf \
                   --model gz_x500_threat_scanner --sensors --unity --mission

# or just the bridge next to a running sim
ros2 launch poc2_unity_camera unity_camera.launch.py             # clock:=false if sensors.launch.py runs
ros2 launch poc2_unity_camera unity_camera.launch.py fake:=true  # test pattern, no Unity needed
```

Unity side, either:

* **Editor:** open `unity/GuardianSim`, then *Guardian > Play Drone Camera (compound)*.
  The Main Camera chases the drone; the gimbal feed is drawn bottom-right with the
  bridge status. *Play* again to stop.
* **Player:** build once with
  `Unity -batchmode -quit -projectPath unity/GuardianSim -executeMethod Guardian.Sim.Editor.GuardianMenu.BuildPlayer`
  (-> `unity/GuardianSim/Build/GuardianSim.x86_64`); `--unity` starts it.
  Options: `--region compound|full --scene manifest.json --bridge-port 5700 --no-feed`.

| piece | file |
|---|---|
| protocol + camera math (ROS) | `src/pocs/poc2_unity_camera/src/unity_bridge_protocol.py` |
| bridge node | `src/pocs/poc2_unity_camera/src/unity_bridge.py` |
| fake Unity (test pattern) | `src/pocs/poc2_unity_camera/src/fake_unity_client.py` |
| TCP client (Unity) | `unity/com.guardian.sim/Runtime/BridgeClient.cs` |
| gimbal geometry (Unity) | `unity/com.guardian.sim/Runtime/GimbalMath.cs` |
| render + readback | `unity/com.guardian.sim/Runtime/GimbalCamera.cs` |
| drone, pose, sim clock | `unity/com.guardian.sim/Runtime/DroneRig.cs` |
| player setup | `unity/com.guardian.sim/Runtime/GuardianBootstrap.cs` |

Unity's people follow Gazebo's sim time: DroneRig makes the newest pose's sim
time the `SimClock`, so in every frame the drone, the people and the stamp are
the same instant.

## Verification

| check | how | result |
|---|---|---|
| protocol, gimbal geometry, projection (Python) | `python3 -m pytest src/pocs/poc2_unity_camera/test -q` | 13 passed |
| gimbal geometry + projection through a real Unity `Camera`, FRAME byte layout | EditMode `GimbalTests` | |
| ROS output conventions (flip, TF quaternion, K) | fake client, live sim: checker corners projected from TF + CameraInfo with scipy land on colour edges | 338/338; flipped + shifted control 7% |
| rate | raw subscriber on `/unity_cam/image_raw` (`ros2 topic hz` is too slow for 1.5 MB images in Python) | fake client 14.7 Hz; **Unity HDRP editor, Bonn compound, with Gazebo + PX4 + QGC: 15.2 Hz** (gate >= 10) |
| Unity image vs its TF + CameraInfo | `scripts/check_unity_reprojection.py` (fence panels from the manifest, best-shift search) | **0 px offset in 8/8 live frames** (hover, climb, transit, descent); ~93% of projected fence pixels are fence-coloured vs 6-14% background |

The frame-behind-newest-pose figure the bridge logs (Unity: ~80 ms mean, 140 ms max)
is render + readback + transfer; it does not make stamps wrong, because each
image carries its own pose's time.

# UAV-Thread-Detection-And-Neutralization
![alt text](image-1.png)


An advanced UAV which can auto detect the intruder with computer vision and lidar sensors to map  around the near by restricted area and give up the alaram for thread near by
![Map](world.png)

# Autonomous Drone Landing Mission POC

This project implements a simulated autonomous drone mission using **ROS 2 Humble**, **PX4 Autopilot**, and **Gazebo**.


## Project Architecture

*   **Simulation Environment:** Gazebo (Garden/Harmonic) via `ros_gz_sim`.
*   **Flight Control:** PX4 Autopilot running in SITL (Software In The Loop) mode.
*   **Communication Bridge:** Micro-XRCE-DDS-Agent bridging PX4 (uORB) and ROS 2 topics.
*   **Mission Logic:** A custom ROS 2 Python node (`mission_control.py`) that sends trajectory setpoints and high-level commands (Arm, Takeoff, Land).

## Prerequisites

Ensure you have the following installed on Ubuntu 22.04:
*   ROS 2 Humble
*   PX4 Autopilot Toolchain
*   Micro-XRCE-DDS-Agent
*   Gazebo (`ros-humble-ros-gz`)

## Installation

1.  **Clone the repository:**
    ```bash
    cd ~/drone_ws/src
    # Clone your repo here
    ```

2.  **Build the ROS 2 workspace:**
    ```bash
    cd ~/drone_ws
    colcon build --symlink-install
    source install/setup.bash
    ```

## Usage

### One command

```bash
./start_uav_sim.sh
```

This starts the whole stack and waits: Gazebo server + GUI running
`landing_mission.sdf`, PX4 SITL attached to that world with the x500 spawned,
MicroXRCEAgent on UDP 8888, and QGroundControl. Ctrl+C stops everything.

Useful flags:

| Flag | Effect |
| --- | --- |
| `--mission` | also run `mission_control.py` once PX4 is ready |
| `--headless` | no Gazebo GUI |
| `--no-qgc` | do not start QGroundControl |
| `--no-agent` | do not start MicroXRCEAgent |
| `--build` | `colcon build --symlink-install` first |
| `--world NAME` | use a different world from `worlds/` |

Logs for each process are written to `.sim_logs/`.

### Running the pieces by hand

PX4 is started in **standalone** mode so it attaches to a world this project
owns. PX4's generated `gz_env.sh` unconditionally overwrites `PX4_GZ_WORLDS`
to point inside the PX4 tree, so letting PX4 start Gazebo would require
copying this project's world into `PX4-Autopilot`.

```bash
# 1. Gazebo (server + GUI) with this project's world
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PWD/src/pocs/poc1_landing_world/models"
gz sim -r -s src/pocs/poc1_landing_world/worlds/landing_mission.sdf &
gz sim -g &

# 2. PX4 attaches to the running world and spawns the vehicle
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=landing_mission \
PX4_SIM_MODEL=gz_x500 PX4_SYS_AUTOSTART=4001 \
./build/px4_sitl_default/bin/px4 -d

# 3. ROS 2 bridge
MicroXRCEAgent udp4 -p 8888

# 4. Mission
ros2 run poc1_landing_world mission_control.py
```

`ros2 launch poc1_landing_world world.launch.py` performs step 1 with the same
environment if you prefer the ROS entry point.

## Real-world satellite terrain

Terrain comes from [gazebo_terrain_generator](https://github.com/saiaravind19/gazebo_terrain_generator),
which builds a Gazebo world from Mapbox elevation + satellite imagery and
optional OSM buildings. It targets modern Gazebo, so it drops straight into
this stack.

```bash
cd ~/UAV/gazebo_terrain_generator
uv sync
uv run scripts/server.py        # then open http://localhost:8080
```

Paste a Mapbox public token (`pk.eyJ1...`) under **Settings -> Mapbox API Key**,
draw a polygon, set the spawn marker, download the `.zip` and unzip it.

### Generated worlds need one fix before PX4 will fly in them

A generated world declares its own Gazebo systems, but only the ones needed to
render: physics, sensors, imu, navsat, scene-broadcaster, user-commands.
Because the world declares plugins inline, Gazebo uses *that* list and PX4's
own `server.config` is not applied - so the magnetometer and barometer systems
are simply absent and PX4 refuses to arm:

```
Preflight Fail: barometer 0 missing
Preflight Fail: Found 0 compass (required: 1)
Preflight Fail: heading estimate invalid
```

Patch the world once:

```bash
scripts/prepare_terrain_world.py /path/to/<name>/<name>.world
```

It injects the missing systems (idempotent, keeps a `.orig` backup) and prints
the terrain's real-world origin. Then fly in it:

```bash
./start_uav_sim.sh --world /path/to/<name>/<name>.world
```

`--world` accepts a bare name in `worlds/`, or any path to a `.sdf`/`.world`.
The launcher reads the `<world name>` from inside the file (it need not match
the filename) and picks up the terrain's `<spherical_coordinates>` to set
`PX4_HOME_LAT/LON/ALT`, so GPS, QGC's map and the terrain agree.

Two sample worlds ship with the generator and need **no Mapbox key**, which
makes them the quickest way to check the pipeline:
`sample_worlds/applepark` (Apple Park, with 3D buildings) and
`sample_worlds/Joshimath` (Himalayan terrain).

## LiDAR and the 3D point cloud

`models/x500_threat_scanner/` is the x500 plus:

- **16-beam 3D LiDAR** (`gpu_lidar`), 360 deg x 512 samples, 100 m range,
  10 Hz, vertical band biased downward (-25 to +15 deg) for mapping the ground
- **forward RGB camera**, 640x480 @ 15 Hz

LiDAR gives metric geometry regardless of lighting; the camera gives
appearance. Fuse them: cluster the cloud for position/size, classify with the
image. Neither alone answers "what is that and where".

```bash
./start_uav_sim.sh --model gz_x500_threat_scanner --sensors --mission
```

ROS 2 topics (via `sensors.launch.py`):

| Topic | Type |
| --- | --- |
| `/lidar_3d/points` | `sensor_msgs/PointCloud2` |
| `/lidar_3d` | `sensor_msgs/LaserScan` |
| `/front_cam/image` | `sensor_msgs/Image` |
| `/front_cam/camera_info` | `sensor_msgs/CameraInfo` |

Cloud density is `horizontal x vertical x rate` = 512 x 16 x 10 = 82k rays/s.
Raise `<samples>` in the model for a denser cloud once you are on the dGPU.

## Verifying it actually works

A successful build proves nothing; check the running system:

```bash
gz model --list                       # must list x500_0 alongside the buildings
ros2 topic list | grep /fmu           # ~65 topics via MicroXRCEAgent
ros2 topic echo --once --qos-reliability best_effort \
  --qos-durability volatile /fmu/out/vehicle_local_position_v1
```

Note that PX4 v1.16+ uses **versioned** topic names: `vehicle_status_v4`,
`vehicle_local_position_v1`. The unversioned names do not exist.

## Troubleshooting

**Gazebo opens but there is no drone.** PX4 was built without its Gazebo
bridge, almost always because it was built inside a container that has no
`gz-harmonic`. Check and rebuild on the host:

```bash
strings ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 | grep -cx gz_bridge   # 0 = broken
rm -rf ~/PX4-Autopilot/build/px4_sitl_default
cd ~/PX4-Autopilot && PATH=/usr/bin:$PATH make px4_sitl_default
```

See `docker/README.md` for the full explanation. `start_uav_sim.sh` refuses to
start if this check fails.

**Two Gazebo windows / drone in one and buildings in the other.** Something is
running both Gazebo Classic and modern Gazebo. This project is modern Gazebo
(Harmonic) only.

**QGroundControl exits immediately / `GLIBC_2.38' not found`.** Recent QGC
AppImages are built against glibc 2.38 and cannot run on Ubuntu 22.04, which
ships glibc 2.35. If you have two AppImages in `~/Downloads`, the newer one is
probably the broken one — delete it, or keep the older working build. The
launcher tries candidates newest-first, reports the crash, and falls back to
the next one; it also skips starting QGC entirely if an instance is already
running. Check your glibc with `ldd --version`.

**PX4 never accepts OFFBOARD.** PX4 requires a stream of
`OffboardControlMode` *and* `TrajectorySetpoint` messages *before* the mode
request; `mission_control.py` streams for ~1.5 s first.

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

**PX4 never accepts OFFBOARD.** PX4 requires a stream of
`OffboardControlMode` *and* `TrajectorySetpoint` messages *before* the mode
request; `mission_control.py` streams for ~1.5 s first.

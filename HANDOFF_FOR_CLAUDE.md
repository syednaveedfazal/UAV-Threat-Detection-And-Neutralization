# Handoff for Claude Code

## User goal

Build and run a simulated UAV threat-detection / neutralization project. The immediate practical goal is to get PX4 SITL, Gazebo, ROS 2, and QGroundControl working together so the drone visibly spawns and can be controlled. The longer-term goal is to implement an autonomous mission that can detect threats with computer vision/LiDAR and navigate/land in simulation.

The user wants a simple repeatable startup workflow, preferably one command or one launcher, but correctness comes before convenience.

## Workspace and environment

- Host OS: Ubuntu 22.04/Linux.
- Project repository: `/home/syed/UAV/UAV-Thread-Detection-And-Neutralization`
- PX4 source tree: `/home/syed/PX4-Autopilot`
- ROS 2: Humble, installed under `/opt/ros/humble`.
- QGroundControl: native host application.
- Main ROS package: `poc1_landing_world`.
- PX4 messages package: `src/px4_msgs`.
- The workspace has previously been built with `colcon build --symlink-install`.

## Project architecture

The intended architecture is:

1. PX4 Autopilot runs SITL.
2. Gazebo provides the simulated world and vehicle.
3. Micro-XRCE-DDS-Agent bridges PX4 uORB/DDS to ROS 2.
4. ROS 2 runs `mission_control.py` and sends PX4 offboard commands.
5. QGroundControl connects to PX4 over MAVLink UDP, normally port `14540`.

## Important simulator mismatch to resolve

The project ROS launch file is written for modern Gazebo (`gz sim`) through `ros_gz_sim` and uses `GZ_SIM_RESOURCE_PATH`.

PX4's currently inspected checkout contains both:

- `Tools/simulation/gz/` for modern Gazebo
- `Tools/simulation/gazebo-classic/` for Gazebo Classic

The current `start_uav_sim.sh` uses PX4's Gazebo Classic launcher:

```text
Tools/simulation/gazebo-classic/sitl_run.sh
```

but then separately starts the project's `world.launch.py`, which starts a second modern Gazebo instance. This is not a validated integration and can produce two unrelated simulations, port conflicts, or a world without a PX4-controlled vehicle. Do not claim this launcher works until it is tested end to end.

The preferred next step is to choose one simulator stack and make PX4 and the custom world use that same stack. Since `world.launch.py` already uses `ros_gz_sim`, modern Gazebo/PX4 `Tools/simulation/gz` is likely the cleaner direction. Alternatively, convert the project world to Gazebo Classic and use only the Classic PX4 launcher. Do not run both stacks as independent worlds unless deliberately required.

## Commands tried and confirmed findings

### PX4 Docker helper

This command was tried:

```bash
cd ~/PX4-Autopilot
./Tools/docker_run.sh make px4_sitl gazebo
```

Output included:

```text
PX4_DOCKER_REPO: px4io/px4-dev:v1.17.0-beta1
[docker-entrypoint.sh] Starting
[docker-entrypoint.sh] (x86_64)
ninja: no work to do.
```

After it returned, `docker ps` showed no running container. This is expected for a one-shot `docker run --rm` command: the container exits after the command completes. It did not prove that SITL or Gazebo was running.

An interactive Docker shell was then started:

```bash
cd ~/PX4-Autopilot
./Tools/docker_run.sh bash
```

Inside that container, this failed:

```bash
make px4_sitl gazebo
```

with:

```text
ninja: error: unknown target 'gazebo', did you mean 'geo'?
make: *** [Makefile:233: px4_sitl] Error 1
```

`make help` in this checkout exposed `px4_sitl`, `px4_sitl_default-clang`, and `px4_sitl_default-clang-test`, but not a `gazebo` make target. Therefore `make px4_sitl gazebo` is invalid for this checkout. Build/start commands must match the PX4 version's actual `Tools/simulation/gz` or `Tools/simulation/gazebo-classic` scripts.

### PX4 Gazebo Classic scripts

The following files exist in the PX4 checkout:

```text
/home/syed/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash
/home/syed/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh
/home/syed/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh
```

`sitl_run.sh` usage is:

```text
sitl_run.sh sitl_bin debugger model world src_path build_path
```

A previously suggested Classic command was:

```bash
cd ~/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash "$(pwd)" "$(pwd)/build/px4_sitl_default"
./Tools/simulation/gazebo-classic/sitl_run.sh \
  "$(pwd)/build/px4_sitl_default/bin/px4" \
  none \
  iris \
  none \
  "$(pwd)" \
  "$(pwd)/build/px4_sitl_default"
```

This has not been successfully validated in the current environment. It requires Gazebo Classic to be installed and a built PX4 SITL binary. Do not assume that the container image has the necessary Classic Gazebo GUI/runtime.

## Current project files

### `src/pocs/poc1_landing_world/launch/world.launch.py`

- Starts modern Gazebo with `ros_gz_sim` and `gz_sim.launch.py`.
- Sets `GZ_SIM_RESOURCE_PATH` to the package models and PX4 modern Gazebo models at:
  `~/PX4-Autopilot/Tools/simulation/gz/models`.
- Bridges `/clock` with `ros_gz_bridge`.
- Explicitly does not spawn the drone; the comment says PX4 should spawn it.
- The custom world is `worlds/landing_mission.world`.

This file was modified during earlier debugging for Gazebo resource paths. Its current working-tree status should be preserved unless a targeted integration fix requires changes.

### `src/pocs/poc1_landing_world/src/mission_control.py`

- ROS 2 Python node named `mission_control`.
- Publishes:
  - `/fmu/in/offboard_control_mode`
  - `/fmu/in/trajectory_setpoint`
  - `/fmu/in/vehicle_command`
- Subscribes to:
  - `/fmu/out/vehicle_status`
  - `/fmu/out/vehicle_local_position`
- Uses PX4 message QoS with best effort, transient local, depth 1.
- Sends arm, offboard, trajectory, and land commands.
- Waypoints are NED coordinates:
  - `[0.0, 0.0, -10.0]`
  - `[5.0, 5.0, -10.0]`
  - `[-4.0, 2.0, -10.0]`
  - `[0.0, 0.0, -10.0]`
  - `[0.0, 0.0, 0.0]`

Important code risk: the node attempts to engage offboard and arm before demonstrating that enough setpoints have been published. PX4 normally requires a stream of offboard setpoints before accepting OFFBOARD mode. Validate and fix this only after the basic simulator/ROS bridge path works.

### `src/pocs/poc1_landing_world/package.xml`

Declares `ament_cmake`, `rclcpp`, `gazebo_ros`, `rclpy`, `px4_msgs`, and `geometry_msgs`. It currently mixes a `gazebo_ros` dependency with a launch file using `ros_gz_sim`; dependency cleanup may be needed after choosing the simulator stack.

### `docker/Dockerfile`

- Based on Ubuntu 22.04.
- Installs ROS 2 Humble desktop, colcon, `gz-harmonic`, `ros-humble-ros-gz`, and `ros-humble-ros-gz-bridge`.
- Does not currently install/build `MicroXRCEAgent`.
- Does not provide PX4 SITL itself; PX4 is mounted from the host by compose.
- Default command is just `bash`.

The Docker image build previously succeeded, but a complete running simulation through Docker was not verified.

### `docker/docker-compose.yml`

- Defines service `sim`, container name `uav_sim`.
- Uses host networking.
- Mounts the project at `/workspace` and PX4 at `/px4`.
- Mounts X11 socket for GUI.
- Runs idle with `tail -f /dev/null` and expects manual `docker exec` steps.
- It is not currently a one-command working simulation launcher.

### `start_uav_sim.sh`

Current contents launch three host terminals:

1. PX4 Gazebo Classic via `Tools/simulation/gazebo-classic/sitl_run.sh`.
2. ROS package `world.launch.py`, which starts a separate modern Gazebo instance.
3. `qgroundcontrol`.

It has been executed once and returned exit code 0, but this does not prove the three processes remained alive or that the drone spawned. It also does not start `MicroXRCEAgent`.

The script needs to be redesigned after choosing one Gazebo stack. It should check prerequisites, build/source the correct workspaces, start the agent, wait for PX4 readiness, launch the ROS world/vehicle exactly once, and clean up child processes on Ctrl+C.

## Git cleanup already performed

The repository previously showed more than 10,000 generated-file changes because ROS build outputs and backup build trees were visible to Git.

`.gitignore` was expanded to ignore:

- `/build/`, `/install/`, `/log/`
- nested `**/build/`, `**/install/`, `**/log/`
- `build_backup_*`, `install_backup_*`, `log_backup_*`
- `.vscode/`, `.idea/`, swap files
- Python caches and OS files

Generated directories were removed from the Git index with `git rm -r --cached`; files on disk were not deleted by that command.

Current status captured during handoff:

```text
A  .github/agents/uav-research-build-planner.agent.md
 M .gitignore
 M src/px4_msgs
?? docker/Dockerfile
?? docker/README.md
?? docker/docker-compose.yml
?? start_uav_sim.sh
```

`src/px4_msgs` is a modified Git submodule or nested repository entry. Inspect it before changing or committing it. Do not discard user changes.

The generated `build`, `install`, `log`, and backup folders still exist on disk but are ignored. The PDF and image assets at the repository root are intentional project/reference assets unless the user says otherwise.

## Known successful/unsuccessful items

Successful or observed:

- ROS workspace built successfully at least once with `colcon build --symlink-install`.
- `px4_msgs` built successfully at least once.
- Custom Docker image build succeeded.
- Gazebo model/resource path and malformed material-script issues were investigated.
- Git generated-file noise was reduced by ignore rules and index cleanup.

Not yet proven:

- A live PX4 SITL process with a visible drone.
- A live Gazebo world containing the PX4 vehicle.
- MAVLink traffic on UDP `14540`.
- QGroundControl connection to the vehicle.
- MicroXRCEAgent running and ROS PX4 topics flowing.
- `mission_control.py` successfully arming, entering OFFBOARD, flying, and landing.
- Docker one-command startup.

## Recommended next work order

1. Choose modern Gazebo or Gazebo Classic. Prefer one stack only.
2. Run the smallest possible PX4 SITL + matching Gazebo example from the PX4 checkout, without the custom ROS world.
3. Verify the PX4 process stays alive and the vehicle is visible.
4. Verify MAVLink UDP traffic, especially port `14540`, and connect QGroundControl.
5. Start `MicroXRCEAgent udp4 -p 8888` and verify ROS 2 PX4 topics with `ros2 topic list` and `ros2 topic echo`.
6. Launch the custom world using the same Gazebo stack as PX4 and confirm the vehicle appears in that world.
7. Build/source the ROS workspace and run `mission_control.py` only after the bridge and vehicle are confirmed.
8. Fix offboard sequencing and add focused runtime checks/tests.
9. Only then automate the sequence in Docker Compose or `start_uav_sim.sh`.

## Useful validation commands

From the project root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix poc1_landing_world
```

For PX4 target discovery:

```bash
cd ~/PX4-Autopilot
make help | grep -iE 'gazebo|sitl|gz'
find Tools/simulation -maxdepth 2 -type f | sort
```

For processes and MAVLink:

```bash
pgrep -a px4
docker ps
sudo ss -lunp | grep -E '14540|14550|14556|8888'
sudo tcpdump -n -i any udp port 14540 -c 20
```

For ROS bridge checks:

```bash
MicroXRCEAgent udp4 -p 8888
ros2 topic list | grep -E 'fmu|clock'
ros2 topic echo /fmu/out/vehicle_status
```

Do not use `sudo colcon build`; previous root-owned build artifacts caused permission errors. If a clean rebuild is necessary, remove only ignored generated directories as the normal user, then rebuild.

## Critical handoff instruction

Do not report that the simulation works based only on a successful build or a command returning to the shell. A valid completion requires fresh evidence that:

- PX4 remains running,
- Gazebo remains running,
- a drone is visible/spawned,
- MAVLink traffic reaches QGroundControl,
- and, for ROS control, PX4 topics are present through MicroXRCEAgent.

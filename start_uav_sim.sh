#!/usr/bin/env bash
#
# One-command launcher for the UAV threat-detection simulation.
#
# Stack (single simulator, modern Gazebo / Harmonic only):
#   1. gz sim server  - runs worlds/landing_mission.sdf
#   2. gz sim gui     - viewer (skip with --headless)
#   3. PX4 SITL       - standalone mode; attaches to the running world and
#                       spawns the x500, then bridges it via gz_bridge
#   4. MicroXRCEAgent - PX4 uORB <-> ROS 2 bridge on UDP 8888
#   5. QGroundControl - MAVLink GCS on UDP 14550 / 14540
#
# PX4 is deliberately started with PX4_GZ_STANDALONE=1. PX4's own gz_env.sh
# hard-overrides PX4_GZ_WORLDS to point inside the PX4 tree, so letting PX4
# start Gazebo makes it impossible to use this project's world without
# copying files into PX4-Autopilot. Starting the server here and having PX4
# attach keeps the world in this repository.
#
set -euo pipefail

# ---------------------------------------------------------------- settings --
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/humble/setup.bash}"
PKG_DIR="$PROJECT_DIR/src/pocs/poc1_landing_world"

WORLD="${WORLD:-landing_mission}"
MODEL="${MODEL:-gz_x500}"
AUTOSTART="${AUTOSTART:-4001}"          # 4001 = gz_x500 airframe
SPAWN_POSE="${SPAWN_POSE:-0,0,0.2,0,0,0}"

HEADLESS=0
START_QGC=1
START_AGENT=1
START_MISSION=0
DO_BUILD=0

QGC_BIN="${QGC_BIN:-}"

LOG_DIR="$PROJECT_DIR/.sim_logs"

# ------------------------------------------------------------------- usage --
usage() {
	cat <<EOF
Usage: $(basename "$0") [options]

  --headless      Do not start the Gazebo GUI
  --no-qgc        Do not start QGroundControl
  --no-agent      Do not start MicroXRCEAgent
  --mission       Also run the ROS 2 mission_control node once PX4 is ready
  --build         colcon build the ROS workspace before starting
  --world NAME    World basename in $PKG_DIR/worlds (default: $WORLD)
  -h, --help      Show this help

Logs are written to $LOG_DIR
EOF
}

while [[ $# -gt 0 ]]; do
	case "$1" in
		--headless)  HEADLESS=1 ;;
		--no-qgc)    START_QGC=0 ;;
		--no-agent)  START_AGENT=0 ;;
		--mission)   START_MISSION=1 ;;
		--build)     DO_BUILD=1 ;;
		--world)     WORLD="$2"; shift ;;
		-h|--help)   usage; exit 0 ;;
		*) echo "Unknown option: $1" >&2; usage; exit 1 ;;
	esac
	shift
done

# ------------------------------------------------------------------ helpers --
PIDS=()

log()  { printf '\033[1;34m[sim]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[sim]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[sim] %s\033[0m\n' "$*" >&2; exit 1; }

track() { PIDS+=("$1"); }

# Kill a process and everything it spawned, deepest first. `ros2 run` and the
# Gazebo launchers fork children that do not die with their parent.
kill_tree() {
	local pid="$1" sig="$2" child
	for child in $(pgrep -P "$pid" 2>/dev/null || true); do
		kill_tree "$child" "$sig"
	done
	kill -"$sig" "$pid" 2>/dev/null || true
}

cleanup() {
	local ec=$?
	set +e
	# Disarm the traps: without this the EXIT trap re-runs cleanup after the
	# INT/TERM handler calls exit, and the shutdown is reported twice.
	trap - INT TERM EXIT
	echo
	log "Shutting down..."
	local pid
	for pid in "${PIDS[@]:-}"; do
		[[ -n "$pid" ]] && kill_tree "$pid" TERM
	done
	sleep 3
	for pid in "${PIDS[@]:-}"; do
		[[ -n "$pid" ]] && kill_tree "$pid" KILL
	done
	log "Done."
	exit $ec
}
trap cleanup INT TERM EXIT

# NOTE: do not wrap this in `setsid` - setsid forks, so $! would be the PID of
# the (immediately exiting) setsid process rather than the real child, and
# cleanup would then kill nothing.
spawn() {
	local name="$1"; shift
	"$@" > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
	local pid=$!
	track "$pid"
	log "started $name (pid $pid) -> $LOG_DIR/$name.log"
}

# wait_for <description> <timeout-seconds> <command...>
wait_for() {
	local what="$1" timeout="$2"; shift 2
	local waited=0
	while ! "$@" >/dev/null 2>&1; do
		sleep 1
		waited=$((waited + 1))
		if (( waited >= timeout )); then
			die "Timed out after ${timeout}s waiting for: $what"
		fi
	done
	log "ready: $what"
}

# ----------------------------------------------------------- prerequisites --
log "Checking prerequisites..."

[[ -d "$PX4_DIR" ]] || die "PX4 source not found at $PX4_DIR (set PX4_DIR)"
[[ -x "$PX4_BUILD/bin/px4" ]] || die \
	"PX4 SITL binary missing. Build it on the HOST (not in Docker):
    cd $PX4_DIR && make px4_sitl_default"
[[ -f "$PX4_BUILD/rootfs/gz_env.sh" ]] || die \
	"$PX4_BUILD/rootfs/gz_env.sh missing - PX4 was built without Gazebo support."

# A PX4 built without the Gazebo bridge silently produces a world with no
# vehicle, which is the failure this project hit before. Check explicitly.
# Note: use grep -c, not grep -q. Under `set -o pipefail`, grep -q closes the
# pipe early and `strings` dies with SIGPIPE, which would fail this check even
# on a good binary.
GZ_BRIDGE_SYMS="$(strings "$PX4_BUILD/bin/px4" | grep -cx "gz_bridge" || true)"
if [[ "${GZ_BRIDGE_SYMS:-0}" -eq 0 ]]; then
	die "PX4 binary has no gz_bridge module. It was almost certainly built in
a container without gz-harmonic installed. Rebuild on the host:
    rm -rf $PX4_BUILD && cd $PX4_DIR && make px4_sitl_default"
fi

command -v gz >/dev/null || die "'gz' not found - install gz-harmonic"
GZ_VER="$(gz sim --versions 2>/dev/null | tr -d ' ' | sed -n '1p' || true)"
[[ -n "$GZ_VER" ]] || die "'gz sim' not working"
log "Gazebo $GZ_VER"

[[ -f "$PKG_DIR/worlds/$WORLD.sdf" ]] || die "World not found: $PKG_DIR/worlds/$WORLD.sdf"

# The world name inside the SDF must equal the file stem: PX4 waits on the
# service /world/<PX4_GZ_WORLD>/scene/info.
if ! grep -q "<world name=\"$WORLD\">" "$PKG_DIR/worlds/$WORLD.sdf"; then
	die "worlds/$WORLD.sdf must contain <world name=\"$WORLD\"> for PX4 to find it."
fi

if (( START_AGENT )) && ! command -v MicroXRCEAgent >/dev/null; then
	warn "MicroXRCEAgent not found; ROS 2 PX4 topics will not appear (--no-agent to silence)"
	START_AGENT=0
fi

if (( START_QGC )); then
	if [[ -z "$QGC_BIN" ]]; then
		QGC_BIN="$(command -v qgroundcontrol 2>/dev/null || true)"
	fi
	if [[ -z "$QGC_BIN" ]]; then
		# Fall back to an AppImage in the usual download locations.
		QGC_BIN="$(ls -t "$HOME"/Downloads/QGroundControl*.AppImage \
		               "$HOME"/QGroundControl*.AppImage 2>/dev/null | head -n1 || true)"
	fi
	if [[ -z "$QGC_BIN" || ! -e "$QGC_BIN" ]]; then
		warn "QGroundControl not found; skipping (--no-qgc to silence)"
		START_QGC=0
	else
		[[ -x "$QGC_BIN" ]] || chmod +x "$QGC_BIN" 2>/dev/null || true
		log "QGroundControl: $QGC_BIN"
	fi
fi

# Refuse to stack a second simulation on top of a running one.
if pgrep -f 'bin/px[4]' >/dev/null || pgrep -f 'gz[ ]sim' >/dev/null; then
	die "PX4 or Gazebo is already running. Stop it first:
    pkill -f 'bin/px[4]'; pkill -f 'gz[ ]sim'"
fi

mkdir -p "$LOG_DIR"

# ------------------------------------------------------------- ROS 2 setup --
# ROS 2 setup scripts reference unset variables, so `set -u` must be relaxed
# while they are sourced.
source_relaxed() {
	set +u
	# shellcheck disable=SC1090
	source "$1"
	set -u
}

if [[ -f "$ROS_DISTRO_SETUP" ]]; then
	source_relaxed "$ROS_DISTRO_SETUP"
	log "ROS 2 sourced: $ROS_DISTRO_SETUP"
else
	warn "ROS 2 setup not found at $ROS_DISTRO_SETUP"
fi

if (( DO_BUILD )); then
	log "Building ROS workspace..."
	( cd "$PROJECT_DIR" && colcon build --symlink-install ) \
		|| die "colcon build failed"
fi

if [[ -f "$PROJECT_DIR/install/setup.bash" ]]; then
	source_relaxed "$PROJECT_DIR/install/setup.bash"
	log "Workspace sourced"
else
	warn "No install/setup.bash - run with --build to build the workspace"
fi

# --------------------------------------------------------- Gazebo environment --
# gz_env.sh provides PX4_GZ_MODELS/WORLDS/PLUGINS and the server config that
# loads the physics, sensors, IMU, magnetometer and NavSat systems PX4 needs.
source_relaxed "$PX4_BUILD/rootfs/gz_env.sh"
# Append this project's models so the world's landing pads resolve.
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PKG_DIR/models:$PKG_DIR/worlds"
export GZ_IP=127.0.0.1

# ------------------------------------------------------------------ launch --
log "World: $PKG_DIR/worlds/$WORLD.sdf"

spawn gz-server gz sim --verbose=1 -r -s "$PKG_DIR/worlds/$WORLD.sdf"
wait_for "Gazebo world '$WORLD'" 60 \
	bash -c "gz service -i --service /world/$WORLD/scene/info 2>&1 | grep -q 'Service providers'"

if (( ! HEADLESS )); then
	spawn gz-gui gz sim -g
fi

if (( START_AGENT )); then
	spawn microxrce MicroXRCEAgent udp4 -p 8888
fi

log "Starting PX4 SITL (standalone, attaching to '$WORLD')..."
cd "$PX4_DIR"
spawn px4 env \
	PX4_GZ_STANDALONE=1 \
	PX4_GZ_WORLD="$WORLD" \
	PX4_SIM_MODEL="$MODEL" \
	PX4_SYS_AUTOSTART="$AUTOSTART" \
	PX4_GZ_MODEL_POSE="$SPAWN_POSE" \
	"$PX4_BUILD/bin/px4" -d
cd "$PROJECT_DIR"

wait_for "PX4 vehicle spawned in Gazebo" 90 \
	bash -c "gz model --list 2>/dev/null | grep -q '${MODEL#gz_}_0'"
wait_for "PX4 ready for takeoff" 90 \
	bash -c "grep -aq 'Ready for takeoff' '$LOG_DIR/px4.log'"

if (( START_QGC )); then
	spawn qgc "$QGC_BIN"
fi

if (( START_MISSION )); then
	log "Waiting for PX4 ROS 2 topics before starting the mission..."
	wait_for "ROS 2 topic /fmu/out/vehicle_status_v4" 60 \
		bash -c "ros2 topic list 2>/dev/null | grep -q '/fmu/out/vehicle_status_v4'"
	spawn mission ros2 run poc1_landing_world mission_control.py
fi

# ----------------------------------------------------------------- summary --
cat <<EOF

  Simulation is up.

    Gazebo world : $WORLD   (server$( ((HEADLESS)) && echo ", headless" || echo " + GUI"))
    Vehicle      : ${MODEL#gz_}_0
    MAVLink      : udp 14550 (GCS)   udp 14540 (onboard/offboard)
    uXRCE-DDS    : udp 8888  $( ((START_AGENT)) || echo "(agent not started)" )
    Logs         : $LOG_DIR

  Useful checks:
    gz model --list
    ros2 topic list | grep /fmu
    ros2 topic echo /fmu/out/vehicle_local_position_v1

  Press Ctrl+C to stop everything.

EOF

# Keep the script in the foreground so the cleanup trap owns the whole stack.
wait

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
# ROS distro is auto-detected so the same script works on Jazzy (Ubuntu 24.04)
# and Humble (22.04). Newest first: Jazzy is the distro officially paired with
# Gazebo Harmonic, so prefer it when both are installed.
detect_ros_setup() {
	local d
	if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
		echo "/opt/ros/$ROS_DISTRO/setup.bash"; return
	fi
	for d in kilted jazzy iron humble; do
		[[ -f "/opt/ros/$d/setup.bash" ]] && { echo "/opt/ros/$d/setup.bash"; return; }
	done
}
ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-$(detect_ros_setup)}"
PKG_DIR="$PROJECT_DIR/src/pocs/poc1_landing_world"

WORLD="${WORLD:-landing_mission}"
MODEL="${MODEL:-gz_x500}"
AUTOSTART="${AUTOSTART:-4001}"          # 4001 = gz_x500 airframe
SPAWN_POSE="${SPAWN_POSE:-0,0,0.2,0,0,0}"

HEADLESS=0
START_QGC=1
START_AGENT=1
START_MISSION=0
START_SENSORS=0
START_UNITY=0
START_JOY=0
GZ_GUI_FORCED=0
DO_BUILD=0

QGC_BIN="${QGC_BIN:-}"
# Unity player built by GuardianMenu.BuildPlayer; region compound (fast) or full
UNITY_PLAYER="${UNITY_PLAYER:-$PROJECT_DIR/unity/GuardianSim/Build/GuardianSim.x86_64}"
UNITY_REGION="${UNITY_REGION:-compound}"

LOG_DIR="$PROJECT_DIR/.sim_logs"

# ------------------------------------------------------------------- usage --
usage() {
	cat <<EOF
Usage: $(basename "$0") [options]

  --headless      Do not start the Gazebo GUI
  --no-qgc        Do not start QGroundControl
  --no-agent      Do not start MicroXRCEAgent
  --mission       Also run the ROS 2 mission_control node once PX4 is ready
  --model NAME    Vehicle model, e.g. gz_x500 (default) or
                  gz_x500_threat_scanner (3D LiDAR + forward camera)
  --sensors       Bridge LiDAR / camera topics into ROS 2
  --unity         Photoreal gimbal camera from Unity (docs/UNITY_BRIDGE.md):
                  starts the bridge and the Unity player if built, else use the
                  editor (Guardian > Play Drone Camera). Implies --headless.
                  Needs --model gz_x500_threat_scanner (it publishes the pose).
  --gz-gui        Keep the Gazebo GUI with --unity
  --joystick      Fly with a game controller and scan with the LiDAR
                  (poc3_manual_scan: START take off, B land, Y record, X save).
                  Implies --sensors; not together with --mission.
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
		--model)     MODEL="$2"; shift ;;
		--sensors)   START_SENSORS=1 ;;
		--unity)     START_UNITY=1 ;;
		--gz-gui)    GZ_GUI_FORCED=1 ;;
		--joystick)  START_JOY=1 ;;
		-h|--help)   usage; exit 0 ;;
		*) echo "Unknown option: $1" >&2; usage; exit 1 ;;
	esac
	shift
done
# Unity is the viewer; the Gazebo GUI would only compete for the 6 GB GPU.
(( START_UNITY && ! GZ_GUI_FORCED )) && HEADLESS=1
# The scanner needs the LiDAR; and two nodes must never command PX4 at once.
(( START_JOY )) && START_SENSORS=1
(( START_JOY && START_MISSION )) && { echo '--joystick and --mission both fly the drone; pick one' >&2; exit 1; }

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
	SPAWN_PID=$!
	track "$SPAWN_PID"
	log "started $name (pid $SPAWN_PID) -> $LOG_DIR/$name.log"
}

# Did a just-spawned process survive its first few seconds? A crash-on-startup
# (a missing library, an incompatible AppImage) otherwise goes unnoticed and
# the script cheerfully reports "Simulation is up".
alive_after() {
	local pid="$1" seconds="$2"
	local i=0
	while (( i < seconds )); do
		kill -0 "$pid" 2>/dev/null || return 1
		sleep 1
		i=$((i + 1))
	done
	kill -0 "$pid" 2>/dev/null
}

# Print the most useful lines from a crashed process's log.
report_crash() {
	local name="$1"
	warn "$name exited immediately. Last lines of $LOG_DIR/$name.log:"
	tail -n 4 "$LOG_DIR/$name.log" 2>/dev/null | sed 's/^/    /' || true
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

# Run a Gazebo/PX4 process with ROS library paths stripped (see ROS setup below).
gzenv() {
	env LD_LIBRARY_PATH="${GZ_ONLY_LD-${LD_LIBRARY_PATH:-}}" GZ_CONFIG_PATH=/usr/share/gz "$@"
}

# Like wait_for, but warns instead of aborting.
wait_for_soft() {
	local what="$1" timeout="$2"; shift 2
	local waited=0
	while ! "$@" >/dev/null 2>&1; do
		sleep 1
		waited=$((waited + 1))
		if (( waited >= timeout )); then
			warn "not reached within ${timeout}s: $what"
			return 1
		fi
	done
	log "ready: $what"
}

# ----------------------------------------------------------- prerequisites --
log "Checking prerequisites..."

# A second stack cannot start next to a running one: the agent's UDP 8888 and
# PX4's MAVLink ports are taken, and a second PX4 would attach to the old world.
# Say so up front instead of failing half-way with "bind error errno 98".
# Matched on the executable (not anywhere in the command line), so a shell or
# editor that merely mentions these names is not mistaken for a simulation.
leftover=$(ps -eo pid=,args= | awk '{
	exe = $2; sub(/.*\//, "", exe); script = $3; sub(/.*\//, "", script)
	if ((exe == "gz" && $3 == "sim" && / -s /) || exe == "px4" || exe == "MicroXRCEAgent" ||
	    (exe ~ /^python3?$/ && script == "unity_bridge.py")) print
}')
if [[ -n "$leftover" ]]; then
	warn "A simulation is already running:"
	while IFS= read -r line; do warn "    ${line:0:110}"; done <<< "$leftover"
	die "Stop it first (Ctrl+C in its terminal, or: kill $(echo "$leftover" | awk '{print $1}' | tr '\n' ' '))"
fi

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

# Resolve the world file. Accept a bare name in the package worlds/ directory
# (.sdf or .world - gazebo_terrain_generator emits .world), or a path to a
# world anywhere on disk.
WORLD_FILE=""
for cand in \
	"$WORLD" \
	"$PKG_DIR/worlds/$WORLD.sdf" \
	"$PKG_DIR/worlds/$WORLD.world" \
	"$PKG_DIR/worlds/$WORLD"
do
	if [[ -f "$cand" ]]; then WORLD_FILE="$(readlink -f "$cand")"; break; fi
done
[[ -n "$WORLD_FILE" ]] || die "World not found: tried '$WORLD', \
'$PKG_DIR/worlds/$WORLD.sdf' and '$PKG_DIR/worlds/$WORLD.world'"

# PX4 waits on /world/<name>/scene/info, where <name> is the name declared
# INSIDE the SDF - not the filename. Read it rather than assuming they match.
WORLD_NAME="$(grep -om1 '<world name="[^"]*"' "$WORLD_FILE" | sed 's/.*name="//; s/"//')"
[[ -n "$WORLD_NAME" ]] || die "No <world name=\"...\"> element in $WORLD_FILE"
if [[ "$WORLD_NAME" != "$WORLD" ]]; then
	log "World file declares name '$WORLD_NAME' (file: $(basename "$WORLD_FILE"))"
fi

# Terrain worlds reference their textures with relative URIs (mesh/aerial.png),
# so the directory holding the world has to be searchable.
WORLD_DIR="$(dirname "$WORLD_FILE")"

# Pick a MicroXRCEAgent that can actually load. After an OS upgrade the old
# /usr/local/bin build can still be first on PATH while linking libraries the
# new release no longer ships (22.04 -> 24.04: libspdlog.so.1 is gone). It then
# dies instantly and PX4's /fmu topics never appear, with no obvious error.
AGENT_BIN=""
if (( START_AGENT )); then
	for cand in "${AGENT_BIN_OVERRIDE:-}" "$HOME/.local/bin/MicroXRCEAgent" \
	            "$(command -v MicroXRCEAgent 2>/dev/null || true)"; do
		[[ -n "$cand" && -x "$cand" ]] || continue
		if ldd "$cand" 2>/dev/null | grep -q "not found"; then
			warn "Skipping broken agent $cand (missing: $(ldd "$cand" | awk '/not found/{print $1}' | tr '\n' ' '))"
			continue
		fi
		AGENT_BIN="$cand"; break
	done
	if [[ -z "$AGENT_BIN" ]]; then
		warn "No working MicroXRCEAgent found; ROS 2 PX4 topics will not appear."
		warn "  Rebuild it for this OS - see docs/JAZZY_MIGRATION.md (--no-agent to silence)"
		START_AGENT=0
	else
		log "MicroXRCEAgent: $AGENT_BIN"
	fi
fi

QGC_CANDIDATES=()

if (( START_QGC )); then
	# Already running? Do not start a second one - it would fight for UDP
	# 14550/14540 and the existing instance is known to work.
	if pgrep -f 'QGroundControl' >/dev/null 2>&1; then
		log "QGroundControl is already running; not starting another"
		START_QGC=0
	fi
fi

if (( START_QGC )); then
	if [[ -n "$QGC_BIN" ]]; then
		QGC_CANDIDATES=("$QGC_BIN")
	else
		# A packaged install wins; otherwise try AppImages newest-first.
		# NOTE: newest is not necessarily runnable. Recent QGroundControl
		# AppImages are built against glibc 2.38 and will not start on
		# Ubuntu 22.04 (glibc 2.35); an older AppImage alongside it still
		# works. So collect every candidate and try them in order rather
		# than committing to one.
		local_qgc="$(command -v qgroundcontrol 2>/dev/null || true)"
		[[ -n "$local_qgc" ]] && QGC_CANDIDATES+=("$local_qgc")
		while IFS= read -r -d '' f; do
			QGC_CANDIDATES+=("$f")
		done < <(find "$HOME/Downloads" "$HOME" -maxdepth 1 \
			-name 'QGroundControl*.AppImage' -printf '%T@\t%p\0' 2>/dev/null \
			| sort -zrn | cut -z -f2-)
	fi

	if (( ${#QGC_CANDIDATES[@]} == 0 )); then
		warn "QGroundControl not found; skipping (--no-qgc to silence)"
		START_QGC=0
	else
		for f in "${QGC_CANDIDATES[@]}"; do
			[[ -x "$f" ]] || chmod +x "$f" 2>/dev/null || true
		done
		log "QGroundControl candidates: ${#QGC_CANDIDATES[@]}"
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

if [[ -n "$ROS_DISTRO_SETUP" && -f "$ROS_DISTRO_SETUP" ]]; then
	source_relaxed "$ROS_DISTRO_SETUP"
	log "ROS 2 sourced: $ROS_DISTRO_SETUP (${ROS_DISTRO:-unknown})"

	# Jazzy's ros_gz packages pull in Gazebo "vendor" packages whose setup sets
	# GZ_CONFIG_PATH to their own directories, which only register the
	# transport/msgs subcommands. That hides the system /usr/share/gz/sim8.yaml,
	# so after sourcing ROS `gz sim` prints the generic help and exits - the
	# server "starts" and dies instantly. Put the system config dir back.
	#
	# Worse, the same setup puts the vendor copies of libgz-msgs/transport on
	# LD_LIBRARY_PATH. The system Gazebo (OSRF gz-harmonic) then loads a
	# libgz-msgs10 built differently from the one it was linked against:
	#   libgz-sim8.so.8: undefined symbol: scc_info_Header_gz_2fmsgs_2fheader_2eproto
	# So Gazebo and PX4 run in a "Gazebo-only" environment with every /opt/ros
	# library path removed; only ROS nodes see the ROS environment. The two
	# sides talk over gz-transport on the wire, so no shared libraries needed.
	GZ_ONLY_LD="$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/ros/' | paste -sd: - || true)"
else
	warn "No ROS 2 installation found under /opt/ros (looked for jazzy, humble)"
fi

# The ros_gz bridge must be built against the SAME Gazebo generation the
# simulator uses, or it connects, reports no error, and silently delivers
# nothing ("Unknown message type"). This exact mismatch broke the sensor
# bridge on Humble, where the default ros-humble-ros-gz targets Fortress
# while the simulator is Harmonic. Jazzy pairs with Harmonic natively, so
# there the stock ros-jazzy-ros-gz-bridge is already correct.
BRIDGE_BIN="$(command -v parameter_bridge 2>/dev/null || true)"
[[ -z "$BRIDGE_BIN" && -n "${ROS_DISTRO:-}" ]] \
	&& BRIDGE_BIN="/opt/ros/$ROS_DISTRO/lib/ros_gz_bridge/parameter_bridge"
if [[ -f "$BRIDGE_BIN" ]]; then
	if ldd "$BRIDGE_BIN" 2>/dev/null | grep -q "libignition-msgs"; then
		warn "ros_gz bridge is built against Ignition (Fortress-era), but the"
		warn "simulator is Gazebo $GZ_VER. Sensor topics will appear in"
		warn "'ros2 topic list' and carry NO data. Fix on Humble with:"
		warn "    sudo apt install ros-humble-ros-gzharmonic-bridge"
		warn "On Jazzy the stock ros-jazzy-ros-gz-bridge is already correct."
	fi
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
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PKG_DIR/models:$PKG_DIR/worlds:$WORLD_DIR"
export GZ_IP=127.0.0.1

# A georeferenced terrain world carries the real-world origin it was generated
# for. Hand it to PX4 so GPS, the QGC map and the terrain all agree.
if [[ -z "${PX4_HOME_LAT:-}" ]]; then
	t_lat="$(grep -om1 '<latitude_deg>[^<]*' "$WORLD_FILE" | cut -d'>' -f2 || true)"
	t_lon="$(grep -om1 '<longitude_deg>[^<]*' "$WORLD_FILE" | cut -d'>' -f2 || true)"
	t_alt="$(grep -om1 '<elevation>[^<]*' "$WORLD_FILE" | cut -d'>' -f2 || true)"
	if [[ -n "$t_lat" && -n "$t_lon" ]]; then
		export PX4_HOME_LAT="$t_lat"
		export PX4_HOME_LON="$t_lon"
		export PX4_HOME_ALT="${t_alt:-0}"
		log "World origin: lat=$t_lat lon=$t_lon alt=${t_alt:-0}"
	fi
fi

# In standalone mode PX4 does NOT re-source gz_env.sh, so PX4_GZ_MODELS is ours
# to set. Point it at this repository when the requested vehicle lives here, so
# custom models do not have to be copied into the PX4 tree.
MODEL_DIR_NAME="${MODEL#gz_}"
if [[ -d "$PKG_DIR/models/$MODEL_DIR_NAME" ]]; then
	export PX4_GZ_MODELS="$PKG_DIR/models"
	log "Vehicle model from this repo: $PKG_DIR/models/$MODEL_DIR_NAME"
else
	log "Vehicle model from PX4: $PX4_GZ_MODELS/$MODEL_DIR_NAME"
fi

# gpu_lidar and camera sensors render on the GPU. On a hybrid laptop the
# discrete GPU must be selected explicitly or Gazebo silently uses the Intel
# iGPU, which makes a dense LiDAR crawl. These variables are harmless when no
# NVIDIA driver is present.
if [[ -e /dev/nvidiactl ]] || command -v nvidia-smi >/dev/null 2>&1; then
	export __NV_PRIME_RENDER_OFFLOAD=1
	export __GLX_VENDOR_LIBRARY_NAME=nvidia
	# The GUI renders through GLX, but the simulation SERVER renders the
	# camera and GPU-LiDAR headless through EGL, which the GLX setting does
	# not affect. Left alone, glvnd tries Mesa first, Mesa fails on the NVIDIA
	# device ("failed to create dri2 screen") and silently falls back to CPU
	# rendering: fine for a toy world, but a real-site world stalls the
	# sensors at 0 Hz. Pin EGL to the NVIDIA driver.
	if [[ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]]; then
		export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
	fi
	export __VK_LAYER_NV_optimus=NVIDIA_only
	log "NVIDIA GPU detected - rendering offloaded to it"
else
	warn "No NVIDIA driver loaded; Gazebo will render on the Intel iGPU."
	warn "  A 16-beam LiDAR will be slow. Install with:"
	warn "    sudo ubuntu-drivers install nvidia:595   # then reboot"
fi

# ------------------------------------------------------------------ launch --
log "World: $WORLD_FILE"

spawn gz-server gzenv gz sim --verbose=1 -r -s "$WORLD_FILE"
alive_after "$SPAWN_PID" 2 || { report_crash gz-server; die "Gazebo server exited on startup"; }
wait_for "Gazebo world '$WORLD_NAME'" 60 \
	gzenv bash -c "gz service -i --service /world/$WORLD_NAME/scene/info 2>&1 | grep -q 'Service providers'"

if (( ! HEADLESS )); then
	spawn gz-gui gzenv gz sim -g
fi

if (( START_AGENT )); then
	spawn microxrce "$AGENT_BIN" udp4 -p 8888
	alive_after "$SPAWN_PID" 2 || { report_crash microxrce; die "MicroXRCEAgent exited on startup"; }
fi

log "Starting PX4 SITL (standalone, attaching to '$WORLD_NAME')..."
cd "$PX4_DIR"
spawn px4 gzenv env \
	PX4_GZ_STANDALONE=1 \
	PX4_GZ_WORLD="$WORLD_NAME" \
	PX4_SIM_MODEL="$MODEL" \
	PX4_SYS_AUTOSTART="$AUTOSTART" \
	PX4_GZ_MODEL_POSE="$SPAWN_POSE" \
	"$PX4_BUILD/bin/px4" -d
cd "$PROJECT_DIR"

wait_for "PX4 vehicle spawned in Gazebo" 90 \
	gzenv bash -c "gz model --list 2>/dev/null | grep -q '${MODEL#gz_}_0'"
# NOTE: do NOT gate readiness on "Ready for takeoff". PX4 only prints that
# when every preflight check passes, and one of those checks is "connection to
# the GCS". With no GCS running the simulation is perfectly healthy but that
# line never appears. Gate on PX4 finishing its startup script instead.
wait_for "PX4 boot complete" 90 \
	bash -c "grep -aq 'Startup script returned successfully' '$LOG_DIR/px4.log'"

# Without a GCS the datalink-loss check keeps preflight red and the vehicle
# refuses to arm, which would break --mission in a headless run. Clear it.
if ! pgrep -f 'QGroundControl' >/dev/null 2>&1; then
	log "No GCS present - clearing datalink-loss arming check (NAV_DLL_ACT=0)"
	"$PX4_BUILD/bin/px4-param" set NAV_DLL_ACT 0 >/dev/null 2>&1 || true
fi

wait_for_soft "PX4 preflight green (Ready for takeoff)" 45 \
	bash -c "grep -aq 'Ready for takeoff' '$LOG_DIR/px4.log'" || true

if (( START_QGC )); then
	qgc_started=0
	for qgc in "${QGC_CANDIDATES[@]}"; do
		log "Trying QGroundControl: $qgc"
		spawn qgc "$qgc"
		if alive_after "$SPAWN_PID" 6; then
			log "QGroundControl running: $qgc"
			qgc_started=1
			break
		fi
		report_crash qgc
		if grep -q "GLIBC_2\.\(3[6-9]\|[4-9][0-9]\)' not found" "$LOG_DIR/qgc.log" 2>/dev/null; then
			warn "  -> this AppImage needs a newer glibc than this system has"
			warn "     ($(ldd --version | head -n1)). Trying an older build."
		fi
	done
	if (( ! qgc_started )); then
		warn "Could not start QGroundControl. The simulation is unaffected;"
		warn "start a working GCS yourself, or re-run with --no-qgc."
	fi
fi

if (( START_SENSORS )); then
	log "Bridging LiDAR / camera into ROS 2..."
	spawn sensors ros2 launch poc1_landing_world sensors.launch.py
	if alive_after "$SPAWN_PID" 5; then
		log "sensor bridge running"
	else
		report_crash sensors
	fi
fi

if (( START_UNITY )); then
	[[ "$MODEL" == gz_x500_threat_scanner ]] \
		|| warn "--unity: model $MODEL has no PosePublisher; use --model gz_x500_threat_scanner"
	log "Starting the Unity camera bridge..."
	# /clock is already bridged by the sensors launch; do not publish it twice.
	spawn unity-bridge ros2 launch poc2_unity_camera unity_camera.launch.py \
		model:="${MODEL#gz_}_0" clock:="$( ((START_SENSORS)) && echo false || echo true)"
	alive_after "$SPAWN_PID" 4 || { report_crash unity-bridge; die "Unity bridge exited on startup (built? use --build)"; }
	if [[ -x "$UNITY_PLAYER" ]]; then
		log "Starting Unity player ($UNITY_REGION): $UNITY_PLAYER"
		spawn unity-player "$UNITY_PLAYER" --region "$UNITY_REGION" \
			-logFile "$LOG_DIR/unity-player-engine.log"
		alive_after "$SPAWN_PID" 5 || report_crash unity-player
	else
		warn "No Unity player at $UNITY_PLAYER - use the editor instead:"
		warn "  open unity/GuardianSim, then Guardian > Play Drone Camera (compound)"
		warn "  (build a player: Unity -batchmode -quit -projectPath unity/GuardianSim \\"
		warn "     -executeMethod Guardian.Sim.Editor.GuardianMenu.BuildPlayer)"
	fi
fi

if (( START_JOY )); then
	compgen -G '/dev/input/js*' >/dev/null \
		|| warn "No game controller found (/dev/input/js*). Connect one by USB or Bluetooth; the driver picks it up."
	log "Starting game-controller flight + LiDAR scan recorder (rviz2 opens)..."
	spawn joystick ros2 launch poc3_manual_scan manual_scan.launch.py \
		model:="${MODEL#gz_}_0" world:="$WORLD_NAME" \
		pose_bridge:="$( ((START_UNITY)) && echo false || echo true)" \
		rviz:="${JOY_RVIZ:-true}"
	alive_after "$SPAWN_PID" 4 || { report_crash joystick; die "manual flight exited on startup (built? use --build)"; }
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

    Gazebo world : $WORLD_NAME   (server$( ((HEADLESS)) && echo ", headless" || echo " + GUI"))
    Vehicle      : ${MODEL#gz_}_0
    MAVLink      : udp 14550 (GCS)   udp 14540 (onboard/offboard)
    uXRCE-DDS    : udp 8888  $( ((START_AGENT)) || echo "(agent not started)" )
    Logs         : $LOG_DIR
$( ((START_UNITY)) && echo "    Unity camera : /unity_cam/image_raw  /unity_cam/camera_info  (bridge 127.0.0.1:5700)" )
$( ((START_JOY)) && echo "    Controller   : START take off | sticks fly (Mode 2) | Y record scan | X save | B land
                   scans -> ~/UAV/data/scans/<date_time>/  (log: .sim_logs/joystick.log)" )

  Useful checks:
    gz model --list
    ros2 topic list | grep /fmu
    ros2 topic echo /fmu/out/vehicle_local_position_v1

  Press Ctrl+C to stop everything.

EOF

# Keep the script in the foreground so the cleanup trap owns the whole stack.
wait

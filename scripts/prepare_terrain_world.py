#!/usr/bin/env python3
"""Make a gazebo_terrain_generator world usable with PX4 SITL.

Worlds produced by https://github.com/saiaravind19/gazebo_terrain_generator
load their own Gazebo systems, but only the ones needed to *look* right:
physics, sensors, imu, navsat, scene-broadcaster, user-commands.

PX4 needs more than that. Without the magnetometer and air-pressure systems the
vehicle spawns fine and then refuses to arm:

    Preflight Fail: barometer 0 missing
    Preflight Fail: Found 0 compass (required: 1)
    Preflight Fail: heading estimate invalid

Because the world declares plugins inline, Gazebo uses that list and PX4's own
server.config is not applied, so the missing systems have to be added to the
world file itself. This script does that, idempotently, and prints the
PX4_HOME_* values that match the terrain's real-world origin.

Usage:
    scripts/prepare_terrain_world.py /path/to/applepark.world
    scripts/prepare_terrain_world.py /path/to/applepark.world --install

--install copies the world and its mesh/ directory into the ROS package's
worlds/ directory so ./start_uav_sim.sh --world <name> can find it.
"""

import argparse
import re
import shutil
import sys
from pathlib import Path

# Systems PX4 expects, mirroring PX4's
# src/modules/simulation/gz_bridge/server.config
REQUIRED_SYSTEMS = [
    ("gz-sim-physics-system", "gz::sim::systems::Physics"),
    ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
    ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ("gz-sim-contact-system", "gz::sim::systems::Contact"),
    ("gz-sim-imu-system", "gz::sim::systems::Imu"),
    ("gz-sim-air-pressure-system", "gz::sim::systems::AirPressure"),
    ("gz-sim-air-speed-system", "gz::sim::systems::AirSpeed"),
    ("gz-sim-apply-link-wrench-system", "gz::sim::systems::ApplyLinkWrench"),
    ("gz-sim-navsat-system", "gz::sim::systems::NavSat"),
    ("gz-sim-magnetometer-system", "gz::sim::systems::Magnetometer"),
]

WORLD_OPEN_RE = re.compile(r'(<world\s+name\s*=\s*"([^"]+)"\s*>)')


def spherical_coordinates(text):
    def grab(tag):
        m = re.search(rf"<{tag}>\s*([-\d.eE+]+)\s*</{tag}>", text)
        return m.group(1) if m else None
    return grab("latitude_deg"), grab("longitude_deg"), grab("elevation")


def inject(text):
    """Add any missing PX4 system plugins just inside <world>."""
    m = WORLD_OPEN_RE.search(text)
    if not m:
        sys.exit("error: no <world name=\"...\"> element found")
    world_tag, world_name = m.group(1), m.group(2)

    missing = [(f, n) for f, n in REQUIRED_SYSTEMS
               if f'filename="{f}"' not in text and f"filename='{f}'" not in text]
    if not missing:
        return text, world_name, []

    block = ["", "    <!-- Added by prepare_terrain_world.py: Gazebo systems PX4 requires.",
             "         Without magnetometer/air-pressure PX4 reports no compass and no",
             "         barometer, and refuses to arm. -->"]
    for filename, name in missing:
        block.append(
            f'    <plugin filename="{filename}" name="{name}"></plugin>')
    block.append("")

    insert_at = m.end(1)
    return text[:insert_at] + "\n".join(block) + text[insert_at:], world_name, missing


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world", type=Path, help="generated .world file")
    ap.add_argument("--install", action="store_true",
                    help="copy into the ROS package worlds/ directory")
    ap.add_argument("--package-worlds", type=Path,
                    default=Path(__file__).resolve().parent.parent
                    / "src/pocs/poc1_landing_world/worlds",
                    help="destination for --install")
    args = ap.parse_args()

    src = args.world.resolve()
    if not src.is_file():
        sys.exit(f"error: not a file: {src}")

    text = src.read_text()
    patched, world_name, added = inject(text)

    if added:
        backup = src.with_suffix(src.suffix + ".orig")
        if not backup.exists():
            shutil.copy2(src, backup)
        src.write_text(patched)
        print(f"Added {len(added)} system plugin(s) to {src.name}:")
        for f, _ in added:
            print(f"  + {f}")
        print(f"  (original saved as {backup.name})")
    else:
        print(f"{src.name}: already has every system PX4 needs")

    if src.stem != world_name:
        print(f"\nWARNING: <world name=\"{world_name}\"> does not match the file "
              f"stem \"{src.stem}\".\n"
              f"         PX4 waits on /world/{world_name}/scene/info, so pass "
              f"--world {world_name}\n"
              f"         and make sure the file is named {world_name}"
              f"{src.suffix}.")

    lat, lon, alt = spherical_coordinates(patched)
    if lat:
        print("\nTerrain origin - launch PX4 with these so GPS matches the map:")
        print(f"  PX4_HOME_LAT={lat} PX4_HOME_LON={lon} PX4_HOME_ALT={alt}")

    if args.install:
        dest_dir = args.package_worlds
        dest_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dest_dir / src.name)
        mesh = src.parent / "mesh"
        if mesh.is_dir():
            shutil.copytree(mesh, dest_dir / "mesh", dirs_exist_ok=True)
        print(f"\nInstalled into {dest_dir}")
        print("Rebuild so the world is installed into the ROS share directory:")
        print("  colcon build --symlink-install --packages-select poc1_landing_world")
        print(f"Then:  ./start_uav_sim.sh --world {world_name}")


if __name__ == "__main__":
    main()

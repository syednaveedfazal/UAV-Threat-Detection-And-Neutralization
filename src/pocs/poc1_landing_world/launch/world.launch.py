"""Launch the landing-mission world in modern Gazebo (Harmonic).

This starts only the simulator; PX4 spawns the vehicle itself. Run PX4 in
standalone mode afterwards so it attaches to the world started here:

    PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=landing_mission \
    PX4_SIM_MODEL=gz_x500 PX4_SYS_AUTOSTART=4001 \
    ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -d

or just use ../../../start_uav_sim.sh, which orchestrates the whole stack.

The Gazebo environment below mirrors PX4's own gz_env.sh. The server config is
what loads the physics, sensors, IMU, magnetometer and NavSat systems; without
it the spawned x500 produces no sensor data and PX4's EKF never converges.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('poc1_landing_world')

    px4_dir = os.environ.get('PX4_DIR', os.path.expanduser('~/PX4-Autopilot'))
    px4_build = os.path.join(px4_dir, 'build', 'px4_sitl_default')

    px4_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    px4_plugins = os.path.join(
        px4_build, 'src', 'modules', 'simulation', 'gz_plugins')
    px4_server_config = os.path.join(
        px4_dir, 'src', 'modules', 'simulation', 'gz_bridge', 'server.config')

    pkg_models = os.path.join(pkg_share, 'models')
    pkg_worlds = os.path.join(pkg_share, 'worlds')

    def prepend(var, *paths):
        existing = os.environ.get(var, '')
        joined = ':'.join(p for p in paths if p)
        return f'{existing}:{joined}' if existing else joined

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    world_path = [os.path.join(pkg_worlds, ''), world, '.sdf']

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='landing_mission',
            description='World basename in the package worlds/ directory'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Start the Gazebo server without the GUI'),

        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            prepend('GZ_SIM_RESOURCE_PATH', pkg_models, pkg_worlds, px4_models)),
        SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            prepend('GZ_SIM_SYSTEM_PLUGIN_PATH', px4_plugins)),
        SetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH', px4_server_config),
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),

        ExecuteProcess(
            cmd=['gz', 'sim', '--verbose=1', '-r', '-s', *world_path],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', '-g'],
            output='screen',
            condition=UnlessCondition(headless),
        ),

        # Gazebo clock -> ROS, so ROS nodes can run on simulation time.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),
    ])

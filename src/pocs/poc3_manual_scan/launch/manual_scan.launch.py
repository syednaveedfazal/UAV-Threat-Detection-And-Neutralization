"""Fly with a game controller and scan: controller driver, flight node, scan recorder, rviz2.

    ros2 launch poc3_manual_scan manual_scan.launch.py
    ./start_uav_sim.sh --world <bonn.sdf> --model gz_x500_threat_scanner --sensors --joystick [--unity]

Needs the simulation with the threat-scanner drone and sensors.launch.py
(LiDAR + /clock + static base_link -> lidar_link). Arguments:
    pose_bridge:=false   when unity_camera.launch.py already bridges the drone pose
    joy:=false           no controller driver (e.g. to drive /joy from a script)
    rviz:=false          no rviz2 window
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('poc3_manual_scan'))
    config = str(share / 'config' / 'controller.yaml')
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    pose_topic = PythonExpression(["'/model/' + '", model, "' + '/pose'"])
    # gz names a sensor topic after its full scoped path
    imu_gz = PythonExpression(["'/world/' + '", world, "' + '/model/' + '", model,
                               "' + '/link/base_link/sensor/imu_sensor/imu'"])

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='x500_threat_scanner_0'),
        DeclareLaunchArgument('world', default_value='bonn_poppelsdorf'),
        DeclareLaunchArgument('pose_bridge', default_value='true'),
        DeclareLaunchArgument('joy', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(package='joy', executable='game_controller_node', name='game_controller',
             parameters=[config], condition=IfCondition(LaunchConfiguration('joy')), output='log'),
        Node(package='poc3_manual_scan', executable='joy_flight.py', name='joy_flight',
             parameters=[config], output='screen'),
        Node(package='poc3_manual_scan', executable='scan_recorder.py', name='scan_recorder',
             parameters=[{'use_sim_time': True, 'pose_topic': pose_topic}], output='screen'),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='scan_pose_bridge',
             arguments=[[pose_topic, '@geometry_msgs/msg/PoseStamped[gz.msgs.Pose']],
             condition=IfCondition(LaunchConfiguration('pose_bridge')), output='log'),
        # IMU for LiDAR-inertial odometry later (phase L2); recorded with the scans.
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='scan_imu_bridge',
             arguments=[[imu_gz, '@sensor_msgs/msg/Imu[gz.msgs.IMU']],
             # (the bridge's startup log prints the gz name; the ROS topic is /imu)
             remappings=[(imu_gz, '/imu')], output='log'),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', str(share / 'rviz' / 'manual_scan.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='log'),
    ])

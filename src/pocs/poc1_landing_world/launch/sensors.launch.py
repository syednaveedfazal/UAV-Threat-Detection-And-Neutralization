"""Bridge the threat-scanner drone's LiDAR and camera from Gazebo into ROS 2.

Run this alongside a simulation that spawned `x500_threat_scanner`:

    ros2 launch poc1_landing_world sensors.launch.py

or let the launcher do it:  ./start_uav_sim.sh --model gz_x500_threat_scanner --sensors

Topics published into ROS 2:
    /lidar_3d/points   sensor_msgs/PointCloud2   ~10 Hz, 512x16 rays
    /lidar_3d/scan     sensor_msgs/LaserScan     single-ring view of the same
    /front_cam/image   sensor_msgs/Image
    /front_cam/camera_info  sensor_msgs/CameraInfo

The Gazebo-side names come from the <topic> elements in the model SDF, so they
are independent of the world and model instance names.

Note on frames: gz publishes these in the sensor's own frame (lidar_link /
front_camera_link). Nothing here publishes a TF tree connecting them to the
vehicle or to a map frame - that is the job of whatever mapping node consumes
them (see `static_tf` below for a minimal stand-in).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_static_tf = LaunchConfiguration('static_tf')

    bridges = [
        # LiDAR point cloud. gz.msgs.PointCloudPacked -> sensor_msgs/PointCloud2
        '/lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        # Flat scan, handy for quick 2D checks and for rviz LaserScan display
        '/lidar_3d@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        # Forward camera
        '/front_cam@sensor_msgs/msg/Image[gz.msgs.Image',
        '/front_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        # Simulation clock
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'static_tf', default_value='true',
            description='Publish a minimal base_link -> sensor static TF so '
                        'point clouds can be viewed in rviz2 immediately'),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='sensor_bridge',
            arguments=bridges,
            remappings=[
                ('/front_cam', '/front_cam/image'),
            ],
            output='screen',
        ),

        # Minimal TF so rviz2 has somewhere to put the cloud. Replace with a
        # real odometry -> base_link -> sensor chain when you add mapping.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar',
            arguments=['0', '0', '0.13', '0', '0', '0', 'base_link', 'lidar_link'],
            condition=IfCondition(publish_static_tf),
            output='log',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_cam',
            arguments=['0.15', '0', '0.03', '0', '0', '0',
                       'base_link', 'front_camera_link'],
            condition=IfCondition(publish_static_tf),
            output='log',
        ),
    ])

"""Unity gimbal camera: drone pose Gazebo -> Unity, images Unity -> ROS 2.

    ros2 launch poc2_unity_camera unity_camera.launch.py
    ros2 launch poc2_unity_camera unity_camera.launch.py fake:=true   # no Unity needed

Then start Unity (editor: Guardian > Play Drone Camera, or the player build);
it connects to the bridge on 127.0.0.1:5700 and retries until it is up.

Published:
    /unity_cam/image_raw    sensor_msgs/Image (rgb8, stamped with the pose's sim time)
    /unity_cam/camera_info  sensor_msgs/CameraInfo
    TF gz_world -> unity_cam_optical

`clock:=false` when sensors.launch.py (poc1) already bridges /clock, so it is
not published twice.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model = LaunchConfiguration('model')
    pose_topic = PythonExpression(["'/model/' + '", model, "' + '/pose'"])

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='x500_threat_scanner_0',
                              description='Gazebo model instance name of the drone'),
        DeclareLaunchArgument('clock', default_value='true',
                              description='Bridge /clock from Gazebo'),
        DeclareLaunchArgument('fake', default_value='false',
                              description='Run the fake Unity client (test pattern) instead of Unity'),
        DeclareLaunchArgument('port', default_value='5700'),

        # Drone ground truth from the model's PosePublisher (sim-time stamped).
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='unity_pose_bridge',
            arguments=[[pose_topic, '@geometry_msgs/msg/PoseStamped[gz.msgs.Pose']],
            output='log',
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='unity_clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            condition=IfCondition(LaunchConfiguration('clock')),
            output='log',
        ),
        Node(
            package='poc2_unity_camera', executable='unity_bridge.py', name='unity_bridge',
            parameters=[{'use_sim_time': True,
                         'pose_topic': pose_topic,
                         'port': ParameterValue(LaunchConfiguration('port'), value_type=int)}],
            output='screen',
        ),
        Node(
            package='poc2_unity_camera', executable='fake_unity_client.py', name='fake_unity',
            arguments=['--port', LaunchConfiguration('port')],
            condition=IfCondition(LaunchConfiguration('fake')),
            output='screen',
        ),
    ])

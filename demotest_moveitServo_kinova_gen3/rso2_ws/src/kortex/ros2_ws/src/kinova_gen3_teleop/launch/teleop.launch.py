"""
Launch Kinova Gen3 teleoperation.

Starts:
  1. teleop_node — camera + mediapipe + left hand start/stop + right hand control
  2. arm_controller — IK + trajectory publishing

Prerequisite (run in another terminal or via docker-compose):
  ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
    robot_ip:=xxx.xxx.xxx.xxx use_fake_hardware:=true
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('kinova_gen3_teleop')
    config = os.path.join(pkg, 'config', 'teleop_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument('show_camera', default_value='true'),

        Node(
            package='kinova_gen3_teleop',
            executable='teleop_node',
            name='hand_pose_publisher',
            output='screen',
            parameters=[config, {
                'camera_id': LaunchConfiguration('camera_id'),
                'show_camera': LaunchConfiguration('show_camera'),
            }],
        ),

        Node(
            package='kinova_gen3_teleop',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            parameters=[config],
        ),
    ])

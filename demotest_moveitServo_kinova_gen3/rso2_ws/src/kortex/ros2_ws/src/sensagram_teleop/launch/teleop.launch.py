from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('sensagram_teleop')

    params_file = os.path.join(pkg_share, 'config', 'teleop_params.yaml')

    return LaunchDescription([
        Node(
            package='sensagram_teleop',
            executable='sensagram_teleop_node',
            name='sensagram_teleop',
            output='screen',
            parameters=[params_file]
        )
    ])
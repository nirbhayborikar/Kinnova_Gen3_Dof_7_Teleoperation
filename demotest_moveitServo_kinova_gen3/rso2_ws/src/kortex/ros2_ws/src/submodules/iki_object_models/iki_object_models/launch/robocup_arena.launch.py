from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the package
    pkg_share = get_package_share_directory('iki_object_models')
    
    # Path to the world file
    world_file_path = os.path.join(pkg_share, 'models', 'robocup_2023', 'arena.world')
    
    # Use the gazebo_ros launch file
    gazebo_launch_path = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={'world': world_file_path}.items(),
        ),
    ])

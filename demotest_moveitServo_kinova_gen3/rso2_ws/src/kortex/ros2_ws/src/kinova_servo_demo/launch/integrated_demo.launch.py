import os
import yaml
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Path setup
    # Use your actual xacro path from your previous snippet
    xacro_path = "/root/ws_moveit/src/moveit2/moveit_ros/moveit_servo/kortex/ros2_ws/src/iki_kortex_description/urdf_xacro/kortex_standalone.urdf.xacro"
    
    # 2. Process XACRO to get URDF String
    xacro_command = [
        'xacro', xacro_path,
        'name:=gen3', 'arm:=gen3', 'dof:=7', 'gripper:=robotiq_2f_140',
        'use_fake_hardware:=true', 'sim_gazebo:=true'
    ]
    urdf_content = subprocess.check_output(xacro_command).decode('utf-8')

    # 3. Load SRDF and other MoveIt configs
    # Change 'moveit_servo' to wherever your kortex.srdf actually lives
    # Based on your previous code, it's in the 'moveit_servo' share dir
    srdf_path = os.path.join(get_package_share_directory("moveit_servo"), "config", "kortex.srdf")
    with open(srdf_path, 'r') as f:
        srdf_content = f.read()

    # 4. Load your YAML configs
    pkg_share = get_package_share_directory("kinova_servo_demo")
    
    with open(os.path.join(pkg_share, "config", "gen3_2_servo.yaml"), 'r') as f:
        servo_params = yaml.safe_load(f)

    # 5. Launch our INTEGRATED node
    integrated_node = Node(
        package="kinova_servo_demo",
        executable="kinova_integrated_servo",
        name="kinova_integrated_servo",
        output="screen",
        parameters=[
            servo_params,
            {"robot_description": urdf_content},
            {"robot_description_semantic": srdf_content},
            # Add kinematics/limits if you have them, otherwise Servo uses defaults
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        integrated_node
    ])

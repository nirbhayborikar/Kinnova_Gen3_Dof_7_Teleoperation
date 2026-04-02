# Copyright (c) 2021 PickNik, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Marq Rasmussen, Denis Stogl

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    robot_type = LaunchConfiguration("robot_type")
    robot_ip = LaunchConfiguration("robot_ip")
    dof = LaunchConfiguration("dof")
    vision = LaunchConfiguration("vision")
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    print(controllers_file.perform(context))
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    robot_name = LaunchConfiguration("robot_name")
    prefix = LaunchConfiguration("prefix")
    gripper = LaunchConfiguration("gripper")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    robot_traj_controller = LaunchConfiguration("robot_controller")
    robot_pos_controller = LaunchConfiguration("robot_pos_controller")
    robot_hand_controller = LaunchConfiguration("robot_hand_controller")
    fault_controller = LaunchConfiguration("fault_controller")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")
    gripper_joint_name = LaunchConfiguration("gripper_joint_name")

    # if we are using fake hardware then we can't use the internal gripper communications of the hardware
    use_fake_hardware_value = use_fake_hardware.perform(context)
    if use_fake_hardware_value == "true":
        use_internal_bus_gripper_comm = "false"

    description_path_middle=""
    if (description_package.perform(context) == "iki_kortex_description"):
        description_path_middle="urdf_xacro"
    elif (description_package.perform(context) == "kortex_description"):
        description_path_middle="robots"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_path_middle, description_file]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "name:=",
            robot_name,
            " ",
            "arm:=",
            robot_type,
            " ",
            "dof:=",
            dof,
            " ",
            "vision:=",
            vision,
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "gripper:=",
            gripper,
            " ",
            "use_internal_bus_gripper_comm:=",
            use_internal_bus_gripper_comm,
            " ",
            "gripper_max_velocity:=",
            gripper_max_velocity,
            " ",
            "gripper_max_force:=",
            gripper_max_force,
            " ",
            "gripper_joint_name:=",
            gripper_joint_name,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Check if controllers_file starts with '/' (absolute path)
    controllers_file_value = controllers_file.perform(context)
    if controllers_file_value.startswith('/'):
        # Use absolute path directly
        robot_controllers = controllers_file_value
    else:
        # Use relative path as before
        robot_controllers = PathJoinSubstitution(
            [
                FindPackageShare(description_package),
                "arms/" + robot_type.perform(context) + "/" + dof.perform(context) + "dof/config",
                controllers_file,
            ]
        )   

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_traj_controller, "-c", "/controller_manager"],
    )

    robot_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_pos_controller, "--inactive", "-c", "/controller_manager"],
    )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_hand_controller, "-c", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", gripper, "' != ''"])),
    )

    # only start the fault controller if we are using hardware
    fault_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[fault_controller, "-c", "/controller_manager"],
        condition=IfCondition(use_internal_bus_gripper_comm),
    )

    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        robot_traj_controller_spawner,
        robot_pos_controller_spawner,
        fault_controller_spawner
    ]

    start_robot_hand_controller = gripper.perform(context) != ""
    # Conditionally add robot_hand_controller_spawner
    if start_robot_hand_controller:
        nodes_to_start.append(robot_hand_controller_spawner)

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # Robot specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type", description="Type/series of robot.", default_value="gen3", choices=["gen3", "gen3_lite"]
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "dof", default_value="7", description="DoF of robot.", choices=["6", "7"]
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "vision", default_value="true", description="Enable vision modules.", choices=["true", "false"]
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", default_value="192.168.1.10", description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "username", description="Robot session username.", default_value="admin"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "password", description="Robot session password.", default_value="admin"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "port", description="Robot port for tcp connection.", default_value="10000"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "port_realtime",
            description="Robot port for udp realtime control.",
            default_value="10001",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "session_inactivity_timeout_ms",
            description="Robot session inactivity timeout in milliseconds.",
            default_value="60000",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "connection_inactivity_timeout_ms",
            description="Robot connection inactivity timeout in milliseconds.",
            default_value="2000",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("iki_kortex_config"), "config", "ros2_control.yaml"]), #ros2_controllers.yaml
            description="YAML file with the controllers configuration. (If global path is given use it if not use default kinova package path)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="iki_kortex_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
            choices=["kortex_description", "iki_kortex_description"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="kortex_standalone.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="kortex",
            description="Name of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="robotiq_2f_140",
            choices=["", "robotiq_2f_85", "robotiq_2f_140"],
            description="Name of the gripper attached to the arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_pos_controller",
            default_value="twist_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_hand_controller",
            default_value="robotiq_gripper_controller",
            description="Robot hand controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fault_controller",
            default_value="fault_controller",
            description="Name of the 'fault controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="true",
            description="Use internal bus for gripper communication?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_velocity",
            default_value="100.0",
            description="Max velocity for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_force",
            default_value="100.0",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_joint_name",
            default_value="finger_joint",
            description="Max force for gripper commands",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
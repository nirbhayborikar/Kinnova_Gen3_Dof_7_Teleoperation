# #!/usr/bin/env python3
# import os
# import launch
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# import launch_ros.actions
# import launch_ros.descriptions
# from launch_ros.actions import Node, ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode
# from ament_index_python.packages import get_package_share_directory
# from launch.conditions import IfCondition, UnlessCondition
# from launch_param_builder import ParameterBuilder
# from moveit_configs_utils import MoveItConfigsBuilder
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import IncludeLaunchDescription


# def generate_launch_description():
#     # Launch arguments
#     declared_arguments = []
    
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "launch_as_standalone_node",
#             default_value="false",
#             description="Launch Servo as a standalone node or as a node component",
#         )
#     )
    
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "use_sim_time",
#             default_value="true",
#             description="Use simulation time",
#         )
#     )
    
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "use_rviz",
#             default_value="true",
#             description="Launch RViz2",
#         )
#     )
    
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "rviz_config",
#             default_value=os.path.join(
#                 get_package_share_directory("kortex_demo_config"),
#                 "config",
#                 "rviz",
#                 "kortex_table.rviz"
#             ),
#             description="RViz config file",
#         )
#     )
    
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "servo_config_file",
#             default_value="gen3_servo.yaml",
#             description="Servo configuration file",
#         )
#     )

#     # Build MoveIt configuration
#     moveit_config = (
#         MoveItConfigsBuilder("kortex", package_name="iki_kortex_moveit_config")
#         .robot_description(file_path="config/kortex.urdf")
#         .trajectory_execution(file_path="config/moveit_controllers.yaml")
#         .planning_scene_monitor(
#             publish_robot_description=True,
#             publish_robot_description_semantic=True,
#         )
#         .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
#         .to_moveit_configs()
#     )

#     # Get parameters for the Servo node
#     servo_params = {
#         "moveit_servo": ParameterBuilder("iki_kortex_moveit_config")
#         .yaml(
#             PathJoinSubstitution([
#                 get_package_share_directory("iki_kortex_moveit_config"),
#                 "config",
#                 LaunchConfiguration("servo_config_file")
#             ])
#         )
#         .to_dict()
#     }

#     # Parameters for acceleration limiting filter
#     acceleration_filter_update_period = {"update_period": 0.01}
#     planning_group_name = {"planning_group_name": "arm"}
    
#     # Use simulation time parameter
#     use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

#     # RViz node (only if use_rviz is true)
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", LaunchConfiguration("rviz_config")],
#         parameters=[
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.robot_description_kinematics,
#             use_sim_time,
#         ],
#         condition=IfCondition(LaunchConfiguration("use_rviz")),
#         additional_env={'QT_X11_NO_MITSHM': '1'},
#         emulate_tty=True,
#     )

#     # Servo container for running as component
#     servo_container = ComposableNodeContainer(
#         name="moveit_servo_container",
#         namespace="/",
#         package="rclcpp_components",
#         executable="component_container_mt",
#         composable_node_descriptions=[
#             # Servo node as component
#             ComposableNode(
#                 package="moveit_servo",
#                 plugin="moveit_servo::ServoNode",
#                 name="servo_node",
#                 parameters=[
#                     servo_params,
#                     acceleration_filter_update_period,
#                     planning_group_name,
#                     moveit_config.robot_description,
#                     moveit_config.robot_description_semantic,
#                     moveit_config.robot_description_kinematics,
#                     moveit_config.joint_limits,
#                     use_sim_time,
#                 ],
#                 condition=UnlessCondition(LaunchConfiguration("launch_as_standalone_node")),
#             ),
#             # Robot State Publisher
#             ComposableNode(
#                 package="robot_state_publisher",
#                 plugin="robot_state_publisher::RobotStatePublisher",
#                 name="robot_state_publisher",
#                 parameters=[moveit_config.robot_description, use_sim_time],
#             ),
#             # Static transform from world to base_link
#             ComposableNode(
#                 package="tf2_ros",
#                 plugin="tf2_ros::StaticTransformBroadcasterNode",
#                 name="static_tf2_broadcaster",
#                 parameters=[
#                     {"child_frame_id": "base_link", "frame_id": "world"},
#                     use_sim_time,
#                 ],
#             ),
#         ],
#         output="screen",
#         parameters=[use_sim_time],
#         condition=UnlessCondition(LaunchConfiguration("launch_as_standalone_node")),
#     )

#     # Standalone Servo node
#     servo_standalone_node = Node(
#         package="moveit_servo",
#         executable="servo_node",
#         name="servo_node",
#         parameters=[
#             servo_params,
#             acceleration_filter_update_period,
#             planning_group_name,
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.robot_description_kinematics,
#             moveit_config.joint_limits,
#             use_sim_time,
#         ],
#         output="screen",
#         condition=IfCondition(LaunchConfiguration("launch_as_standalone_node")),
#     )

#     # Optional: Include simulation launch if needed
#     # simulation_launch = IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource([
#     #         PathJoinSubstitution([
#     #             get_package_share_directory("iki_kortex_moveit_config"),
#     #             "launch",
#     #             "sim.launch.py"
#     #         ])
#     #     ]),
#     #     launch_arguments=[
#     #         ("use_sim_time", LaunchConfiguration("use_sim_time")),
#     #     ]
#     # )

#     return launch.LaunchDescription(
#         declared_arguments +
#         [
#             rviz_node,
#             servo_container,
#             servo_standalone_node,
#         ]
#     )































#!/usr/bin/env python3
import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_ros.actions
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_as_standalone_node",
            default_value="false",
            description="Launch Servo as a standalone node or as a node component",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                get_package_share_directory("kortex_demo_config"),
                "config", "rviz", "kortex_table.rviz"
            ]),
            description="RViz config file",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "servo_config_file",
            default_value="gen3_servo.yaml",
            description="Servo configuration file",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_controllers",
            default_value="true",
            description="Start ros2_control controllers",
        )
    )

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("kortex", package_name="iki_kortex_moveit_config")
        .robot_description(file_path="config/kortex.urdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Get servo parameters
    servo_params = {
        "moveit_servo": ParameterBuilder("iki_kortex_moveit_config")
        .yaml(
            PathJoinSubstitution([
                get_package_share_directory("iki_kortex_moveit_config"),
                "config",
                LaunchConfiguration("servo_config_file")
            ])
        )
        .to_dict()
    }

    # Common parameters
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "arm"}
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    
    # Combine all parameters for servo
    servo_all_params = [
        servo_params,
        acceleration_filter_update_period,
        planning_group_name,
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        use_sim_time,
    ]

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="servo_rviz",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            use_sim_time,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        additional_env={'QT_X11_NO_MITSHM': '1'},
        emulate_tty=True,
    )

    # Controller manager (needed for servo to publish commands)
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory("iki_kortex_config"),
                "config",
                "ros2_control.yaml",
            ]),
            use_sim_time,
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_controllers")),
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "300",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_controllers")),
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c", "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_controllers")),
    )

    # Servo container (component mode)
    servo_container = ComposableNodeContainer(
        name="moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=servo_all_params,
            ),
        ],
        output="screen",
        parameters=[use_sim_time],
        condition=UnlessCondition(LaunchConfiguration("launch_as_standalone_node")),
    )

    # Standalone servo node
    servo_standalone_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=servo_all_params,
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_as_standalone_node")),
    )

    # Robot State Publisher (needed for transforms)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, use_sim_time],
    )

    return launch.LaunchDescription(
        declared_arguments +
        [
            controller_manager_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            robot_state_publisher_node,
            rviz_node,
            servo_container,
            servo_standalone_node,
        ]
    )
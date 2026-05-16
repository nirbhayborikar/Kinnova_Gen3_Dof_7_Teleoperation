#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def build_moveit_config(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    enable_sensors_3d = LaunchConfiguration("enable_sensors_3d").perform(context).lower() == "true"

    builder = (
        MoveItConfigsBuilder("kortex", package_name="iki_kortex_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
    )

    if enable_sensors_3d:
        sensors_file = "config/sensors_3d_sim.yaml" if use_sim_time else "config/sensors_3d.yaml"
        builder = builder.sensors_3d(file_path=sensors_file)

    moveit_config = builder.to_moveit_configs()

    # Inject sim time into robot description
    moveit_config.robot_description.update({
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    })

    return [generate_move_group_launch(moveit_config)]


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    enable_sensors_3d_arg = DeclareLaunchArgument(
        "enable_sensors_3d",
        default_value="false",
        description="Enable sensors_3d if true",
    )

    return LaunchDescription([
        use_sim_time_arg,
        enable_sensors_3d_arg,
        OpaqueFunction(function=build_moveit_config),
    ])

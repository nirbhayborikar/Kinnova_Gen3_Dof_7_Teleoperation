import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros.actions
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    # Launch arguments 
    declared_arguments = []

    rviz_config_path = "/root/ros2_ws/src/prj-iki-ros2/robots/kortex/iki_kortex_config/config/rviz"
    
    declared_arguments.append(DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(rviz_config_path, 'kortex.rviz'),
        description='RViz .rviz config file to load. If starts with /, treated as absolute path. Otherwise, relative to rviz_config_path.'
    ))
    
    declared_arguments.append(DeclareLaunchArgument(
        'dark_mode',
        default_value='true',
        description='Enable dark mode.'
    ))
    
    declared_arguments.append(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true.'
    ))
    
    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("kortex", package_name="iki_kortex_moveit_config")
        .robot_description(
            file_path="config/kortex.urdf"
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    moveit_config.robot_description.update({"use_sim_time": LaunchConfiguration("use_sim_time")})
    
    if IfCondition(launch.substitutions.LaunchConfiguration('dark_mode')):
        rviz_stylesheet = os.path.join(rviz_config_path, 'dark.qss')
    else:
        rviz_stylesheet = ""
    
    # Handle rviz_config path: absolute if starts with /, otherwise relative to rviz_config_path
    rviz_config_resolved = PythonExpression([
        "'", LaunchConfiguration('rviz_config'), "' if '", LaunchConfiguration('rviz_config'), "'.startswith('/') else '", rviz_config_path, "/' + '", LaunchConfiguration('rviz_config'), "'"
    ])
    
    # RViz node arguments - conditional stylesheet
    rviz_args = ['-d', rviz_config_resolved]
    
    # Add stylesheet argument conditionally
    rviz_args.extend(['--stylesheet', rviz_stylesheet])
    
    # RViz node
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',  # Changed from 'rviz' to avoid naming conflicts
        output='screen',
        arguments=rviz_args,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        # Add some additional options to help with stability
        additional_env={'QT_X11_NO_MITSHM': '1'},
        emulate_tty=True,
    )
    
    return launch.LaunchDescription([
        *declared_arguments,
        rviz_node
    ])
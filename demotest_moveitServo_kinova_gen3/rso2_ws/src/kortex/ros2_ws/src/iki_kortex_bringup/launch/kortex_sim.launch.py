import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    AppendEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix


launch_args = [
    # Gripper arguments
    DeclareLaunchArgument('gripper', default_value='robotiq_2f_140',
                          description='Which gripper to use.',
                          choices=['robotiq_2f_140', 'robotiq_2f_85']),

    # General arguments
    DeclareLaunchArgument('controllers_file', default_value='ros2_control.yaml',
                          description='YAML file with the controllers configuration.'),
    DeclareLaunchArgument('description_package', default_value='iki_kortex_description',
                          description='Description package with robot URDF/XACRO files.'),
    DeclareLaunchArgument('description_file', default_value='kortex_standalone.urdf.xacro',
                          description='URDF/XACRO description file with the robot.'),
    DeclareLaunchArgument('robot_name', default_value='gen3',
                          description='Robot name.'),
    DeclareLaunchArgument('prefix', default_value='""',
                          description='Prefix of the joint names.'),
    DeclareLaunchArgument('robot_controller', default_value='joint_trajectory_controller',
                          description='Robot controller to start.'),
    DeclareLaunchArgument('robot_pos_controller', default_value='twist_controller',
                          description='Robot position controller to start.'),
    DeclareLaunchArgument('robot_hand_controller', default_value='robotiq_gripper_controller',
                          description='Robot hand controller to start.'),

    # Spawn position/orientation
    # In front of regal
    # DeclareLaunchArgument('spawn_x', default_value='-1.7', description='Spawn position X'),
    # DeclareLaunchArgument('spawn_y', default_value='-1.25', description='Spawn position Y'),
    # DeclareLaunchArgument('spawn_z', default_value='0.0', description='Spawn position Z'),
    # DeclareLaunchArgument('spawn_R', default_value='0.0', description='Spawn roll'),
    # DeclareLaunchArgument('spawn_P', default_value='0.0', description='Spawn pitch'),
    # DeclareLaunchArgument('spawn_Y', default_value='1.57', description='Spawn yaw'),

    # On table
    DeclareLaunchArgument('spawn_x', default_value='1.05', description='Spawn position X'),
    DeclareLaunchArgument('spawn_y', default_value='-2.25', description='Spawn position Y'),
    DeclareLaunchArgument('spawn_z', default_value='0.65', description='Spawn position Z'),
    DeclareLaunchArgument('spawn_R', default_value='0.0', description='Spawn roll'),
    DeclareLaunchArgument('spawn_P', default_value='0.0', description='Spawn pitch'),
    DeclareLaunchArgument('spawn_Y', default_value='0.0', description='Spawn yaw'),

    # Gazebo arguments
    DeclareLaunchArgument('world_file', description='Gazebo world file', default_value='/root/ros2_ws/src/prj-iki-ros2/robots/kortex/iki_object_models/models/robocup_2025/arena.world.sdf'),
    DeclareLaunchArgument('spawn_objects', default_value='true', description='Spawn YCB objects in the default world'),
]


def launch_setup(context, *args, **kwargs):
    # Extract arguments
    gripper = LaunchConfiguration('gripper').perform(context)
    controllers_file = LaunchConfiguration('controllers_file').perform(context)
    description_package = LaunchConfiguration('description_package').perform(context)
    description_file = LaunchConfiguration('description_file').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    robot_traj_controller = LaunchConfiguration('robot_controller').perform(context)
    robot_pos_controller = LaunchConfiguration('robot_pos_controller').perform(context)
    robot_hand_controller = LaunchConfiguration('robot_hand_controller').perform(context)

    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    spawn_z = LaunchConfiguration('spawn_z').perform(context)
    spawn_R = LaunchConfiguration('spawn_R').perform(context)
    spawn_P = LaunchConfiguration('spawn_P').perform(context)
    spawn_Y = LaunchConfiguration('spawn_Y').perform(context)

    world_file = LaunchConfiguration('world_file').perform(context)
    spawn_objects = LaunchConfiguration('spawn_objects').perform(context).lower() == 'true'

    default_world_file = '/root/ros2_ws/src/prj-iki-ros2/robots/kortex/iki_object_models/models/robocup_2025/arena.world.sdf'
    is_default_world = world_file == default_world_file
    
    if world_file == '':
        world_file = 'empty.sdf'
    elif not os.path.isabs(world_file):
        world_file = PathJoinSubstitution([
            FindPackageShare('iki_kortex_moveit_config'),
            'worlds',
            world_file
        ])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("iki_kortex_config"), "config", controllers_file,
    ])

    # Robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf_xacro", description_file]),
        " ",
        "robot_ip:=xxx.yyy.zzz.www",
        " ",
        f"name:={robot_name}",
        " ",
        f"arm:=gen3",
        " ",
        f"dof:=7",
        " ",
        f"vision:=true",
        " ",
        f"prefix:={prefix}",
        " ",
        f"sim_gazebo:=true",
        " ",
        "simulation_controllers:=",
        robot_controllers,
        " ",
        f"gripper:={gripper}",
        " ",
    ])
    robot_description = {'robot_description': robot_description_content.perform(context)}

        # Gazebo resource path for Robotiq
    robotiq_description_prefix = get_package_prefix('robotiq_description')
    gz_robotiq_env_var_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(robotiq_description_prefix, 'share')
    )

    # Gazebo resource path for kortex_description
    kortex_description_prefix = get_package_prefix('kortex_description')
    gz_kortex_env_var_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(kortex_description_prefix, 'share')
    )

    # Gazebo resource path for iki_kortex
    iki_kortex_description_prefix = get_package_prefix('iki_kortex_description')
    gz_iki_kortex_env_var_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(iki_kortex_description_prefix, 'share')
    )

    gz_server_params = [
        {'use_sim_time': True}, 
        {'on_exit_shutdown': True},
        {'world_sdf_file': world_file}
    ]


    # Gazebo server + bridges inside container
    gazebo_container = ComposableNodeContainer(
        name='gazebo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='ros_gz_sim',
                plugin='ros_gz_sim::GzServer',
                name='gz_server',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=gz_server_params
            ),
            ComposableNode(
                package='ros_gz_bridge',
                plugin='ros_gz_bridge::RosGzBridge',
                name='main_bridge',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[
                    {'use_sim_time': True},
                    {'config_file': PathJoinSubstitution([
                        FindPackageShare("iki_kortex_config"), "config", "sim", "main_bridge.yaml"
                    ])}
                ]
            ),
            ComposableNode(
                package='ros_gz_bridge',
                plugin='ros_gz_bridge::RosGzBridge',
                name='kortex_vision_bridge',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[{'use_sim_time': True},
                            {'config_file': PathJoinSubstitution([
                                FindPackageShare("iki_kortex_config"), "config", "sim", "kortex_vision_bridge.yaml"
                            ])}],
            )
        ]
    )

    # Spawn robot entity
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description_content,
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-R', spawn_R, '-P', spawn_P, '-Y', spawn_Y,
        ]
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Static Transform Publisher
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_to_optical_frame',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'camera_color_frame', 'color_optical_frame'],
        parameters=[{'use_sim_time': True}]
    )

    # Controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    robot_traj_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[robot_traj_controller, '-c', '/controller_manager'],
    )

    robot_pos_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[robot_pos_controller, '--inactive', '-c', '/controller_manager'],
    )

    robot_hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[robot_hand_controller, '-c', '/controller_manager'],
    )

    return_list = [
        gz_robotiq_env_var_resource_path,
        gz_kortex_env_var_resource_path,
        gz_iki_kortex_env_var_resource_path,
        gazebo_container,
        gz_spawn_entity,
        robot_state_publisher,
        static_tf_publisher,
        joint_state_broadcaster_spawner,
        robot_traj_controller_spawner,
        robot_pos_controller_spawner,
        robot_hand_controller_spawner,
    ]

    if spawn_objects and is_default_world:
        # Example usage
        x_range = (1.5, 1.8)   # X can be between -5 and 5
        y_range = (-2.05, -2.45)   # Y can be between -3 and 3

        positions = generate_positions(x_range, y_range, num_objects=2)
        object_configs = [
            {'name': 'bleach_cleanser', 'model': 'ycb_021_bleach_cleanser', 'x': '-1.90', 'y': '-0.31', 'z': '0.5', 'Y': '0.35'},
            {'name': 'sugar_box', 'model': 'ycb_004_sugar_box', 'x': '-1.72', 'y': '-0.38', 'z': '0.45', 'Y': '0.35'},
            {'name': 'peach', 'model': 'ycb_015_peach', 'x': '-1.51', 'y': '-0.33', 'z': '0.42', 'Y': '0.0'},
            {'name': 'peach_table', 'model': 'ycb_015_peach', 'x': str(positions[0][0]), 'y': str(positions[0][1]), 'z': '0.77', 'Y': '0.0'},
            {'name': 'lemon_table', 'model': 'ycb_014_lemon', 'x': str(positions[1][0]), 'y': str(positions[1][1]), 'z': '0.77', 'Y': '0.0'},
        ]

        for obj_config in object_configs:
            spawn_object_node = Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', obj_config['name'],
                    '-x', obj_config['x'], 
                    '-y', obj_config['y'], 
                    '-z', obj_config['z'],
                    '-Y', obj_config['Y'],
                    '-file', PathJoinSubstitution([
                        FindPackageShare('iki_object_models'), 
                        'models', 'ycb_models', obj_config['model'], 'model-1_4.sdf'
                    ]),
                ]
            )
            return_list.append(spawn_object_node)

    return return_list


def generate_launch_description():
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


import random

def generate_positions(x_range, y_range, num_objects):
    positions = []
    for _ in range(num_objects):
        x = random.uniform(x_range[0], x_range[1])
        y = random.uniform(y_range[0], y_range[1])
        positions.append((x, y))
    return positions
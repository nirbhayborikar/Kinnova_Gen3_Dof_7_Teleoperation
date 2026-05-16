import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context: LaunchContext, *args, **kwargs):
    # Retrieve the prefix at runtime.
    prefix = LaunchConfiguration('prefix').perform(context)
    if prefix:
        node_name = prefix + '_apriltag_node'
    else:
        node_name = 'apriltag_node'

    return [
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name=node_name,
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'prefix': LaunchConfiguration('prefix'),
                 'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'image_transport': LaunchConfiguration('image_transport')}
            ],
            remappings=[
                ('image_rect', LaunchConfiguration('image_rect_topic')),
                ('camera_info', LaunchConfiguration('camera_info_topic'))
            ]
        )
    ]

def generate_launch_description():
    image_rect_topic_arg = DeclareLaunchArgument(
        'image_rect_topic',
        default_value='/image_rect_color',
        description='Topic for rectified images.'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera_info',
        description='Topic for camera info.'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('apriltag_ros'), 'cfg', 'tags_36h11.yaml'),
        description='Full path to the parameter file to use.'
    )

    # Default prefix is an empty string.
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Prefix for the node name, detection topic and tfs (used if multiple nodes want to be ran at the same time)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Set to true to use simulation time.'
    )

    image_transport_arg = DeclareLaunchArgument(
        'image_transport',
        default_value='raw',
        description='Image transport type (e.g. raw, compressed). This will override the config file. If compressed is used /compressed is appended to the topic name.'
    )

    return LaunchDescription([
        image_rect_topic_arg,
        camera_info_topic_arg,
        params_file_arg,
        prefix_arg,
        use_sim_time_arg,
        image_transport_arg,
        OpaqueFunction(function=launch_setup)
    ])

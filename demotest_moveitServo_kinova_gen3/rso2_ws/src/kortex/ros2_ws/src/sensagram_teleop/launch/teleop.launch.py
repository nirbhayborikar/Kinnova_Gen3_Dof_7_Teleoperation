from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_id',       default_value='0'),
        DeclareLaunchArgument('rate',            default_value='30.0'),
        DeclareLaunchArgument('show_camera',     default_value='true'),
        DeclareLaunchArgument('linear_scale',    default_value='0.35'),
        DeclareLaunchArgument('angular_scale',   default_value='0.25'),
        DeclareLaunchArgument('command_frame',   default_value='base_link'),
        DeclareLaunchArgument('pinch_threshold', default_value='0.07'),

        Node(
            package='sensagram_teleop',
            executable='servo_teleop_node',
            name='servo_teleop_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'camera_id':       LaunchConfiguration('camera_id'),
                'rate':            LaunchConfiguration('rate'),
                'show_camera':     LaunchConfiguration('show_camera'),
                'linear_scale':    LaunchConfiguration('linear_scale'),
                'angular_scale':   LaunchConfiguration('angular_scale'),
                'command_frame':   LaunchConfiguration('command_frame'),
                'pinch_threshold': LaunchConfiguration('pinch_threshold'),
            }],
        ),
    ])

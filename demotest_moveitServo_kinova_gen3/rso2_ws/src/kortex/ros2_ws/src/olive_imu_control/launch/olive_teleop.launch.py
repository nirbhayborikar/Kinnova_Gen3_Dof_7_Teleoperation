from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('olive_imu_control')
    
    # Parameters file
    params_file = os.path.join(pkg_dir, 'config', 'olive_teleop_params.yaml')

    # ── Olive teleop node ────────────────────────────────────────────────────
    olive_teleop_node = Node(
        package='olive_imu_control',
        executable='olive_teleop_node',
        name='olive_teleop_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
    )

    # ── Unpause MoveIt Servo (after 3s to let servo_node start) ─────────────
    unpause_servo = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     '/servo_node/pause_servo',
                     'std_srvs/srv/SetBool',
                     '{data: false}'],
                output='screen'
            )
        ]
    )

    # ── Set command type to TWIST = 1 (after 4s, after unpause) ─────────────
    set_command_type = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     '/servo_node/switch_command_type',
                     'moveit_msgs/srv/ServoCommandType',
                     '{command_type: 1}'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        olive_teleop_node,
        unpause_servo,
        set_command_type,
    ])
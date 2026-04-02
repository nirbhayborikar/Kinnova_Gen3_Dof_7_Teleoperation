from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='olive_imu_teleop',
            executable='olive_imu_teleop_node',
            name='olive_imu_teleop_node',
            output='screen',
            parameters=[{
                'imu_frame_id': 'id001_sensor_link',
                'control_rate_hz': 50.0,
                'max_linear_velocity': 0.3,
                'max_angular_velocity': 0.8,
                'max_linear_accel': 1.0,
                'max_angular_accel': 2.0,
                'deadzone': 0.005,
                'smoothing_factor': 0.3,
                'timeout_ms': 500.0,
                'max_consecutive_timeouts': 3,
                'linear_scale': 0.8,
                'angular_scale': 1.5,
                'invert_x': False,
                'invert_y': True,
                'invert_z': True,
            }],
        ),
    ])

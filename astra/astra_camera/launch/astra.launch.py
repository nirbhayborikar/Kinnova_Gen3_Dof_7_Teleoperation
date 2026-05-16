from launch_ros.actions import Node, PushRosNamespace

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # launch arguments
    args = [
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("depth_registration", default_value="false"),
        DeclareLaunchArgument("serial_number", default_value=""),
        DeclareLaunchArgument("device_num", default_value="1"),
        DeclareLaunchArgument("vendor_id", default_value="0x2bc5"),
        DeclareLaunchArgument("product_id", default_value=""),
        DeclareLaunchArgument("enable_point_cloud", default_value="true"),
        DeclareLaunchArgument("enable_colored_point_cloud", default_value="false"),
        DeclareLaunchArgument("point_cloud_qos", default_value="default"),
        DeclareLaunchArgument("connection_delay", default_value="100"),
        DeclareLaunchArgument("color_width", default_value="640"),
        DeclareLaunchArgument("color_height", default_value="480"),
        DeclareLaunchArgument("color_fps", default_value="30"),
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("flip_color", default_value="false"),
        DeclareLaunchArgument("color_qos", default_value="default"),
        DeclareLaunchArgument("color_camera_info_qos", default_value="default"),
        DeclareLaunchArgument("depth_width", default_value="640"),
        DeclareLaunchArgument("depth_height", default_value="480"),
        DeclareLaunchArgument("depth_fps", default_value="30"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("flip_depth", default_value="false"),
        DeclareLaunchArgument("depth_qos", default_value="default"),
        DeclareLaunchArgument("depth_camera_info_qos", default_value="default"),
        DeclareLaunchArgument("ir_width", default_value="640"),
        DeclareLaunchArgument("ir_height", default_value="480"),
        DeclareLaunchArgument("ir_fps", default_value="30"),
        DeclareLaunchArgument("enable_ir", default_value="true"),
        DeclareLaunchArgument("flip_ir", default_value="false"),
        DeclareLaunchArgument("ir_qos", default_value="default"),
        DeclareLaunchArgument("ir_camera_info_qos", default_value="default"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("tf_publish_rate", default_value="10.0"),
        DeclareLaunchArgument("ir_info_url", default_value=""),
        DeclareLaunchArgument("color_info_url", default_value=""),
        DeclareLaunchArgument("color_depth_synchronization", default_value="false"),
        DeclareLaunchArgument("oni_log_level", default_value="verbose"),
        DeclareLaunchArgument("oni_log_to_console", default_value="false"),
        DeclareLaunchArgument("oni_log_to_file", default_value="false"),
        DeclareLaunchArgument("enable_d2c_viewer", default_value="false"),
        DeclareLaunchArgument("enable_publish_extrinsic", default_value="false"),
    ]

    # substitutions
    cfg = LaunchConfiguration
    camera_ns = cfg("camera_name")

    # node parameters
    params = {
        "camera_name": camera_ns,
        "depth_registration": cfg("depth_registration"),
        "serial_number": cfg("serial_number"),
        "device_num": cfg("device_num"),
        "vendor_id": cfg("vendor_id"),
        "product_id": cfg("product_id"),
        "enable_point_cloud": cfg("enable_point_cloud"),
        "enable_colored_point_cloud": cfg("enable_colored_point_cloud"),
        "point_cloud_qos": cfg("point_cloud_qos"),
        "connection_delay": cfg("connection_delay"),
        "color_width": cfg("color_width"),
        "color_height": cfg("color_height"),
        "color_fps": cfg("color_fps"),
        "enable_color": cfg("enable_color"),
        "flip_color": cfg("flip_color"),
        "color_qos": cfg("color_qos"),
        "color_camera_info_qos": cfg("color_camera_info_qos"),
        "depth_width": cfg("depth_width"),
        "depth_height": cfg("depth_height"),
        "depth_fps": cfg("depth_fps"),
        "enable_depth": cfg("enable_depth"),
        "flip_depth": cfg("flip_depth"),
        "depth_qos": cfg("depth_qos"),
        "depth_camera_info_qos": cfg("depth_camera_info_qos"),
        "ir_width": cfg("ir_width"),
        "ir_height": cfg("ir_height"),
        "ir_fps": cfg("ir_fps"),
        "enable_ir": cfg("enable_ir"),
        "flip_ir": cfg("flip_ir"),
        "ir_qos": cfg("ir_qos"),
        "ir_camera_info_qos": cfg("ir_camera_info_qos"),
        "publish_tf": cfg("publish_tf"),
        "tf_publish_rate": cfg("tf_publish_rate"),
        "ir_info_url": cfg("ir_info_url"),
        "color_info_url": cfg("color_info_url"),
        "color_depth_synchronization": cfg("color_depth_synchronization"),
        "oni_log_level": cfg("oni_log_level"),
        "oni_log_to_console": cfg("oni_log_to_console"),
        "oni_log_to_file": cfg("oni_log_to_file"),
        "enable_d2c_viewer": cfg("enable_d2c_viewer"),
        "enable_publish_extrinsic": cfg("enable_publish_extrinsic"),
    }

    # node inside namespace group
    camera_node = GroupAction(
        [
            PushRosNamespace(camera_ns),
            Node(
                package="astra_camera",
                executable="astra_camera_node",
                name="camera",
                output="screen",
                parameters=[params],
                remappings=[("depth/color/points", "depth_registered/points")],
            ),
        ]
    )

    return LaunchDescription(args + [camera_node])

import yaml
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

configurable_parameters = [
    {"name": "device", "default": "192.168.1.10", "description": "Device IPv4 address"},
    {
        "name": "camera",
        "default": "camera",
        "description": "'camera' should uniquely identify the device. All topics are pushed down into the 'camera' namespace.",
    },
    {
        "name": "camera_link_frame_id",
        "default": "camera_link",
        "description": "Camera link frame identifier",
    },
    {
        "name": "color_frame_id",
        "default": "camera_color_frame",
        "description": "Color camera frame identifier",
    },
    {
        "name": "depth_frame_id",
        "default": "camera_depth_frame",
        "description": "Depth camera frame identifier",
    },
    {
        "name": "color_camera_info_url",
        "default": "",
        "description": "URL of custom calibration file for color camera. See camera_info_manager docs for calibration URL details",
    },
    {
        "name": "depth_camera_info_url",
        "default": "",
        "description": "URL of custom calibration file for depth camera. See camera_info_manager docs for calibration URL details",
    },
    {
        "name": "depth_rtsp_element_config",
        "default": "depth latency=30",
        "description": "RTSP element configuration for depth stream",
    },
    {
        "name": "depth_rtp_depay_element_config",
        "default": "rtpgstdepay",
        "description": "RTP element configuration for depth stream",
    },
    {
        "name": "color_rtsp_element_config",
        "default": "color latency=30",
        "description": "RTSP element configuration for color stream",
    },
    {
        "name": "color_rtp_depay_element_config",
        "default": "rtph264depay",
        "description": "RTP element configuration for color stream",
    },
    {
        "name": "launch_color",
        "default": "true",
        "description": "Launch the color image node",
    },
    {
        "name": "launch_depth",
        "default": "true",
        "description": "Launch the depth image node",
    },
    {
        "name": "depth_registration",
        "default": "true",
        "description": "Hardware depth registration",
    },
    {
        "name": "max_color_pub_rate",
        "default": "10.0",
        "description": "Maximum image publication rate",
    },
    {
        "name": "max_depth_pub_rate",
        "default": "10.0",
        "description": "Maximum image publication rate",
    },
    {
        "name": "container_name",
        "default": "kinova_vision_container",
        "description": "Name of the component container",
    },
]


def declare_configurable_parameters():
    return [
        DeclareLaunchArgument(
            param["name"],
            default_value=param["default"],
            description=param["description"],
        )
        for param in configurable_parameters
    ]


def set_configurable_parameters(parameters):
    return dict(
        [(param["name"], LaunchConfiguration(param["name"])) for param in parameters]
    )


def launch_setup(context, *args, **kwargs):
    # Create composable nodes for zero-copy communication
    depth_composable_node = ComposableNode(
        package="kinova_vision",
        plugin="ros_kortex_vision::Vision",
        name="kinova_vision_depth",
        namespace=LaunchConfiguration("camera"),
        parameters=[
            {
                "camera_type": "depth",
                "camera_name": "depth",
                "camera_info_url_default": "package://kinova_vision/launch/calibration/default_depth_calib_%ux%u.ini",
                "camera_info_url_user": LaunchConfiguration(
                    "depth_camera_info_url"
                ).perform(context),
                "stream_config": "rtspsrc location=rtsp://"
                + LaunchConfiguration("device").perform(context)
                + "/"
                + LaunchConfiguration("depth_rtsp_element_config").perform(context)
                + " ! "
                + LaunchConfiguration("depth_rtp_depay_element_config").perform(
                    context
                ),
                "frame_id": LaunchConfiguration("depth_frame_id").perform(context),
                "max_pub_rate": LaunchConfiguration("max_depth_pub_rate"),
            }
        ],
        remappings=[
            ("camera_info", "depth/camera_info"),
            ("image_raw", "depth/image_raw"),
        ],
        condition=IfCondition(LaunchConfiguration("launch_depth")),
    )

    color_composable_node = ComposableNode(
        package="kinova_vision",
        plugin="ros_kortex_vision::Vision",
        name="kinova_vision_color",
        namespace=LaunchConfiguration("camera"),
        parameters=[
            {
                "camera_type": "color",
                "camera_name": "color",
                "camera_info_url_default": "package://kinova_vision/launch/calibration/default_color_calib_%ux%u.ini",
                "camera_info_url_user": LaunchConfiguration(
                    "color_camera_info_url"
                ).perform(context),
                "stream_config": "rtspsrc location=rtsp://"
                + LaunchConfiguration("device").perform(context)
                + "/"
                + LaunchConfiguration("color_rtsp_element_config").perform(context)
                + " ! "
                + LaunchConfiguration("color_rtp_depay_element_config").perform(context)
                + " ! avdec_h264 ! videoconvert",
                "frame_id": LaunchConfiguration("color_frame_id").perform(context),
                "max_pub_rate": LaunchConfiguration("max_color_pub_rate"),
            }
        ],
        remappings=[
            ("camera_info", "color/camera_info"),
            ("image_raw", "color/image_raw"),
        ],
        condition=IfCondition(LaunchConfiguration("launch_color")),
    )

    # Depth image processing composable nodes for zero-copy
    register_composable_node = ComposableNode(
        package="depth_image_proc",
        plugin="depth_image_proc::RegisterNode",
        name="register_node",
        namespace=LaunchConfiguration("camera"),
        remappings=[
            ("rgb/camera_info", "color/camera_info"),
            ("depth/camera_info", "depth/camera_info"),
            ("depth/image_rect", "depth/image_raw"),
        ],
        parameters=[
            {
                "fill_upsampling_holes": True,
            }
        ],
        condition=IfCondition(LaunchConfiguration("depth_registration")),
    )

    pointcloud_composable_node = ComposableNode(
        package="depth_image_proc",
        plugin="depth_image_proc::PointCloudXyzrgbNode",
        name="point_cloud_xyzrgb",
        namespace=LaunchConfiguration("camera"),
        remappings=[
            ("rgb/camera_info", "color/camera_info"),
            ("depth/camera_info", "depth/camera_info"),
            ("rgb/image_rect_color", "color/image_raw"),
            ("depth/image_rect", "depth/image_raw"),
            ("points", "depth/color/points"),
        ],
        condition=IfCondition(LaunchConfiguration("depth_registration")),
    )

    # Build composable node list based on conditions
    composable_nodes = []

    if LaunchConfiguration("launch_depth").perform(context) == "true":
        composable_nodes.append(depth_composable_node)

    if LaunchConfiguration("launch_color").perform(context) == "true":
        composable_nodes.append(color_composable_node)

    if LaunchConfiguration("depth_registration").perform(context) == "true":
        composable_nodes.extend([register_composable_node, pointcloud_composable_node])

    # Always use container for zero-copy intra-process communication
    vision_container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace=LaunchConfiguration("camera"),
        package="rclcpp_components",
        executable="component_container_mt",  # Multi-threaded container for better performance
        composable_node_descriptions=composable_nodes,
        output="both",
        parameters=[
            {
                "use_intra_process_comms": True,  # Enable zero-copy
            }
        ],
    )

    # Static Transformation Publishers (these don't need to be in container)
    camera_depth_tf_publisher = Node(
        package="tf2_ros",
        namespace=LaunchConfiguration("camera"),
        executable="static_transform_publisher",
        name="camera_depth_tf_publisher",
        output="both",
        arguments=[
            "-0.0195",
            "-0.005",
            "0",
            "0",
            "0",
            "0",
            LaunchConfiguration("camera_link_frame_id"),
            LaunchConfiguration("depth_frame_id"),
        ],
        condition=IfCondition(LaunchConfiguration("launch_depth")),
    )

    camera_color_tf_publisher = Node(
        package="tf2_ros",
        namespace=LaunchConfiguration("camera"),
        executable="static_transform_publisher",
        name="camera_color_tf_publisher",
        output="both",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            LaunchConfiguration("camera_link_frame_id"),
            LaunchConfiguration("color_frame_id"),
        ],
        condition=IfCondition(LaunchConfiguration("launch_color")),
    )

    nodes_to_launch = [
        vision_container,
        camera_depth_tf_publisher,
        camera_color_tf_publisher,
    ]

    return nodes_to_launch


def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters() + [OpaqueFunction(function=launch_setup)]
    )

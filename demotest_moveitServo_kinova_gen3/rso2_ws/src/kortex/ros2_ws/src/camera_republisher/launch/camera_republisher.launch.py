from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def load_yaml_params(params_file_path):
    """Load and parse YAML parameters file"""
    try:
        with open(params_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading params file {params_file_path}: {e}")
        return {}

def generate_launch_description():
    # Declare launch argument for params file
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('camera_republisher'),
            'config',
            'kortex_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use'
    )

    # Get the params file path - we need to resolve this at generation time
    # For dynamic loading, we'll use the default and allow override
    default_params_path = os.path.join(
        get_package_share_directory('camera_republisher'),
        'config',
        'kortex_params.yaml'
    )
    
    # Load YAML parameters
    params_data = load_yaml_params(default_params_path)
    
    # Extract topic names from camera_republisher_node parameters
    republisher_params = params_data.get('camera_republisher_node', {}).get('ros__parameters', {})
    
    # Extract topic mappings
    input_camera = republisher_params.get('input_camera_topic', '/camera/image_raw')
    input_camera_info = republisher_params.get('input_camera_info_topic', '/camera/camera_info')
    input_depth = republisher_params.get('input_depth_topic', '/camera/depth/image_raw')
    input_depth_info = republisher_params.get('input_depth_info_topic', '/camera/depth/camera_info')
    
    output_camera = republisher_params.get('output_camera_topic', '/camera_out/image_raw')
    output_camera_info = republisher_params.get('output_camera_info_topic', '/camera_out/camera_info')
    output_depth = republisher_params.get('output_depth_topic', '/camera_out/depth/image_raw')
    output_depth_info = republisher_params.get('output_depth_info_topic', '/camera_out/depth/camera_info')
    output_points = republisher_params.get('output_points_topic', '/camera_out/points')

    # Create registered depth topic names using string operations
    depth_base = output_depth.rsplit("/", 1)[0]  # Get everything before the last "/"
    depth_registered_image = depth_base + "_registered/image_raw"
    
    camera_info_base = output_camera_info.rsplit("/", 1)[0]  # Get everything before the last "/"
    depth_registered_camera_info = camera_info_base + "/depth_registered/camera_info"

    # Get the params file for runtime loading
    params_file = LaunchConfiguration('params_file')

    # Define the container with all components
    container = ComposableNodeContainer(
        name='camera_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Camera Republisher Component - publishes to standard depth_image_proc topic names
            ComposableNode(
                package='camera_republisher',
                plugin='camera_republisher::CameraRepublisherComponent',
                name='camera_republisher_node',
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    # Only remap inputs from your camera topics
                    ('camera/image_raw', input_camera),
                    ('camera/camera_info', input_camera_info),
                    ('depth/image_raw', input_depth),
                    ('depth/camera_info', input_depth_info),
                    ('camera_out/image_raw', output_camera),
                    ('camera_out/camera_info', output_camera_info),
                    ('camera_out/depth/image_raw', output_depth),
                    ('camera_out/depth/camera_info', output_depth_info),
                ]
            ),
            # Register Node (for depth image registration)
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::RegisterNode',
                name='register_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    # Input remappings (these work fine)
                    ('depth/image_rect', input_depth),
                    ('depth/camera_info', input_depth_info),
                    ('rgb/camera_info', input_camera_info),
                    ('depth_registered/image_rect', depth_registered_image),
                    ('depth_registered/camera_info', depth_registered_camera_info)
                ]
            ),
            # Point Cloud XYZ RGB Node
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ('rgb/image_rect_color', input_camera),
                    ('rgb/camera_info', input_camera_info),
                    ('depth_registered/image_rect', depth_registered_image),
                    ('points', output_points)
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        params_file_arg,
        container
    ])
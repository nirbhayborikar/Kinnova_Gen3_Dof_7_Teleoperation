

# ######################GOKUL#################
# import launch
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from ament_index_python.packages import get_package_share_directory
# import yaml
# import os
# import subprocess

# def generate_launch_description():
#     declared_arguments = [
#         DeclareLaunchArgument(
#             "use_sim_time",
#             default_value="true",
#         ),
#         DeclareLaunchArgument(
#             "planning_group",
#             default_value="manipulator", # manipulator,
#         ),
#     ]
    
#     # **HARDCODED PATH - SIMPLEST**
#     xacro_path = "/root/ws_moveit/src/moveit2/moveit_ros/moveit_servo/kortex/ros2_ws/src/iki_kortex_description/urdf_xacro/kortex_standalone.urdf.xacro"
    
#     print(f"Looking for xacro file: {xacro_path}")
    
#     # Check if file exists
#     if not os.path.exists(xacro_path):
#         print(f"✗ XACRO file not found at {xacro_path}")
#         urdf_content = ""
#     else:
#         print(f"✓ Found xacro file")
#         xacro_command = [
#             'xacro',
#             xacro_path,
#             'name:=gen3',
#             'arm:=gen3',
#             'dof:=7',
#             'gripper:=robotiq_2f_140',
#             'use_fake_hardware:=true',
#             'sim_gazebo:=true',
#             'sim_isaac:=false',  # Add this
#             'vision:=false',     # Add this
#             'moveit_active:=true'  # Might be needed
#         ]
        
#         print(f"Running: {' '.join(xacro_command)}")
        
#         try:
#             # First, let's check if xacro works at all
#             test_result = subprocess.run(['which', 'xacro'], capture_output=True, text=True)
#             if test_result.returncode != 0:
#                 print("✗ xacro command not found. Install with: sudo apt install ros-${ROS_DISTRO}-xacro")
            
#             result = subprocess.run(
#                 xacro_command,
#                 capture_output=True,
#                 text=True,
#                 check=False  # Don't raise exception
#             )
            
#             if result.returncode == 0:
#                 urdf_content = result.stdout
#                 print(f"✓ XACRO processed successfully ({len(urdf_content)} chars)")
#                 # Save for debugging
#                 with open('/tmp/debug_urdf.xml', 'w') as f:
#                     f.write(urdf_content)
#                 print(f"✓ Saved to /tmp/debug_urdf.xml")
#             else:
#                 print(f"✗ XACRO failed with error: {result.stderr}")
#                 # Try without some arguments
#                 print("Trying simpler xacro command...")
#                 simple_command = ['xacro', xacro_path, '--inorder']
#                 simple_result = subprocess.run(simple_command, capture_output=True, text=True)
#                 if simple_result.returncode == 0:
#                     urdf_content = simple_result.stdout
#                     print(f"✓ Simple xacro worked ({len(urdf_content)} chars)")
#                 else:
#                     print(f"✗ Simple xacro also failed: {simple_result.stderr}")
#                     urdf_content = ""
                    
#         except Exception as e:
#             print(f"✗ Exception during xacro: {e}")
#             urdf_content = ""
    
#     # Get moveit_servo package directory
#     moveit_servo_dir = get_package_share_directory("moveit_servo")
#     print(f"MoveIt Servo directory: {moveit_servo_dir}")
    
#     # **FIX SRDF PATH - check if it's a file or directory**
#     srdf_path = os.path.join(moveit_servo_dir, "config", "kortex.srdf")
#     print(f"Looking for SRDF at: {srdf_path}")
    
#     if os.path.exists(srdf_path):
#         if os.path.isdir(srdf_path):
#             print(f"✗ SRDF path is a directory, not a file")
#             # Maybe there's a file inside the directory
#             srdf_files = [f for f in os.listdir(srdf_path) if f.endswith('.srdf')]
#             if srdf_files:
#                 srdf_path = os.path.join(srdf_path, srdf_files[0])
#                 print(f"✓ Found SRDF file inside directory: {srdf_path}")
#             else:
#                 print(f"✗ No SRDF files found in directory")
#                 srdf_content = ""
#         else:
#             # It's a file
#             try:
#                 with open(srdf_path, 'r') as f:
#                     srdf_content = f.read()
#                 print(f"✓ Loaded SRDF ({len(srdf_content)} chars)")
#             except Exception as e:
#                 print(f"✗ Failed to read SRDF: {e}")
#                 srdf_content = ""
#     else:
#         print(f"✗ SRDF not found at {srdf_path}")
#         srdf_content = ""
    
#     # Load other configs with error handling
#     def load_yaml_config(path, name):
#         if os.path.exists(path):
#             try:
#                 with open(path, 'r') as f:
#                     content = yaml.safe_load(f)
#                 print(f"✓ Loaded {name} from {path}")
#                 return content
#             except Exception as e:
#                 print(f"✗ Failed to load {name}: {e}")
#                 return {}
#         else:
#             print(f"✗ {name} not found at {path}")
#             return {}
    
#     kinematics_path = os.path.join(moveit_servo_dir, "config", "kinematics.yaml")
#     kinematics_content = load_yaml_config(kinematics_path, "kinematics")
    
#     joint_limits_path = os.path.join(moveit_servo_dir, "config", "joint_limits.yaml")
#     joint_limits_content = load_yaml_config(joint_limits_path, "joint_limits")
    
#     servo_config_path = os.path.join(moveit_servo_dir, "config", "gen3_servo.yaml")
#     servo_params = load_yaml_config(servo_config_path, "servo_config")
    
#     # Configure
#     # if "moveit_servo" not in servo_params:
#     #     servo_params["moveit_servo"] = {}
#     # servo_params["moveit_servo"]["move_group_name"] = LaunchConfiguration("planning_group")
#     if "moveit_servo" not in servo_params:
#         servo_params["moveit_servo"] = {}
#     servo_params["moveit_servo"]["command_in_type"] = "unitless" # speed_units
#     servo_params["moveit_servo"]["command_out_type"] = "trajectory_msgs/JointTrajectory"
#     servo_params["moveit_servo"]["command_out_topic"] = "/joint_trajectory_controller/joint_trajectory"
#     servo_params["moveit_servo"]["planning_frame"] = "base_link"
#     servo_params["moveit_servo"]["robot_link_command_frame"] = "grasping_frame"
#     servo_params["moveit_servo"]["move_group_name"] = LaunchConfiguration("planning_group") # manipulator does not exist

# # Topics (they might have defaults, but set them explicitly)
#     servo_params["moveit_servo"]["cartesian_command_in_topic"] = "/servo_node/delta_twist_cmds"
#     servo_params["moveit_servo"]["joint_command_in_topic"] = "/servo_node/delta_joint_cmds"
#     #### 
#     servo_params["moveit_servo"]["publish_joint_velocities"] = False # test 2

#     servo_params["moveit_servo"]["joint_topic"] = "/joint_states"
#     servo_params["moveit_servo"]["status_topic"] = "~/status"

#     servo_params["moveit_servo"]["lower_singularity_threshold"] = 8.0

#     servo_params["moveit_servo"]["hard_stop_singularity_threshold"] = 80.0 # lower =lesss strict  #30.0
#     servo_params["moveit_servo"]["leaving_singularity_threshold_multiplier"] = 50.0  # reduce sensitivity by increase # 5.0

#     # Collision
#     servo_params["moveit_servo"]["check_collisions"] = True
#     servo_params["moveit_servo"]["collision_check_rate"] = 10.0
#     servo_params["moveit_servo"]["self_collision_proximity_threshold"] = 0.01
#     servo_params["moveit_servo"]["scene_collision_proximity_threshold"] = 0.02

#     # Safety
#     servo_params["moveit_servo"]["halt_all_joints_in_joint_mode"] = False
#     servo_params["moveit_servo"]["halt_all_joints_in_cartesian_mode"] = False
#     servo_params["moveit_servo"]["override_velocity_scaling_factor"] = 0.8



#     # Check if we have minimal content
#     if not urdf_content:
#         print("✗ WARNING: Empty URDF content!")
    
#     # Parameters
#     all_params = [
#         {"robot_description": urdf_content},
#         {"robot_description_semantic": srdf_content},
#         {"robot_description_kinematics": kinematics_content},
#         {"robot_description_planning": joint_limits_content},
#         servo_params,
#         {"use_sim_time": LaunchConfiguration("use_sim_time")},
#         os.path.join(moveit_servo_dir, "config", "gen3_servo.yaml"),
#     ]
    
#     # Servo node
#     servo_node = Node(
#         package="moveit_servo",
#         executable="servo_node",
#         name="servo_node",
#         parameters=all_params,
#         output="screen",
#         emulate_tty=True,
#     )
    
#     return launch.LaunchDescription(declared_arguments + [servo_node])


# #################################GOKUL ABOVE#####################

import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import subprocess


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    # -----------------------------
    # URDF (XACRO)
    # -----------------------------
    xacro_path = "/root/ws_moveit/src/moveit2/moveit_ros/moveit_servo/kortex/ros2_ws/src/iki_kortex_description/urdf_xacro/kortex_standalone.urdf.xacro"

    if not os.path.exists(xacro_path):
        print(f"✗ XACRO not found: {xacro_path}")
        urdf_content = ""
    else:
        cmd = [
            "xacro",
            xacro_path,
            "name:=gen3",
            "arm:=gen3",
            "dof:=7",
            "gripper:=robotiq_2f_140",
            "use_fake_hardware:=true",
            "sim_gazebo:=true",
            "vision:=false",
            "moveit_active:=true",
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode == 0:
            urdf_content = result.stdout
            print("✓ URDF generated")
        else:
            print(f"✗ XACRO failed:\n{result.stderr}")
            urdf_content = ""

    # -----------------------------
    # MoveIt config (CORRECT)
    # -----------------------------
    moveit_config_dir = get_package_share_directory(
        "kinova_gen3_7dof_robotiq_2f_85_moveit_config"
    )

    def load_file(path):
        try:
            with open(path, "r") as f:
                return f.read()
        except:
            print(f"✗ Failed to load {path}")
            return ""

    def load_yaml(path):
        try:
            with open(path, "r") as f:
                return yaml.safe_load(f)
        except:
            print(f"✗ Failed to load {path}")
            return {}

    srdf_content = load_file(os.path.join(moveit_config_dir, "config", "gen3.srdf"))
    kinematics_content = load_yaml(os.path.join(moveit_config_dir, "config", "kinematics.yaml"))
    joint_limits_content = load_yaml(os.path.join(moveit_config_dir, "config", "joint_limits.yaml"))

    # -----------------------------
    # FULL SERVO CONFIG (from your YAML, fixed)
    # -----------------------------
    servo_params = {
        "moveit_servo": {

            "publish_period": 0.01,
            "max_expected_latency": 0.1,
            "command_in_type": "unitless",

            "scale": {
                "linear": 0.3,
                "rotational": 0.6,
                "joint": 0.01,
            },

            "publish_joint_positions": True,
            "publish_joint_velocities": True,
            "publish_joint_accelerations": False,

            "use_smoothing": True,
            "smoothing_filter_plugin_name": "online_signal_smoothing::ButterworthFilterPlugin",

            "is_primary_planning_scene_monitor": True,
            "check_octomap_collisions": False,

            # ✅ FIXED
            "move_group_name": "manipulator",
            "planning_frame": "base_link",
            "robot_link_command_frame": "grasping_frame",

            # Singularity
            "lower_singularity_threshold": 10.0,
            "hard_stop_singularity_threshold": 60.0,
            "leaving_singularity_threshold_multiplier": 20.0,

            # Joint limits
            "joint_limit_margins": [0.1, 0.15, 0.2, 0.15, 0.15, 0.1, 0.1],

            # Topics
            "cartesian_command_in_topic": "/servo_node/delta_twist_cmds",
            "joint_command_in_topic": "/servo_node/delta_joint_cmds",
            "joint_topic": "/joint_states",
            "status_topic": "~/status",

            "command_out_type": "trajectory_msgs/JointTrajectory",
            "command_out_topic": "/joint_trajectory_controller/joint_trajectory",

            # Collision
            "check_collisions": True,
            "collision_check_rate": 10.0,
            "self_collision_proximity_threshold": 0.01,
            "scene_collision_proximity_threshold": 0.02,

            # Safety
            "halt_all_joints_in_joint_mode": False,
            "halt_all_joints_in_cartesian_mode": False,
            "override_velocity_scaling_factor": 0.8,
        }
    }

    # -----------------------------
    # Parameters
    # -----------------------------
    all_params = [
        {"robot_description": urdf_content},
        {"robot_description_semantic": srdf_content},
        {"robot_description_kinematics": kinematics_content},
        {"robot_description_planning": joint_limits_content},
        servo_params,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    # -----------------------------
    # Servo Node
    # -----------------------------
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=all_params,
        output="screen",
        emulate_tty=True,
    )

    return launch.LaunchDescription(declared_arguments + [servo_node])
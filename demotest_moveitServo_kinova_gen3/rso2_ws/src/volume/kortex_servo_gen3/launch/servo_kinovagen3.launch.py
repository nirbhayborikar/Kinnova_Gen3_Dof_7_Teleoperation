

######################GOKUL#################
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
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "planning_group",
            default_value="manipulator",
        ),
    ]
    
    # **HARDCODED PATH - SIMPLEST**
    xacro_path = "/root/ws_moveit/src/moveit2/moveit_ros/moveit_servo/kortex/ros2_ws/src/iki_kortex_description/urdf_xacro/kortex_standalone.urdf.xacro"
    
    print(f"Looking for xacro file: {xacro_path}")
    
    # Check if file exists
    if not os.path.exists(xacro_path):
        print(f"✗ XACRO file not found at {xacro_path}")
        urdf_content = ""
    else:
        print(f"✓ Found xacro file")
        # Process xacro with error handling
        # xacro_command = [
        #     'xacro',
        #     xacro_path,
        #     'arm:=gen3',
        #     'dof:=7',
        #     'gripper:=robotiq_2f_140',
        #     'use_fake_hardware:=true',
        #     'sim_gazebo:=false',
        #     '--inorder'
        # ]
        xacro_command = [
            'xacro',
            xacro_path,
            'name:=gen3',
            'arm:=gen3',
            'dof:=7',
            'gripper:=robotiq_2f_140',
            'use_fake_hardware:=true',
            'sim_gazebo:=true',
            'sim_isaac:=false',  # Add this
            'vision:=false',     # Add this
            'moveit_active:=true'  # Might be needed
        ]
        
        print(f"Running: {' '.join(xacro_command)}")
        
        try:
            # First, let's check if xacro works at all
            test_result = subprocess.run(['which', 'xacro'], capture_output=True, text=True)
            if test_result.returncode != 0:
                print("✗ xacro command not found. Install with: sudo apt install ros-${ROS_DISTRO}-xacro")
            
            result = subprocess.run(
                xacro_command,
                capture_output=True,
                text=True,
                check=False  # Don't raise exception
            )
            
            if result.returncode == 0:
                urdf_content = result.stdout
                print(f"✓ XACRO processed successfully ({len(urdf_content)} chars)")
                # Save for debugging
                with open('/tmp/debug_urdf.xml', 'w') as f:
                    f.write(urdf_content)
                print(f"✓ Saved to /tmp/debug_urdf.xml")
            else:
                print(f"✗ XACRO failed with error: {result.stderr}")
                # Try without some arguments
                print("Trying simpler xacro command...")
                simple_command = ['xacro', xacro_path, '--inorder']
                simple_result = subprocess.run(simple_command, capture_output=True, text=True)
                if simple_result.returncode == 0:
                    urdf_content = simple_result.stdout
                    print(f"✓ Simple xacro worked ({len(urdf_content)} chars)")
                else:
                    print(f"✗ Simple xacro also failed: {simple_result.stderr}")
                    urdf_content = ""
                    
        except Exception as e:
            print(f"✗ Exception during xacro: {e}")
            urdf_content = ""
    
    # Get moveit_servo package directory
    moveit_servo_dir = get_package_share_directory("moveit_servo")
    print(f"MoveIt Servo directory: {moveit_servo_dir}")
    
    # **FIX SRDF PATH - check if it's a file or directory**
    srdf_path = os.path.join(moveit_servo_dir, "config", "kortex.srdf")
    print(f"Looking for SRDF at: {srdf_path}")
    
    if os.path.exists(srdf_path):
        if os.path.isdir(srdf_path):
            print(f"✗ SRDF path is a directory, not a file")
            # Maybe there's a file inside the directory
            srdf_files = [f for f in os.listdir(srdf_path) if f.endswith('.srdf')]
            if srdf_files:
                srdf_path = os.path.join(srdf_path, srdf_files[0])
                print(f"✓ Found SRDF file inside directory: {srdf_path}")
            else:
                print(f"✗ No SRDF files found in directory")
                srdf_content = ""
        else:
            # It's a file
            try:
                with open(srdf_path, 'r') as f:
                    srdf_content = f.read()
                print(f"✓ Loaded SRDF ({len(srdf_content)} chars)")
            except Exception as e:
                print(f"✗ Failed to read SRDF: {e}")
                srdf_content = ""
    else:
        print(f"✗ SRDF not found at {srdf_path}")
        srdf_content = ""
    
    # Load other configs with error handling
    def load_yaml_config(path, name):
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    content = yaml.safe_load(f)
                print(f"✓ Loaded {name} from {path}")
                return content
            except Exception as e:
                print(f"✗ Failed to load {name}: {e}")
                return {}
        else:
            print(f"✗ {name} not found at {path}")
            return {}
    
    kinematics_path = os.path.join(moveit_servo_dir, "config", "kinematics.yaml")
    kinematics_content = load_yaml_config(kinematics_path, "kinematics")
    
    joint_limits_path = os.path.join(moveit_servo_dir, "config", "joint_limits.yaml")
    joint_limits_content = load_yaml_config(joint_limits_path, "joint_limits")
    
    servo_config_path = os.path.join(moveit_servo_dir, "config", "gen3_servo.yaml")
    servo_params = load_yaml_config(servo_config_path, "servo_config")
    
    # Configure
    # if "moveit_servo" not in servo_params:
    #     servo_params["moveit_servo"] = {}
    # servo_params["moveit_servo"]["move_group_name"] = LaunchConfiguration("planning_group")
    if "moveit_servo" not in servo_params:
        servo_params["moveit_servo"] = {}
    servo_params["moveit_servo"]["command_in_type"] = "speed_units"
    servo_params["moveit_servo"]["command_out_type"] = "trajectory_msgs/JointTrajectory"
    servo_params["moveit_servo"]["command_out_topic"] = "/joint_trajectory_controller/joint_trajectory"
    servo_params["moveit_servo"]["planning_frame"] = "base_link"
    servo_params["moveit_servo"]["robot_link_command_frame"] = "grasping_frame"
    servo_params["moveit_servo"]["move_group_name"] = LaunchConfiguration("planning_group")

# Topics (they might have defaults, but set them explicitly)
    servo_params["moveit_servo"]["cartesian_command_in_topic"] = "servo_node/delta_twist_cmds"
    servo_params["moveit_servo"]["joint_command_in_topic"] = "servo_node/delta_joint_cmds"
    # Check if we have minimal content
    if not urdf_content:
        print("✗ WARNING: Empty URDF content!")
    
    # Parameters
    all_params = [
        {"robot_description": urdf_content},
        {"robot_description_semantic": srdf_content},
        {"robot_description_kinematics": kinematics_content},
        {"robot_description_planning": joint_limits_content},
        servo_params,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
        os.path.join(moveit_servo_dir, "config", "gen3_servo.yaml"),
    ]
    
    # Servo node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=all_params,
        output="screen",
        emulate_tty=True,
    )
    
    return launch.LaunchDescription(declared_arguments + [servo_node])


#################################GOKUL ABOVE#####################
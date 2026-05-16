# Dual Arm 6DOF Teleoperation — Webcam-Based (xArm6, UFACTORY)

Real-time teleoperation of dual xArm6 robotic arms using hand gesture tracking via MediaPipe and ROS2 Humble. The operator controls both arms simultaneously using a standard webcam no depth camera required.

## 🎬 Demo

![Demo](media/working_demo.gif)

![Full_Video_On_Github_High_Quality] (https://github.com/user-attachments/assets/a066718c-373d-490d-a892-551655c1226c)

> If video doesn't load above, [watch on Google Drive](https://drive.google.com/drive/folders/1xSlDL9iWierv9Z4JuFxI9-DaDwG45fTa?usp=sharing)
---

![System Architecture](media/images/dual_xarm6_pipeline_flowchart.png)

## ✨ Features

- **Dual 6-DOF arm control** with two-finger grippers (xArm6, UFACTORY)
- **Real-time hand tracking** using MediaPipe Hands (30Hz camera loop)
- **2D Movement + 1D Rotation** — end-effector moves in YZ plane, rotates about Z-axis
- **Gesture-based gripper control** — pinch to close, open hand to open
- **3-stage noise filtering** — dead zone, velocity clamping, EMA smoothing
- **Collision avoidance** — MoveIt2 IK with `avoid_collisions: true` (SRDF collision matrix)
- **Decoupled architecture** — camera node never blocks, IK runs in background threads
- **Docker-based setup** — fully reproducible environment with one command
- **Joint angle publishing** — current joint values published via `/joint_states` at ~100Hz

---

## 📋 Prerequisites

- **OS**: Ubuntu 22.04 (X11 display server, not Wayland)
- **Hardware**: Computer webcam or USB webcam
- **Software**: Docker & Docker Compose

### Check your display server
```bash
echo $XDG_SESSION_TYPE
# Must output "x11" — this project requires X11 for GUI forwarding
```

---

## 🚀 Setup & Installation  Docker Recommended

### Step 1: Install Docker

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y ca-certificates curl gnupg lsb-release

sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
  sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Verify installation
sudo docker run hello-world
```

### Step 2: Allow GUI Access for Docker

```bash
echo 'xhost +local:docker > /dev/null 2>&1' >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Clone Repository

```bash
git clone https://github.com/nirbhayborikar/Dual_Arm_Teleoperation_6DOF_two_finger_grippers.git
cd Dual_Arm_Teleoperation_6DOF_two_finger_grippers
```

### Step 4: Build & Run

```bash
cd ros2_ws/src/docker
docker compose -f docker-compose.yml up
```

This single command will:
1. Build the Docker image (ROS2 Humble + MoveIt2 + xarm_ros2 + MediaPipe)
2. Launch dual xArm6 with grippers in RViz2 via MoveIt2
3. Start the teleoperation camera node and arm controller node
4. Open the camera window with hand tracking overlay

> **Note**: First build takes ~15-20 minutes. Subsequent starts are instant.

### Verify GUI Forwarding

If RViz doesn't appear, check that X11 socket exists:
```bash
ls /tmp/.X11-unix
# Should show at least one file like X0
```

---

---

## 🖥️ System Setup Host Machine (Optional)

### 🐳 Docker (Recommended — One Command Setup) for that read above

```bash
cd ros2_ws/src/docker
docker compose -f docker-compose.yml up
```

This builds the image and launches everything automatically. Skip to [Operating Instructions](#-operating-instructions).

### 🐧 Native Installation (Without Docker)

If you prefer to run natively on Ubuntu 22.04 with ROS2 Humble:

**1. Install ROS2 Humble** ([official guide](https://docs.ros.org/en/humble/Installation.html))

**2. Install MoveIt2, controllers, and system dependencies:**

```bash
sudo apt update && sudo apt install -y \
  build-essential cmake git wget python3-pip \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-humble-moveit \
  ros-humble-moveit-servo \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-gripper-controllers \
  ros-humble-rviz2 \
  ros-humble-tf2-tools \
  ros-humble-tf-transformations \
  ros-humble-generate-parameter-library \
  libv4l-dev v4l-utils mesa-utils libgl1-mesa-glx libgl1-mesa-dri
```

**3. Install Python dependencies:**

```bash
pip3 install numpy==1.24.4 matplotlib==3.7.5 mediapipe==0.10.14 \
  opencv-python-headless scipy transforms3d
```

**4. Create workspace and clone xarm_ros2:**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone xarm_ros2 (dual arm URDF, MoveIt configs, controllers)
git clone --branch humble --depth 1 https://github.com/xArm-Developer/xarm_ros2.git
cd xarm_ros2 && git submodule update --init --recursive && cd ..

# Clone this project
git clone https://github.com/nirbhayborikar/Dual_Arm_Teleoperation_6DOF_two_finger_grippers.git
cp -r Dual_Arm_Teleoperation_6DOF_two_finger_grippers/ros2_ws/src/stage_1/dual_xarm6_teleop .
```

**5. Install ROS dependencies and build:**

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**6. Launch the system (3 terminals):**

```bash
# Terminal 1: Launch dual xArm6 + MoveIt2 + RViz2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch xarm_moveit_config dual_xarm6_moveit_fake.launch.py add_gripper:=true

# Terminal 2: Launch arm controller (IK + trajectory publishing)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run dual_xarm6_teleop arm_controller

# Terminal 3: Launch teleop node (camera + hand tracking)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch dual_xarm6_teleop teleop.launch.py
```

> **Note:** The Docker setup runs all three in one container automatically via docker-compose.yml command section.

---

## 🎮 Operating Instructions

Sit facing the camera. Show both hands — you'll see landmark points and connections drawn on your hands in the camera window.

### Controls

| Gesture | Action |
|---------|--------|
| Open hand | Open gripper |
| Pinch thumb + index finger | Close gripper |
| Close fist | Close gripper |
| Move hand left/right | Arm moves laterally (Y-axis) |
| Move hand up/down | Arm moves vertically (Z-axis) |
| Move hand toward/away from camera | Arm moves forward/backward (X-axis) |
| Rotate wrist | End-effector rotates about Z-axis (±45°) |
| Press Q in camera window | Quit |

### Visual Guides

| Action | Image |
|--------|-------|
| Both arms active | ![Both arms](media/images/Gripper_Open.png) |
| Gripper open | ![Open](media/images/Gripper_Open.png) |
| Gripper close (pinch) | ![Pinch](media/images/Gripper_close_with_Pinch.png) |
| Gripper close (fist) | ![Fist](media/images/Gripper_close_with_fist.png) |
| Arms moving apart (Y-direction) | ![Apart](media/images/away_position_moving_away_arm.png) |
| Arms moving together (Y-direction) | ![Together](media/images/going_towards.png) |
| Rotate end-effector | ![Rotate](media/images/Rotate_in_x_axis.png) |

### Joint Angle Logging

Joint angles are logged in real-time to the terminal where Docker Compose was started:

![Joint Logging](media/images/joint_Angles_info.png)

---

## 🏗️ System Architecture

### Pipeline Overview

```
Webcam (30fps)
    │
    ▼
teleop_node.py ─── hand_tracker.py (MediaPipe Hands)
    │                   • Detect & classify left/right hands
    │                   • Extract wrist position, rotation, pinch distance
    │
    ├── smoother.py (3-Stage Noise Filter)
    │       • Dead Zone: Δ < 0.005 → ignored
    │       • Velocity Clamp: Δ > 0.08/frame → clamped
    │       • EMA Smoothing: smoothed = prev + 0.4 × (raw - prev)
    │
    ├── Workspace Mapping (_map_to_pose)
    │       • Hand X → Robot Y (left arm offset Y=0.0, right arm offset Y=1.0)
    │       • Hand Y → Robot Z [0.1m – 0.5m]
    │       • Robot X = fixed 0.35m (2D mode, no depth camera)
    │       • Hand rotation → EE rotation ±45°
    │
    ▼
ROS2 Topics: /left_hand_target_pose, /right_hand_target_pose (PoseStamped)
             /left_hand_gripper_control, /right_hand_gripper_control (Bool)
    │
    ▼
arm_controller_node.py
    │   • Subscribe to target poses
    │   • Call /compute_ik (MoveIt2) in background threads
    │   • avoid_collisions: true (SRDF collision matrix)
    │   • Seed IK with current joint state (prevents elbow flips)
    │   • Publish JointTrajectory (200ms duration)
    │
    ▼
ros2_control (Trajectory Controllers)
    │   • L_xarm6_traj_controller → left arm joints 1-6
    │   • R_xarm6_traj_controller → right arm joints 1-6
    │   • L/R_xarm_gripper_traj_controller → gripper open/close
    │   • joint_state_broadcaster → /joint_states (feedback loop to IK seed)
    │
    ▼
RViz2 (Dual xArm6 Visualization)
    • Left arm base: Y = 0.0m
    • Right arm base: Y = 1.0m
```

### Node Details

**1. teleop_node.py** — Hand Pose Publisher ([source](https://github.com/nirbhayborikar/Dual_Arm_Teleoperation_6DOF_two_finger_grippers/blob/main/ros2_ws/src/stage_1/dual_xarm6_teleop/dual_xarm6_teleop/teleop_node.py))

Captures webcam feed, tracks both hands using MediaPipe, applies 3-stage noise filtering (dead zone → velocity clamping → EMA smoothing), maps hand positions to robot workspace with per-arm Y offset, detects pinch gesture for gripper control, and publishes target poses and gripper commands at 30Hz. This node never blocks — it runs entirely independently from the IK computation.

**2. arm_controller_node.py** — Arm Controller ([source](https://github.com/nirbhayborikar/Dual_Arm_Teleoperation_6DOF_two_finger_grippers/blob/main/ros2_ws/src/stage_1/dual_xarm6_teleop/dual_xarm6_teleop/arm_controller_node.py))

Subscribes to hand target poses, calls MoveIt2's `/compute_ik` service in background threads (one per arm) for collision-free IK solving, seeds the solver with current joint state for smooth motion, and publishes `JointTrajectory` commands to both arm and gripper controllers simultaneously.

**3. hand_tracker.py** — Hand Tracking ([source](https://github.com/nirbhayborikar/Dual_Arm_Teleoperation_6DOF_two_finger_grippers/blob/main/ros2_ws/src/stage_1/dual_xarm6_teleop/dual_xarm6_teleop/hand_tracker.py))

Wraps MediaPipe Hands to detect and classify left/right hands, extracts wrist position, wrist rotation angle, and thumb-index pinch distance from hand landmarks.

**4. smoother.py** — Signal Smoothing ([source](https://github.com/nirbhayborikar/Dual_Arm_Teleoperation_6DOF_two_finger_grippers/blob/main/ros2_ws/src/stage_1/dual_xarm6_teleop/dual_xarm6_teleop/smoother.py))

Implements Exponential Moving Average smoothing with dead zone filtering and velocity clamping to handle noisy MediaPipe output and prevent sudden robot jerks.

**5. Collision Avoidance**

Every IK request sets `avoid_collisions: true`, using the pre-configured [SRDF collision matrix](https://github.com/xArm-Developer/xarm_ros2/blob/humble/xarm_moveit_config/srdf/_xarm6_macro.srdf.xacro) from xarm_ros2 to prevent self-collision and dual-arm collision.

---

## 📡 ROS2 Topics

| Topic | Type | Published By | Subscribed By | Purpose |
|-------|------|-------------|---------------|---------|
| `/left_hand_target_pose` | PoseStamped | teleop_node | arm_controller | Left arm EE target |
| `/right_hand_target_pose` | PoseStamped | teleop_node | arm_controller | Right arm EE target |
| `/left_hand_gripper_control` | Bool | teleop_node | arm_controller | Left gripper open/close |
| `/right_hand_gripper_control` | Bool | teleop_node | arm_controller | Right gripper open/close |
| `/L_xarm6_traj_controller/joint_trajectory` | JointTrajectory | arm_controller | ros2_control | Left arm joint commands |
| `/R_xarm6_traj_controller/joint_trajectory` | JointTrajectory | arm_controller | ros2_control | Right arm joint commands |
| `/joint_states` | JointState | joint_state_broadcaster | arm_controller | Current joint angles (IK seed + logging) |

---

## 🔧 Tunable Parameters

All parameters are in [`config/teleop_params.yaml`](ros2_ws/src/stage_1/dual_xarm6_teleop/config/teleop_params.yaml):

| Parameter | Default | Effect |
|-----------|---------|--------|
| `alpha` | 0.4 | EMA smoothing (higher = more responsive, lower = smoother) |
| `dead_zone` | 0.005 | Ignore movements below this threshold |
| `max_vel` | 0.08 | Max position change per frame (safety limit) |
| `pinch_threshold` | 0.07 | Thumb-index distance to trigger gripper close |
| `robot_y_min/max` | ±0.3 | Lateral workspace range (meters) |
| `robot_z_min/max` | 0.1/0.5 | Vertical workspace range (meters) |
| `max_rotation` | 0.785 | Max EE rotation ±45° (radians) |

---

## 🧠 Design Decisions

### Why IK Service Calls Instead of MoveIt Servo?

We use: **Target Pose → /compute_ik → Joint Angles → JointTrajectory → Controller**

- **No path planning** — we don't use OMPL, RRT, PRM or any motion planner
- **No MoveIt Servo** — we don't stream twist/velocity commands
- **Only MoveIt for IK** — given a target EE pose, MoveIt solves inverse kinematics and returns joint angles
- **`avoid_collisions: true`** — MoveIt checks if the target pose is in collision and rejects it if so, but does NOT plan a collision-free path
- **JointTrajectory with 200ms duration** — target joint angles are sent directly to the trajectory controller, which interpolates linearly

**Why this works for teleoperation:**
- Movements are small and incremental (hand moves smoothly at 30Hz, IK runs at ~10Hz)
- Each new target is very close to the current position (small Δ)
- Small movements = low collision risk during the interpolated motion
- IK seeding with current joint state keeps solutions near the current configuration
- The 3-stage smoothing pipeline ensures no large sudden jumps

**Trade-off:** If the operator makes a very large sudden movement, the interpolated path could theoretically pass through collision. The velocity clamping and EMA smoothing mitigate this by keeping movements small.

### Noise Handling

The assignment states *"your solution should include measures to handle noisy results."* This is addressed by:

1. **Dead Zone Filter** — eliminates sensor noise when hand is stationary
2. **Velocity Clamping** — rejects sudden tracking glitches and limits max arm speed for safety
3. **EMA Smoothing** — produces stable, continuous motion from noisy discrete measurements
4. **IK Seeding** — uses current joint state to find nearest IK solution, preventing sudden configuration jumps
5. **Rate Limiting** — drops excess poses when IK is busy, preventing command queue buildup

### Safety Considerations

- `avoid_collisions: true` in every IK request prevents the two arms from colliding
- Velocity clamping ensures the real robot would never receive a dangerously fast command
- Single operator control: one camera, two hands, two arms — no ambiguity
- Hand timeout: if a hand is lost for 2 seconds, that arm stops receiving commands
- IK timeout: if planning takes longer than 300ms, the pose is skipped rather than queued
- Designed for real-world: trajectory controllers interpolate smoothly over 200ms, compatible with real robot hardware

---

## 📁 Repository Structure

```
Dual_Arm_Teleoperation_6DOF_two_finger_grippers/
├── README.md
├── task.md
├── media/                              # Screenshots and diagrams
│   ├── dual_xarm6_pipeline_flowchart.png
│   ├── Gripper_Open.png
│   ├── Gripper_close_with_Pinch.png
│   └── ...
└── ros2_ws/
    └── src/
        ├── docker/
        │   ├── Dockerfile              # ROS2 Humble + MoveIt2 + xarm_ros2 + MediaPipe
        │   ├── docker-compose.yml      # One-command build & run
        │   └── entrypoint.sh           # Auto-sources ROS2 workspace
        └── stage_1/
            └── dual_xarm6_teleop/      # Custom ROS2 Python package
                ├── config/
                │   └── teleop_params.yaml
                ├── dual_xarm6_teleop/
                │   ├── __init__.py
                │   ├── teleop_node.py          # Camera + MediaPipe + publish poses
                │   ├── arm_controller_node.py  # IK + trajectory publishing
                │   ├── hand_tracker.py         # MediaPipe hand detection
                │   └── smoother.py             # EMA + dead zone + velocity clamp
                ├── launch/
                │   └── teleop.launch.py        # Launches both nodes
                ├── package.xml
                ├── setup.py
                └── setup.cfg
```

---

## 📚 References & Open-Source Acknowledgments

| Repository | Usage |
|------------|-------|
| [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) | Dual xArm6 URDF, MoveIt2 configs, SRDF collision matrix, ros2_control setup, RViz launch files |
| [Google MediaPipe](https://github.com/google/mediapipe) | Hand landmark detection and tracking (MediaPipe Hands solution) |
| [MoveIt2](https://github.com/moveit/moveit2) | Inverse Kinematics solver (`/compute_ik` service), collision checking |


---

## 🚀 Potential Improvements

- **Monocular depth estimation** — add depth axis control without requiring a depth camera
- **Gesture-based start/stop** — use specific hand gestures to enable/disable arm control
- **Workspace visualization** — overlay reachable workspace boundaries on the camera feed
- **Body pose tracking** — use MediaPipe Pose for full upper-body teleoperation

---

## 📄 License

Apache 2.0
# Teleoperation of Robotic Arm (7DOF with Robotiq Gripper) AprilTag-Based 
Real-time teleoperation of a **Kinova Gen3 (7-DOF)** robotic arm using
**AprilTag tracking**, **MediaPipe hand gestures**, and **ROS2 Jazzy**.

In this project report, we have explored algorithms for 3D spatial tracking and robotic teleoperation and tested with 96 random people to study their hand movement and how they will teleoperate. We utilized a Kinova Gen3 robotic arm and a standard RGB camera setup to track AprilTag fiducial markers, along with the following associated control modules: the state estimation module, which extracts the absolute 3D Cartesian coordinates, orientation vectors (e.g., Roll, Pitch, Yaw), and the real-time calculated velocities of the operator's hand movements; and the ROS 2 filtering pipeline, which provides the exponential moving average (EMA) algorithms and deadzone thresholds required for smooth, Gimbal Lock-free kinematic control.

## ✨ Features

-   Kinova Gen3 (7-DOF) teleoperation
-   AprilTag-based workspace tracking
-   MediaPipe hand gesture recognition
-   ROS2 Jazzy + Zenoh DDS communication
-   Containerized deployment using Docker Compose
-   Astra camera + webcam support
-   Modular multi-container architecture

## 📋 Prerequisites

-   Ubuntu 22.04/24.04
-   Docker + Docker Compose
-   X11 display server
-   Astra camera or webcam
-   Kinova Gen3 robot 

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
git clone https://github.com/nirbhayborikar/Kinnova_Gen3_Dof_7_Teleoperation.git
```

### Step 4: Build & Run

```bash
cd Kinnova_Gen3_Dof_7_Teleoperation/kinnova_gen3_7_DOF_webcam_teleop/ros2_ws/src/docker
docker compose -f teleop_astra_marker.yml up
```

This single command will:
1. Build the Docker image (ROS2 Jazzy + MoveIt2 + MediaPipe)
2. Launch the kortex arm from kortex_demo folder (for this contact University)
3. Connect the astra camera
4. Open the camera window with hand tracking overlay

## 🏗 Architecture


![System Architecture](media/images/workflow.png)

Astra Camera → AprilTag Detection → Gesture Tracking → Workspace Mapping
→ Kinova Teleop → Kinova Gen3

## 📡 ROS Topics

-   /camera/color/image_raw
-   /camera/color/camera_info
-   /tag_detections
-   /tf
-   /joint_states

## 📁 Repository Structure

``` text
kinnova_gen3_7_DOF_webcam_teleop/
├── ros2_ws
├── apriltag
├── astra
└── README.md
```

## 🚀 Future Improvements

-   Dynamic workspace calibration
-   Multiple AprilTag support
-   Depth integration
-   Safety zones
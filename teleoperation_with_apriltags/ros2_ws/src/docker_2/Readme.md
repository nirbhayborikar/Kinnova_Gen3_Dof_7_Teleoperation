# Kinova Gen3 — AprilTag Visual Servoing (Docker)

ROS 2 Jazzy · rmw_zenoh_cpp · Ubuntu 24.04

---

## Directory structure

```
kinova-visual-servoing/
├── Dockerfile
├── docker-compose.yml
├── .env                          ← edit ROBOT_IP here
├── config/
│   └── apriltag_params.yaml      ← tag family + physical size
├── scripts/
│   ├── entrypoint.sh
│   └── start_follower.sh
└── src/
    └── apriltag_follower/        ← your custom ROS2 package
        ├── package.xml
        ├── setup.py
        ├── resource/apriltag_follower
        └── apriltag_follower/
            ├── __init__.py
            └── follower_node.py
```

---

## Prerequisites

```bash
# Docker + compose plugin
sudo apt install docker.io docker-compose-plugin
sudo usermod -aG docker $USER   # then re-login

# Allow X11 from Docker containers (for RViz / image_view)
xhost +local:docker
```

---

## Quick start

```bash
# 1. Edit .env — set your robot IP
nano .env     # change ROBOT_IP=192.168.1.10

# 2. Edit config/apriltag_params.yaml — set the physical tag size
#    Measure the black border edge-to-edge in metres

# 3. Build the image (takes 10-20 min first time)
docker compose build

# 4. Start everything
docker compose up

# 5. Watch logs per service
docker compose logs -f follower_node
docker compose logs -f apriltag_node
docker compose logs -f kinova_vision
```

---

## With debug visualisation (RViz + image view)

```bash
xhost +local:docker
docker compose --profile debug up
```

---

## Live parameter tuning (without rebuilding)

```bash
# From host — adjust gain on the running node
docker exec -it follower_node bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /opt/ros2_ws/install/setup.bash &&
  ros2 param set /apriltag_follower kp 1.2
"

# All tunable params:
#   kp             proportional gain       (start 0.5, increase slowly)
#   alpha          EMA filter weight       (lower = smoother, more lag)
#   max_vel        max linear speed m/s    (keep ≤ 0.10 initially)
#   deadband       minimum error to move   (0.01–0.02 m)
#   safe_offset    standoff from tag m     (≥ 0.10)
#   x/y/z_min/max  workspace box metres
```

---

## Verify camera and tag detection

```bash
# Check camera topics are flowing
docker exec -it kinova_vision bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /opt/ros2_ws/install/setup.bash &&
  ros2 topic hz /color/image_raw
"

# Check TF is publishing for the tag
docker exec -it apriltag_node bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /opt/ros2_ws/install/setup.bash &&
  ros2 run tf2_ros tf2_echo base_link 'tag36h11:0'
"
```

---

## Shutdown

```bash
docker compose down
```

The follower node sends a zero Twist on shutdown — the arm stops safely.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `kinova_vision` keeps retrying | Robot IP wrong in `.env`, or arm not powered |
| Tag TF not appearing | Check tag size in `apriltag_params.yaml`, ensure tag is in camera frame |
| Arm doesn't move | Verify `twist_controller` is active: `ros2 control list_controllers` |
| Arm oscillates | Reduce `kp` or reduce `alpha` (more smoothing) |
| Nodes can't find each other | Zenoh router not running — check `docker compose logs zenoh_router` |
| Camera stream fails | GStreamer decode error — try restarting `kinova_vision` service |

---

## Key packages & repos

| Package | URL |
|---|---|
| ros2_kortex | https://github.com/Kinovarobotics/ros2_kortex |
| ros2_kortex_vision | https://github.com/Kinovarobotics/ros2_kortex_vision |
| apriltag_ros | https://index.ros.org/p/apriltag_ros |
| rmw_zenoh | https://github.com/ros2/rmw_zenoh |
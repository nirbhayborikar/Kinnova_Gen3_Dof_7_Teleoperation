// Olive IMU Pro Teleoperation for Robotic Arm
// Subscribes to /tf from Olive IMU Pro, maps orientation to velocity commands

#include "olive_imu_teleop_node.hpp"
#include <cmath>

// ==================== CONSTRUCTOR ====================
OliveImuTeleopNode::OliveImuTeleopNode(const rclcpp::NodeOptions& options)
    : Node("olive_imu_teleop_node", options)
{
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "  Olive IMU Pro Teleoperation Node");
    RCLCPP_INFO(this->get_logger(), "========================================");

    loadParameters();
    setupROS();
    activateServoMode();

    RCLCPP_INFO(this->get_logger(), "Ready! Subscribing to /tf for frame: %s",
                imu_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Hold IMU steady - auto-calibrates on first message");
}

// ==================== DESTRUCTOR ====================
OliveImuTeleopNode::~OliveImuTeleopNode() {
    stopRobot();
    RCLCPP_INFO(this->get_logger(), "Shutdown complete");
}

// ==================== LOAD PARAMETERS ====================
void OliveImuTeleopNode::loadParameters() {
    // Frame ID to look for in /tf messages
    this->declare_parameter("imu_frame_id", "id001_sensor_link");

    this->declare_parameter("control_rate_hz", 50.0);
    this->declare_parameter("max_linear_velocity", 0.3);
    this->declare_parameter("max_angular_velocity", 0.8);
    this->declare_parameter("max_linear_accel", 1.0);
    this->declare_parameter("max_angular_accel", 2.0);
    this->declare_parameter("deadzone", 0.005);
    this->declare_parameter("smoothing_factor", 0.3);
    this->declare_parameter("timeout_ms", 500.0);
    this->declare_parameter("max_consecutive_timeouts", 3);
    this->declare_parameter("linear_scale", 0.8);
    this->declare_parameter("angular_scale", 1.5);
    this->declare_parameter("invert_x", false);
    this->declare_parameter("invert_y", true);
    this->declare_parameter("invert_z", true);

    imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
    control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
    max_linear_vel_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_velocity").as_double();
    max_linear_accel_ = this->get_parameter("max_linear_accel").as_double();
    max_angular_accel_ = this->get_parameter("max_angular_accel").as_double();
    deadzone_ = this->get_parameter("deadzone").as_double();
    smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();
    timeout_ms_ = this->get_parameter("timeout_ms").as_double();
    max_consecutive_timeouts_ = this->get_parameter("max_consecutive_timeouts").as_int();
    linear_scale_ = this->get_parameter("linear_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();
    invert_x_ = this->get_parameter("invert_x").as_bool();
    invert_y_ = this->get_parameter("invert_y").as_bool();
    invert_z_ = this->get_parameter("invert_z").as_bool();

    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  IMU Frame ID: %s", imu_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Control Rate: %.1f Hz", control_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "  Max Linear Vel: %.2f m/s", max_linear_vel_);
    RCLCPP_INFO(this->get_logger(), "  Max Angular Vel: %.2f rad/s", max_angular_vel_);
    RCLCPP_INFO(this->get_logger(), "  Deadzone: %.3f", deadzone_);
    RCLCPP_INFO(this->get_logger(), "  Smoothing: %.2f", smoothing_factor_);
}

// ==================== SETUP ROS ====================
void OliveImuTeleopNode::setupROS() {
    // Subscribe to /tf from Olive IMU Pro
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 10,
        std::bind(&OliveImuTeleopNode::tfCallback, this, std::placeholders::_1));

    // Publish twist commands to MoveIt Servo
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);

    // Servo mode service client
    servo_mode_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(
        "/servo_node/switch_command_type");

    // Control loop timer
    auto control_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / control_rate_hz_));
    control_timer_ = this->create_wall_timer(
        control_period,
        std::bind(&OliveImuTeleopNode::controlLoopCallback, this));

    // Diagnostics timer
    diagnostics_timer_ = this->create_wall_timer(
        2s,
        std::bind(&OliveImuTeleopNode::logStatus, this));

    RCLCPP_INFO(this->get_logger(), "ROS interfaces created");
}

// ==================== ACTIVATE SERVO MODE ====================
void OliveImuTeleopNode::activateServoMode() {
    while (!servo_mode_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for servo service");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for servo service...");
    }

    auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;

    auto result = servo_mode_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result)
        == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Servo activated in TWIST mode");
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to activate servo");
    }
}

// ==================== TF CALLBACK ====================
// This is where we receive orientation data from Olive IMU Pro
void OliveImuTeleopNode::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    for (const auto& transform : msg->transforms) {
        // Only process transforms from our Olive IMU
        if (transform.child_frame_id != imu_frame_id_) {
            continue;
        }

        std::lock_guard<std::mutex> lock(imu_mutex_);

        // Extract quaternion from /tf message
        current_orientation_.x() = transform.transform.rotation.x;
        current_orientation_.y() = transform.transform.rotation.y;
        current_orientation_.z() = transform.transform.rotation.z;
        current_orientation_.w() = transform.transform.rotation.w;
        current_orientation_.normalize();

        orientation_received_ = true;
        last_msg_time_ = std::chrono::steady_clock::now();
        msgs_received_++;

        // Auto-calibrate on first valid message
        if (!initial_orientation_set_) {
            initial_orientation_ = current_orientation_;
            initial_orientation_set_ = true;

            double roll, pitch, yaw;
            quaternionToEuler(initial_orientation_, roll, pitch, yaw);

            RCLCPP_INFO(this->get_logger(),
                        "Calibrated! Initial orientation: R=%.1f P=%.1f Y=%.1f deg",
                        roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
        }
    }
}

// ==================== QUATERNION TO EULER ====================
void OliveImuTeleopNode::quaternionToEuler(const Eigen::Quaterniond& q,
                                            double& roll, double& pitch, double& yaw) {
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    yaw = euler[0];
    pitch = euler[1];
    roll = euler[2];
}

// ==================== CONTROL LOOP ====================
void OliveImuTeleopNode::controlLoopCallback() {
    checkTimeout();

    VelocityCommand vel_cmd = processOrientation();

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->now();
    twist.header.frame_id = "base_link";
    twist.twist.linear.x = vel_cmd.linear.x();
    twist.twist.linear.y = vel_cmd.linear.y();
    twist.twist.linear.z = vel_cmd.linear.z();
    twist.twist.angular.x = vel_cmd.angular.x();
    twist.twist.angular.y = vel_cmd.angular.y();
    twist.twist.angular.z = vel_cmd.angular.z();

    publishTwist(twist);
    current_velocity_ = vel_cmd;
}

// ==================== PROCESS ORIENTATION ====================
VelocityCommand OliveImuTeleopNode::processOrientation() {
    VelocityCommand cmd;

    std::lock_guard<std::mutex> lock(imu_mutex_);

    if (!orientation_received_ || !initial_orientation_set_) {
        return cmd;
    }

    // ========== COMPUTE RELATIVE ORIENTATION ==========
    // How far has the IMU rotated from its starting position?
    Eigen::Quaterniond relative_quat = initial_orientation_.inverse() * current_orientation_;

    // Convert to Euler angles
    double roll, pitch, yaw;
    quaternionToEuler(relative_quat, roll, pitch, yaw);

    // ========== MAP ORIENTATION TO VELOCITY ==========
    // Pitch (tilt forward/back)  → X linear velocity
    // Roll  (tilt left/right)    → Y linear velocity
    // Yaw   (rotate flat)        → Z linear velocity (up/down)
    Eigen::Vector3d raw_linear(
        pitch * linear_scale_ * (invert_y_ ? -1 : 1),
        roll  * linear_scale_ * (invert_x_ ? -1 : 1),
        yaw   * linear_scale_ * (invert_z_ ? -1 : 1)
    );

    Eigen::Vector3d raw_angular(0.0, 0.0, 0.0);

    // ========== FILTERING PIPELINE ==========
    Eigen::Vector3d deadzone_linear = applyDeadzone(raw_linear);
    Eigen::Vector3d deadzone_angular = applyDeadzone(raw_angular);

    Eigen::Vector3d smoothed_linear = applySmoothing(current_velocity_.linear, deadzone_linear);
    Eigen::Vector3d smoothed_angular = applySmoothing(current_velocity_.angular, deadzone_angular);

    double control_dt = 1.0 / control_rate_hz_;
    Eigen::Vector3d limited_linear = limitAcceleration(
        current_velocity_.linear, smoothed_linear, control_dt);
    Eigen::Vector3d limited_angular = limitAcceleration(
        current_velocity_.angular, smoothed_angular, control_dt);

    Eigen::Vector3d final_linear = saturateVelocity(limited_linear, max_linear_vel_);
    Eigen::Vector3d final_angular = saturateVelocity(limited_angular, max_angular_vel_);

    // ========== CLAMP TINY VALUES TO ZERO ==========
    const double MIN_VELOCITY = 1e-6;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(final_linear[i]) < MIN_VELOCITY) final_linear[i] = 0.0;
        if (std::abs(final_angular[i]) < MIN_VELOCITY) final_angular[i] = 0.0;
    }

    // ========== DEBUG LOGGING ==========
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "Orient: R=%.1f P=%.1f Y=%.1f deg -> Vel: [%.3f, %.3f, %.3f]",
                          roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI,
                          final_linear.x(), final_linear.y(), final_linear.z());

    cmd.linear = final_linear;
    cmd.angular = final_angular;
    return cmd;
}

// ==================== FILTERING ====================
Eigen::Vector3d OliveImuTeleopNode::applyDeadzone(const Eigen::Vector3d& input) {
    Eigen::Vector3d output;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(input[i]) < deadzone_) {
            output[i] = 0.0;
        } else {
            double sign = (input[i] > 0) ? 1.0 : -1.0;
            output[i] = sign * (std::abs(input[i]) - deadzone_);
        }
    }
    return output;
}

Eigen::Vector3d OliveImuTeleopNode::applySmoothing(
    const Eigen::Vector3d& current, const Eigen::Vector3d& target) {
    double alpha = 1.0 - smoothing_factor_;
    return alpha * target + smoothing_factor_ * current;
}

Eigen::Vector3d OliveImuTeleopNode::limitAcceleration(
    const Eigen::Vector3d& current, const Eigen::Vector3d& target, double dt) {
    Eigen::Vector3d delta = target - current;
    double max_delta = max_linear_accel_ * dt;

    if (delta.norm() > max_delta) {
        delta = delta.normalized() * max_delta;
    }

    return current + delta;
}

Eigen::Vector3d OliveImuTeleopNode::saturateVelocity(
    const Eigen::Vector3d& vel, double max_vel) {
    double norm = vel.norm();
    if (norm > max_vel) {
        return vel * (max_vel / norm);
    }
    return vel;
}

// ==================== SAFETY ====================
void OliveImuTeleopNode::checkTimeout() {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double, std::milli>(
        now - last_msg_time_).count();

    if (elapsed > timeout_ms_) {
        consecutive_timeouts_++;

        if (consecutive_timeouts_ >= max_consecutive_timeouts_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Connection lost (%.0f ms) - stopping robot", elapsed);
            stopRobot();
        }
    } else {
        consecutive_timeouts_ = 0;
    }
}

void OliveImuTeleopNode::stopRobot() {
    publishTwist(createZeroTwist());
    current_velocity_.linear.setZero();
    current_velocity_.angular.setZero();
}

geometry_msgs::msg::TwistStamped OliveImuTeleopNode::createZeroTwist() {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->now();
    twist.header.frame_id = "base_link";
    return twist;
}

// ==================== PUBLISHING ====================
void OliveImuTeleopNode::publishTwist(const geometry_msgs::msg::TwistStamped& twist) {
    twist_pub_->publish(twist);
}

// ==================== DIAGNOSTICS ====================
void OliveImuTeleopNode::logStatus() {
    std::lock_guard<std::mutex> lock(imu_mutex_);

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    if (orientation_received_ && initial_orientation_set_) {
        Eigen::Quaterniond relative = initial_orientation_.inverse() * current_orientation_;
        quaternionToEuler(relative, roll, pitch, yaw);
    }

    RCLCPP_INFO(this->get_logger(),
                "Msgs=%lu | Orientation=%s | Timeouts=%d | "
                "Orient: R=%.1f P=%.1f Y=%.1f deg | "
                "Vel: Lin=[%.3f, %.3f, %.3f] m/s",
                msgs_received_.load(),
                orientation_received_ ? "OK" : "WAITING",
                consecutive_timeouts_,
                roll * 180.0 / M_PI,
                pitch * 180.0 / M_PI,
                yaw * 180.0 / M_PI,
                current_velocity_.linear.x(),
                current_velocity_.linear.y(),
                current_velocity_.linear.z());
}

// ==================== MAIN ====================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<OliveImuTeleopNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal error: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

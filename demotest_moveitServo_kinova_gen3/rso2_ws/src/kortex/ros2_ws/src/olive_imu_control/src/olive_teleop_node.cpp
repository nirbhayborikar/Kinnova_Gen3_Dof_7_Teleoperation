#include "olive_imu_control/olive_teleop_node.hpp"
//#include <rclcpp/qos.hpp>   // add this include at top of .hpp if not there

OliveTeleopNode::OliveTeleopNode(const rclcpp::NodeOptions& options)
    : Node("olive_teleop_node", options),
      calibrated_(false),
      data_received_(false),
      packets_received_(0)
{
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "  Olive IMU Teleoperation for Kinova");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    current_orientation_ = Eigen::Quaterniond::Identity();
    initial_orientation_ = Eigen::Quaterniond::Identity();
    current_linear_accel_.setZero();
    current_angular_vel_.setZero();
    
    loadParameters();
    setupROS();
    
    RCLCPP_INFO(this->get_logger(), "✓ Ready! Waiting for Olive IMU data...");
    RCLCPP_INFO(this->get_logger(), "📱 Move IMU to calibrate (auto on first packet)");
}

OliveTeleopNode::~OliveTeleopNode() {
    stopRobot();
}

// ==================== LOAD PARAMETERS ====================
void OliveTeleopNode::loadParameters() {
    // Topics
    this->declare_parameter("olive_imu_topic", "/olive/olixSense/x1/id001/imu");
    this->declare_parameter("olive_accel_topic", "/olive/olixSense/x1/id001/acceleration");
    olive_imu_topic_ = this->get_parameter("olive_imu_topic").as_string();
    olive_accel_topic_ = this->get_parameter("olive_accel_topic").as_string();
    
    // Control
    this->declare_parameter("control_rate_hz", 50.0);
    control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
    
    // Velocity limits
    this->declare_parameter("max_linear_velocity", 0.1);
    this->declare_parameter("max_angular_velocity", 0.2);
    max_linear_vel_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_velocity").as_double();
    
    // Acceleration limits
    this->declare_parameter("max_linear_accel", 0.4);
    this->declare_parameter("max_angular_accel", 0.6);
    max_linear_accel_ = this->get_parameter("max_linear_accel").as_double();
    max_angular_accel_ = this->get_parameter("max_angular_accel").as_double();
    
    // Filtering
    this->declare_parameter("deadzone", 0.03);
    this->declare_parameter("smoothing_factor", 0.6);
    this->declare_parameter("timeout_ms", 500.0);
    deadzone_ = this->get_parameter("deadzone").as_double();
    smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();
    timeout_ms_ = this->get_parameter("timeout_ms").as_double();
    
    // Scaling
    this->declare_parameter("roll_to_linear_y_scale", 0.5);
    this->declare_parameter("pitch_to_linear_x_scale", 0.5);
    this->declare_parameter("yaw_to_linear_z_scale", 0.3);
    this->declare_parameter("roll_to_angular_x_scale", 0.0);
    this->declare_parameter("pitch_to_angular_y_scale", 0.0);
    this->declare_parameter("yaw_to_angular_z_scale", 0.4);
    
    roll_to_linear_y_scale_ = this->get_parameter("roll_to_linear_y_scale").as_double();
    pitch_to_linear_x_scale_ = this->get_parameter("pitch_to_linear_x_scale").as_double();
    yaw_to_linear_z_scale_ = this->get_parameter("yaw_to_linear_z_scale").as_double();
    roll_to_angular_x_scale_ = this->get_parameter("roll_to_angular_x_scale").as_double();
    pitch_to_angular_y_scale_ = this->get_parameter("pitch_to_angular_y_scale").as_double();
    yaw_to_angular_z_scale_ = this->get_parameter("yaw_to_angular_z_scale").as_double();
    
    // Coordinate mapping
    this->declare_parameter("invert_roll", false);
    this->declare_parameter("invert_pitch", false);
    this->declare_parameter("invert_yaw", false);
    invert_roll_ = this->get_parameter("invert_roll").as_bool();
    invert_pitch_ = this->get_parameter("invert_pitch").as_bool();
    invert_yaw_ = this->get_parameter("invert_yaw").as_bool();
    
    // Acceleration
    this->declare_parameter("use_acceleration", false);
    this->declare_parameter("accel_scale", 0.02);
    use_acceleration_ = this->get_parameter("use_acceleration").as_bool();
    accel_scale_ = this->get_parameter("accel_scale").as_double();
    
    RCLCPP_INFO(this->get_logger(), "✓ Parameters loaded");
}

// ==================== SETUP ROS ====================
void OliveTeleopNode::setupROS() {
    
    //   // ── FIXED QoS: must match Olive IMU publisher (BEST_EFFORT) ──────────
    // auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    //                    .best_effort()
    //                    .durability_volatile();

  
  
  
  // Subscriber for IMU


    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        olive_imu_topic_, 10,
        std::bind(&OliveTeleopNode::imuCallback, this, std::placeholders::_1));
    
    // Subscriber for acceleration (optional) //imu_qos
    if (use_acceleration_) {
        accel_sub_ = this->create_subscription<geometry_msgs::msg::AccelStamped>(
            olive_accel_topic_, 10,
            std::bind(&OliveTeleopNode::accelCallback, this, std::placeholders::_1));
    }
    
    // Publisher for twist commands
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);
    
    // Control loop timer
    auto control_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
        std::bind(&OliveTeleopNode::controlLoopCallback, this));
    
    // Diagnostics timer
    diagnostics_timer_ = this->create_wall_timer(
        2s, std::bind(&OliveTeleopNode::logStatus, this));
    
    RCLCPP_INFO(this->get_logger(), "✓ Subscribed to: %s", olive_imu_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "✓ Publishing to: /servo_node/delta_twist_cmds");
}

// ==================== IMU CALLBACK ====================
void OliveTeleopNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Extract quaternion
    current_orientation_.w() = msg->orientation.w;
    current_orientation_.x() = msg->orientation.x;
    current_orientation_.y() = msg->orientation.y;
    current_orientation_.z() = msg->orientation.z;
    
    // Extract angular velocity
    current_angular_vel_.x() = msg->angular_velocity.x;
    current_angular_vel_.y() = msg->angular_velocity.y;
    current_angular_vel_.z() = msg->angular_velocity.z;
    
    // Auto-calibrate on first packet
    if (!calibrated_) {
        calibrate(current_orientation_);
    }
    
    data_received_ = true;
    last_imu_time_ = std::chrono::steady_clock::now();
    packets_received_++;
}

// ==================== ACCELERATION CALLBACK ====================
void OliveTeleopNode::accelCallback(const geometry_msgs::msg::AccelStamped::SharedPtr msg) {
    current_linear_accel_.x() = msg->accel.linear.x;
    current_linear_accel_.y() = msg->accel.linear.y;
    current_linear_accel_.z() = msg->accel.linear.z;
}

// ==================== CALIBRATION ====================
void OliveTeleopNode::calibrate(const Eigen::Quaterniond& q) {
    initial_orientation_ = q;
    calibrated_ = true;
    
    double roll, pitch, yaw;
    quaternionToEuler(q, roll, pitch, yaw);
    
    RCLCPP_INFO(this->get_logger(), 
               "✓ Calibrated - Initial orientation: R=%.1f° P=%.1f° Y=%.1f°",
               roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
}

// ==================== GET RELATIVE ORIENTATION ====================
Eigen::Quaterniond OliveTeleopNode::getRelativeOrientation(const Eigen::Quaterniond& current) {
    return initial_orientation_.inverse() * current;
}

// ==================== QUATERNION TO EULER ====================
void OliveTeleopNode::quaternionToEuler(const Eigen::Quaterniond& q, 
                                        double& roll, double& pitch, double& yaw) {
    // Convert to rotation matrix then to Euler angles (ZYX convention)
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    yaw = euler[0];
    pitch = euler[1];
    roll = euler[2];
}

// ==================== QUATERNION TO VELOCITY ====================
VelocityCommand OliveTeleopNode::quaternionToVelocity(const Eigen::Quaterniond& q) {
    VelocityCommand cmd;
    
    // Get Euler angles
    double roll, pitch, yaw;
    quaternionToEuler(q, roll, pitch, yaw);
    
    // Apply inversions
    if (invert_roll_) roll = -roll;
    if (invert_pitch_) pitch = -pitch;
    if (invert_yaw_) yaw = -yaw;
    
    // Map to velocity
    cmd.linear.x() = pitch * pitch_to_linear_x_scale_;
    cmd.linear.y() = roll * roll_to_linear_y_scale_;
    cmd.linear.z() = yaw * yaw_to_linear_z_scale_;
    
    cmd.angular.x() = roll * roll_to_angular_x_scale_;
    cmd.angular.y() = pitch * pitch_to_angular_y_scale_;
    cmd.angular.z() = yaw * yaw_to_angular_z_scale_;
    
    return cmd;
}

// ==================== CONTROL LOOP ====================
void OliveTeleopNode::controlLoopCallback() {
    auto loop_start = std::chrono::steady_clock::now();
    
    if (!data_received_) {
        return;
    }
    
    checkTimeout();
    
    // Get relative orientation
    Eigen::Quaterniond rel_orientation = getRelativeOrientation(current_orientation_);
    
    // Convert to velocity
    VelocityCommand vel_cmd = quaternionToVelocity(rel_orientation);
    
    // Apply filtering
    Eigen::Vector3d deadzone_linear = applyDeadzone(vel_cmd.linear);
    Eigen::Vector3d deadzone_angular = applyDeadzone(vel_cmd.angular);
    
    Eigen::Vector3d smoothed_linear = applySmoothing(current_velocity_.linear, deadzone_linear);
    Eigen::Vector3d smoothed_angular = applySmoothing(current_velocity_.angular, deadzone_angular);
    
    double control_dt = 1.0 / control_rate_hz_;
    Eigen::Vector3d limited_linear = limitAcceleration(
        current_velocity_.linear, smoothed_linear, control_dt);
    Eigen::Vector3d limited_angular = limitAcceleration(
        current_velocity_.angular, smoothed_angular, control_dt);
    
    Eigen::Vector3d final_linear = saturateVelocity(limited_linear, max_linear_vel_);
    Eigen::Vector3d final_angular = saturateVelocity(limited_angular, max_angular_vel_);
    
    // Store current velocity
    current_velocity_.linear = final_linear;
    current_velocity_.angular = final_angular;
    
    // Create Twist message
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link";
    
    twist_msg.twist.linear.x = final_linear.x();
    twist_msg.twist.linear.y = final_linear.y();
    twist_msg.twist.linear.z = final_linear.z();
    twist_msg.twist.angular.x = final_angular.x();
    twist_msg.twist.angular.y = final_angular.y();
    twist_msg.twist.angular.z = final_angular.z();
    
    twist_pub_->publish(twist_msg);
    
    // Track loop time
    auto loop_end = std::chrono::steady_clock::now();
    auto loop_duration = std::chrono::duration<double>(loop_end - loop_start).count();
    loop_times_.push_back(loop_duration);
    if (loop_times_.size() > 100) {
        loop_times_.pop_front();
    }
}

// ==================== FILTERING FUNCTIONS ====================
Eigen::Vector3d OliveTeleopNode::applyDeadzone(const Eigen::Vector3d& input) {
    Eigen::Vector3d output;
    for (int i = 0; i < 3; ++i) {
        output[i] = (std::abs(input[i]) < deadzone_) ? 0.0 : input[i];
    }
    return output;
}

Eigen::Vector3d OliveTeleopNode::applySmoothing(const Eigen::Vector3d& current, 
                                                const Eigen::Vector3d& target) {
    return current * smoothing_factor_ + target * (1.0 - smoothing_factor_);
}

Eigen::Vector3d OliveTeleopNode::limitAcceleration(const Eigen::Vector3d& current,
                                                   const Eigen::Vector3d& target, double dt) {
    Eigen::Vector3d diff = target - current;
    double max_change = max_linear_accel_ * dt;
    
    if (diff.norm() > max_change) {
        diff = diff.normalized() * max_change;
    }
    
    return current + diff;
}

Eigen::Vector3d OliveTeleopNode::saturateVelocity(const Eigen::Vector3d& vel, double max_vel) {
    if (vel.norm() > max_vel) {
        return vel.normalized() * max_vel;
    }
    return vel;
}

// ==================== SAFETY ====================
void OliveTeleopNode::checkTimeout() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double, std::milli>(now - last_imu_time_).count();
    
    if (elapsed > timeout_ms_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "⚠️ IMU timeout (%.1fms) - stopping", elapsed);
        stopRobot();
    }
}

void OliveTeleopNode::stopRobot() {
    current_velocity_.linear.setZero();
    current_velocity_.angular.setZero();
    
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link";
    twist_pub_->publish(twist_msg);
}

// ==================== DIAGNOSTICS ====================
void OliveTeleopNode::logStatus() {
    if (!data_received_) {
        RCLCPP_WARN(this->get_logger(), "⚠️ No data received yet");
        return;
    }
    
    // Calculate average loop time
    double avg_loop_time = 0.0;
    if (!loop_times_.empty()) {
        for (double t : loop_times_) {
            avg_loop_time += t;
        }
        avg_loop_time /= loop_times_.size();
    }
    
    // Get current orientation
    double roll, pitch, yaw;
    Eigen::Quaterniond rel_q = getRelativeOrientation(current_orientation_);
    quaternionToEuler(rel_q, roll, pitch, yaw);
    
    RCLCPP_INFO(this->get_logger(),
               "📊 Packets=%lu | Loop=%.1fms | Orient: R=%.1f° P=%.1f° Y=%.1f° | "
               "Vel: Lin=[%.3f, %.3f, %.3f] Ang=[%.3f, %.3f, %.3f]",
               packets_received_,
               avg_loop_time * 1000.0,
               roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI,
               current_velocity_.linear.x(), current_velocity_.linear.y(), current_velocity_.linear.z(),
               current_velocity_.angular.x(), current_velocity_.angular.y(), current_velocity_.angular.z());
}
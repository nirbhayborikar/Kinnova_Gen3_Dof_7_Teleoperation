// Rotation Vector Implementation for Sensagram Teleoperation

#include "sensagram_teleop_node.hpp"
#include <cmath>

// ==================== CONSTRUCTOR ====================
SensagramTeleopNode::SensagramTeleopNode(const rclcpp::NodeOptions& options)
    : Node("sensagram_teleop_node", options),
      udp_socket_(-1),
      running_(false),
      packets_received_(0),
      initial_orientation_set_(false),
      consecutive_timeouts_(0)
{
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "  Sensagram Teleoperation (Rotation Vector Mode)");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // Initialize quaternion to identity
    initial_orientation_ = Eigen::Quaterniond::Identity();
    
    loadParameters();
    setupUDP();
    setupROS();
    activateServoMode();
    
    RCLCPP_INFO(this->get_logger(), "✓ Ready! Listening on UDP port %d", udp_port_);
    RCLCPP_INFO(this->get_logger(), "📱 Hold phone FLAT - auto-calibrates on first packet");
}

// ==================== DESTRUCTOR ====================
SensagramTeleopNode::~SensagramTeleopNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down...");
    
    running_ = false;
    if (udp_thread_.joinable()) {
        udp_thread_.join();
    }
    
    if (udp_socket_ >= 0) {
        close(udp_socket_);
    }
    
    stopRobot();
    RCLCPP_INFO(this->get_logger(), "✓ Shutdown complete");
}

// ==================== LOAD PARAMETERS ====================
void SensagramTeleopNode::loadParameters() {
    this->declare_parameter("udp_port", 5005);
    this->declare_parameter("control_rate_hz", 50.0);
    this->declare_parameter("max_linear_velocity", 0.3);
    this->declare_parameter("max_angular_velocity", 0.8);
    this->declare_parameter("max_linear_accel", 1.0);
    this->declare_parameter("max_angular_accel", 2.0);
    this->declare_parameter("deadzone", 0.005);
    this->declare_parameter("smoothing_factor", 0.3);
    this->declare_parameter("timeout_ms", 500.0);
    this->declare_parameter("max_consecutive_timeouts", 3);
    this->declare_parameter("gyro_linear_scale", 0.8);
    this->declare_parameter("gyro_angular_scale", 1.5);
    this->declare_parameter("invert_x", false);
    this->declare_parameter("invert_y", true);
    this->declare_parameter("invert_z", true);
    
    udp_port_ = this->get_parameter("udp_port").as_int();
    control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
    max_linear_vel_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_velocity").as_double();
    max_linear_accel_ = this->get_parameter("max_linear_accel").as_double();
    max_angular_accel_ = this->get_parameter("max_angular_accel").as_double();
    deadzone_ = this->get_parameter("deadzone").as_double();
    smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();
    timeout_ms_ = this->get_parameter("timeout_ms").as_double();
    max_consecutive_timeouts_ = this->get_parameter("max_consecutive_timeouts").as_int();
    gyro_linear_scale_ = this->get_parameter("gyro_linear_scale").as_double();
    gyro_angular_scale_ = this->get_parameter("gyro_angular_scale").as_double();
    invert_x_ = this->get_parameter("invert_x").as_bool();
    invert_y_ = this->get_parameter("invert_y").as_bool();
    invert_z_ = this->get_parameter("invert_z").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  UDP Port: %d", udp_port_);
    RCLCPP_INFO(this->get_logger(), "  Control Rate: %.1f Hz", control_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "  Max Linear Vel: %.2f m/s", max_linear_vel_);
    RCLCPP_INFO(this->get_logger(), "  Max Angular Vel: %.2f rad/s", max_angular_vel_);
    RCLCPP_INFO(this->get_logger(), "  Deadzone: %.3f", deadzone_);
    RCLCPP_INFO(this->get_logger(), "  Smoothing: %.2f", smoothing_factor_);
}

// ==================== SETUP UDP ====================
void SensagramTeleopNode::setupUDP() {
    udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to create UDP socket!");
        throw std::runtime_error("UDP socket creation failed");
    }
    
    // Enable port reuse
    int reuse = 1;
    setsockopt(udp_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    #ifdef SO_REUSEPORT
    setsockopt(udp_socket_, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
    #endif
    
    // Non-blocking
    int flags = fcntl(udp_socket_, F_GETFL, 0);
    fcntl(udp_socket_, F_SETFL, flags | O_NONBLOCK);
    
    // Bind
    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(udp_port_);
    server_addr_.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(udp_socket_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to bind UDP socket to port %d", udp_port_);
        close(udp_socket_);
        throw std::runtime_error("UDP bind failed");
    }
    
    running_ = true;
    udp_thread_ = std::thread(&SensagramTeleopNode::udpReceiverThread, this);
    
    RCLCPP_INFO(this->get_logger(), "✓ UDP socket bound to port %d", udp_port_);
}

// ==================== SETUP ROS ====================
void SensagramTeleopNode::setupROS() {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);
    
    servo_mode_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(
        "/servo_node/switch_command_type");
    
    auto control_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / control_rate_hz_));
    control_timer_ = this->create_wall_timer(
        control_period,
        std::bind(&SensagramTeleopNode::controlLoopCallback, this));
    
    diagnostics_timer_ = this->create_wall_timer(
        2s,
        std::bind(&SensagramTeleopNode::logStatus, this));
    
    RCLCPP_INFO(this->get_logger(), "✓ ROS interfaces created");
}

// ==================== ACTIVATE SERVO MODE ====================
void SensagramTeleopNode::activateServoMode() {
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
        RCLCPP_INFO(this->get_logger(), "✓ Servo activated in TWIST mode");
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to activate servo");
    }
}

// ==================== UDP RECEIVER THREAD ====================
void SensagramTeleopNode::udpReceiverThread() {
    while (running_) {
        IMUData imu;
        if (receiveUDPPacket(imu)) {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            
            auto now = std::chrono::steady_clock::now();
            if (packets_received_ > 0) {
                double interval = std::chrono::duration<double, std::milli>(
                    now - last_packet_time_).count();
                packet_intervals_.push_back(interval);
                if (packet_intervals_.size() > 50) {
                    packet_intervals_.pop_front();
                }
            }
            
            latest_imu_ = imu;
            last_packet_time_ = now;
            packets_received_++;
            
            // AUTO-CALIBRATE on first quaternion
            if (!initial_orientation_set_ && imu.has_quaternion) {
                initial_orientation_ = imu.quaternion;
                initial_orientation_set_ = true;
                
                double roll, pitch, yaw;
                quaternionToEuler(initial_orientation_, roll, pitch, yaw);
                
                RCLCPP_INFO(this->get_logger(), 
                           "✓ Calibrated - Initial: R=%.1f° P=%.1f° Y=%.1f°",
                           roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI);
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// ==================== RECEIVE UDP PACKET ====================
bool SensagramTeleopNode::receiveUDPPacket(IMUData& imu) {
    char buffer[4096];
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    
    ssize_t received = recvfrom(udp_socket_, buffer, sizeof(buffer) - 1, 0,
                                 (struct sockaddr*)&client_addr, &addr_len);
    
    if (received > 0) {
        buffer[received] = '\0';
        
        try {
            parseJSON(std::string(buffer), imu);
            imu.timestamp = std::chrono::steady_clock::now();
            return true;
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Failed to parse JSON: %s", e.what());
        }
    }
    
    return false;
}

// ==================== PARSE JSON ====================
void SensagramTeleopNode::parseJSON(const std::string& json_str, IMUData& imu) {
    size_t pos = 0;
    while (pos < json_str.length()) {
        size_t start = json_str.find('{', pos);
        if (start == std::string::npos) break;
        
        int brace_count = 1;
        size_t end = start + 1;
        while (end < json_str.length() && brace_count > 0) {
            if (json_str[end] == '{') brace_count++;
            else if (json_str[end] == '}') brace_count--;
            end++;
        }
        
        if (brace_count == 0) {
            std::string single_json = json_str.substr(start, end - start);
            
            try {
                auto j = json::parse(single_json);
                
                if (j.contains("type") && j.contains("values")) {
                    std::string type = j["type"].get<std::string>();
                    auto values = j["values"];
                    
                    if (type == "android.sensor.rotation_vector" && values.size() >= 4) {
                        imu.quaternion.x() = values[0].get<double>();
                        imu.quaternion.y() = values[1].get<double>();
                        imu.quaternion.z() = values[2].get<double>();
                        imu.quaternion.w() = values[3].get<double>();
                        
                        imu.quaternion.normalize();
                        imu.has_quaternion = true;
                    }
                }
            } catch (const std::exception& e) {
                // Skip malformed JSON
            }
        }
        
        pos = end;
    }
    
    if (!imu.has_quaternion) {
        throw std::runtime_error("No rotation vector found in JSON");
    }
}

// ==================== QUATERNION TO EULER ====================
void SensagramTeleopNode::quaternionToEuler(const Eigen::Quaterniond& q, 
                                             double& roll, double& pitch, double& yaw) {
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    yaw = euler[0];
    pitch = euler[1];
    roll = euler[2];
}

// ==================== CONTROL LOOP ====================
void SensagramTeleopNode::controlLoopCallback() {
    checkTimeout();
    
    IMUData imu;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu = latest_imu_;
    }
    
    VelocityCommand vel_cmd = processGyroData(imu);
    
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

// ==================== PROCESS IMU DATA ====================
VelocityCommand SensagramTeleopNode::processGyroData(const IMUData& imu) {
    VelocityCommand cmd;


    
    if (!imu.has_quaternion || !initial_orientation_set_) {
        return cmd;
    }
    
    // ========== COMPUTE RELATIVE ORIENTATION ==========
    Eigen::Quaterniond relative_quat = initial_orientation_.inverse() * imu.quaternion;
    
    // Convert to Euler angles
    double roll, pitch, yaw;
    quaternionToEuler(relative_quat, roll, pitch, yaw);
   
   
// // new added

    // // ========== LIMIT TILT ANGLES TO PREVENT OVEREXTENSION ==========
    // const double MAX_TILT = 0.1;  // ~17 degrees maximum #0.3
    
    // if (std::abs(pitch) > MAX_TILT) {
    //     pitch = (pitch > 0) ? MAX_TILT : -MAX_TILT;
    //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //                        "⚠️ Pitch limited to %.1f°", MAX_TILT * 180.0/M_PI);
    // }
    
    // if (std::abs(roll) > MAX_TILT) {
    //     roll = (roll > 0) ? MAX_TILT : -MAX_TILT;
    //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //                        "⚠️ Roll limited to %.1f°", MAX_TILT * 180.0/M_PI);
    // }
    
    // if (std::abs(yaw) > MAX_TILT) {
    //     yaw = (yaw > 0) ? MAX_TILT : -MAX_TILT;
    //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //                        "⚠️ Yaw limited to %.1f°", MAX_TILT * 180.0/M_PI);
    // }

// //... above





    //MAPPING....................

    // Forward/back tilt → X (forward/backward)
    // Left/right tilt → Y (left/right)
    // Rotate phone → Z (up/down)


    
    // ========== MAP ORIENTATION TO VELOCITY ==========
    // Eigen::Vector3d raw_linear(
    //     pitch * gyro_linear_scale_ * (invert_y_ ? -1 : 1),
    //     roll * gyro_linear_scale_ * (invert_x_ ? -1 : 1),
    //     0.0
    // );
    
    // Eigen::Vector3d raw_angular(
    //     0.0,
    //     0.0,
    //     yaw * gyro_angular_scale_ * (invert_z_ ? -1 : 1)
    // );


    // New added ....... mapping 
    
    Eigen::Vector3d raw_linear(
        pitch * gyro_linear_scale_ * (invert_y_ ? -1 : 1),   // Pitch → X (forward/back)
        roll * gyro_linear_scale_ * (invert_x_ ? -1 : 1),    // Roll → Y (left/right)
        yaw * gyro_linear_scale_ * (invert_z_ ? -1 : 1)      // Yaw → Z (up/down) ← NEW!
    );
    
    Eigen::Vector3d raw_angular(
        0.0,
        0.0,
        0.0  // No robot rotation (using yaw for Z motion instead)
    );
    
    // ========== APPLY FILTERING ==========
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
    
    
    
    
    // ========== CLAMP TINY VELOCITIES TO ZERO ==========
    const double MIN_VELOCITY = 1e-6;  // 0.000001
    
    for (int i = 0; i < 3; ++i) {
        if (std::abs(final_linear[i]) < MIN_VELOCITY) {
            final_linear[i] = 0.0;
        }
        if (std::abs(final_angular[i]) < MIN_VELOCITY) {
            final_angular[i] = 0.0;
        }
    }

   //....................above clamp added

    
    
    
    
    
    
    
    
    
    // ========== DEBUG LOGGING ==========
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                          "Orient: R=%.1f° P=%.1f° Y=%.1f° → Vel: [%.3f, %.3f, %.3f]",
                          roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI,
                          final_linear.x(), final_linear.y(), final_angular.z());
    
    cmd.linear = final_linear;
    cmd.angular = final_angular;
    return cmd;
}

// ==================== FILTERING ====================
Eigen::Vector3d SensagramTeleopNode::applyDeadzone(const Eigen::Vector3d& input) {
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

Eigen::Vector3d SensagramTeleopNode::applySmoothing(
    const Eigen::Vector3d& current, const Eigen::Vector3d& target) {
    double alpha = 1.0 - smoothing_factor_;
    return alpha * target + smoothing_factor_ * current;
}

Eigen::Vector3d SensagramTeleopNode::limitAcceleration(
    const Eigen::Vector3d& current, const Eigen::Vector3d& target, double dt) {
    Eigen::Vector3d delta = target - current;
    double max_delta = max_linear_accel_ * dt;
    
    if (delta.norm() > max_delta) {
        delta = delta.normalized() * max_delta;
    }
    
    return current + delta;
}

Eigen::Vector3d SensagramTeleopNode::saturateVelocity(
    const Eigen::Vector3d& vel, double max_vel) {
    double norm = vel.norm();
    if (norm > max_vel) {
        return vel * (max_vel / norm);
    }
    return vel;
}

// ==================== SAFETY ====================
void SensagramTeleopNode::checkTimeout() {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double, std::milli>(
        now - last_packet_time_).count();
    
    if (elapsed > timeout_ms_) {
        consecutive_timeouts_++;
        
        if (consecutive_timeouts_ >= max_consecutive_timeouts_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "⏱ Connection lost (%.0f ms) → stopping robot",
                               elapsed);
            stopRobot();
        }
    } else {
        consecutive_timeouts_ = 0;
    }
}

void SensagramTeleopNode::stopRobot() {
    publishTwist(createZeroTwist());
    current_velocity_.linear.setZero();
    current_velocity_.angular.setZero();
}

geometry_msgs::msg::TwistStamped SensagramTeleopNode::createZeroTwist() {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->now();
    twist.header.frame_id = "base_link";
    return twist;
}

// ==================== PUBLISHING ====================
void SensagramTeleopNode::publishTwist(const geometry_msgs::msg::TwistStamped& twist) {
    twist_pub_->publish(twist);
}

// ==================== DIAGNOSTICS ====================
void SensagramTeleopNode::logStatus() {
    double packet_rate = 0.0;
    if (!packet_intervals_.empty()) {
        double avg_interval = 0.0;
        for (double interval : packet_intervals_) {
            avg_interval += interval;
        }
        avg_interval /= packet_intervals_.size();
        packet_rate = 1000.0 / avg_interval;
    }
    
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    if (latest_imu_.has_quaternion && initial_orientation_set_) {
        Eigen::Quaterniond relative = initial_orientation_.inverse() * latest_imu_.quaternion;
        quaternionToEuler(relative, roll, pitch, yaw);
    }
    
    RCLCPP_INFO(this->get_logger(),
                "📊 Packets=%lu | Rate=%.1f Hz | Quat=%s | Timeouts=%d\n"
                "   Orient: R=%.1f° P=%.1f° Y=%.1f°\n"
                "   Vel: Lin=[%.3f, %.3f] | Ang=[%.3f] m/s, rad/s",
                packets_received_.load(),
                packet_rate,
                latest_imu_.has_quaternion ? "✓" : "✗",
                consecutive_timeouts_,
                roll * 180.0/M_PI,
                pitch * 180.0/M_PI,
                yaw * 180.0/M_PI,
                current_velocity_.linear.x(),
                current_velocity_.linear.y(),
                current_velocity_.angular.z());
}

// ==================== MAIN ====================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SensagramTeleopNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
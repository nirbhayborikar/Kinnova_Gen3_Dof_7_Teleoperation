#ifndef SENSAGRAM_TELEOP_NODE_HPP
#define SENSAGRAM_TELEOP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <std_msgs/msg/int8.hpp>

#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <deque>

using json = nlohmann::json;
using namespace std::chrono_literals;

// ==================== IMU DATA (ROTATION VECTOR) ====================
struct IMUData {
    Eigen::Quaterniond quaternion;
    bool has_quaternion;
    std::chrono::steady_clock::time_point timestamp;
    
    IMUData() : has_quaternion(false) {
        quaternion = Eigen::Quaterniond::Identity();
        timestamp = std::chrono::steady_clock::now();
    }
};

// ==================== FILTERED VELOCITY ====================
struct VelocityCommand {
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
    
    VelocityCommand() {
        linear.setZero();
        angular.setZero();
    }
};

// ==================== TELEOPERATION NODE ====================
class SensagramTeleopNode : public rclcpp::Node {
public:
    explicit SensagramTeleopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SensagramTeleopNode();

private:
    // ===== INITIALIZATION =====
    void loadParameters();
    void setupUDP();
    void setupROS();
    void activateServoMode();
    
    // ===== UDP THREAD =====
    void udpReceiverThread();
    bool receiveUDPPacket(IMUData& imu);
    void parseJSON(const std::string& json_str, IMUData& imu);
    
    // ===== CONTROL LOOP =====
    void controlLoopCallback();
    VelocityCommand processGyroData(const IMUData& imu);
    
    // ===== QUATERNION PROCESSING =====
    void quaternionToEuler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);
    
    // ===== FILTERING =====
    Eigen::Vector3d applyDeadzone(const Eigen::Vector3d& input);
    Eigen::Vector3d applySmoothing(const Eigen::Vector3d& current, const Eigen::Vector3d& target);
    Eigen::Vector3d limitAcceleration(const Eigen::Vector3d& current, const Eigen::Vector3d& target, double dt);
    Eigen::Vector3d saturateVelocity(const Eigen::Vector3d& vel, double max_vel);
    
    // ===== SAFETY =====
    void checkTimeout();
    void stopRobot();
    geometry_msgs::msg::TwistStamped createZeroTwist();
    
    // ===== PUBLISHING =====
    void publishTwist(const geometry_msgs::msg::TwistStamped& twist);
    
    // ===== DIAGNOSTICS =====
    void logStatus();
    
    // ===== ROS INTERFACES =====
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr servo_mode_client_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    
    // ===== UDP SOCKET =====
    int udp_socket_;
    struct sockaddr_in server_addr_;
    std::thread udp_thread_;
    std::atomic<bool> running_;
    
    // ===== DATA & STATE =====
    std::mutex imu_mutex_;
    IMUData latest_imu_;
    VelocityCommand current_velocity_;
    std::chrono::steady_clock::time_point last_packet_time_;
    
    // Edited: 2026-03-30 - Track homing state for 10s timeout logic
    bool is_homing_; 
    
    // ===== ORIENTATION TRACKING =====
    Eigen::Quaterniond initial_orientation_;
    bool initial_orientation_set_;
    
    // ===== STATISTICS =====
    std::atomic<uint64_t> packets_received_;
    std::deque<double> packet_intervals_;
    
    // ===== TIMEOUT HANDLING =====
    int consecutive_timeouts_;
    int max_consecutive_timeouts_;
    
    // ===== PARAMETERS =====
    int udp_port_;
    double control_rate_hz_;
    double max_linear_vel_;
    double max_angular_vel_;
    double max_linear_accel_;
    double max_angular_accel_;
    double deadzone_;
    double smoothing_factor_;
    double timeout_ms_;
    double gyro_linear_scale_;
    double gyro_angular_scale_;
    bool invert_x_, invert_y_, invert_z_;
};

#endif  // SENSAGRAM_TELEOP_NODE_HPP
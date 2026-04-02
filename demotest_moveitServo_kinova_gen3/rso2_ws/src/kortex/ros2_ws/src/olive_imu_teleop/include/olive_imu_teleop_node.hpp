#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <Eigen/Geometry>

#include <mutex>
#include <atomic>
#include <deque>
#include <chrono>

using namespace std::chrono_literals;

// ==================== DATA STRUCTURES ====================
struct VelocityCommand {
    Eigen::Vector3d linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular = Eigen::Vector3d::Zero();
};

// ==================== NODE CLASS ====================
class OliveImuTeleopNode : public rclcpp::Node {
public:
    explicit OliveImuTeleopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~OliveImuTeleopNode();

private:
    // ---------- SETUP ----------
    void loadParameters();
    void setupROS();
    void activateServoMode();

    // ---------- CALLBACKS ----------
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void controlLoopCallback();

    // ---------- PROCESSING ----------
    VelocityCommand processOrientation();
    void quaternionToEuler(const Eigen::Quaterniond& q,
                           double& roll, double& pitch, double& yaw);

    // ---------- FILTERING ----------
    Eigen::Vector3d applyDeadzone(const Eigen::Vector3d& input);
    Eigen::Vector3d applySmoothing(const Eigen::Vector3d& current,
                                    const Eigen::Vector3d& target);
    Eigen::Vector3d limitAcceleration(const Eigen::Vector3d& current,
                                       const Eigen::Vector3d& target, double dt);
    Eigen::Vector3d saturateVelocity(const Eigen::Vector3d& vel, double max_vel);

    // ---------- SAFETY ----------
    void checkTimeout();
    void stopRobot();
    geometry_msgs::msg::TwistStamped createZeroTwist();

    // ---------- PUBLISHING ----------
    void publishTwist(const geometry_msgs::msg::TwistStamped& twist);

    // ---------- DIAGNOSTICS ----------
    void logStatus();

    // ========== ROS INTERFACES ==========
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr servo_mode_client_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    // ========== IMU STATE ==========
    Eigen::Quaterniond current_orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond initial_orientation_ = Eigen::Quaterniond::Identity();
    bool initial_orientation_set_ = false;
    bool orientation_received_ = false;
    std::mutex imu_mutex_;
    std::chrono::steady_clock::time_point last_msg_time_;

    // ========== CONTROL STATE ==========
    VelocityCommand current_velocity_;

    // ========== PARAMETERS ==========
    std::string imu_frame_id_;
    double control_rate_hz_;
    double max_linear_vel_;
    double max_angular_vel_;
    double max_linear_accel_;
    double max_angular_accel_;
    double deadzone_;
    double smoothing_factor_;
    double timeout_ms_;
    int max_consecutive_timeouts_;
    double linear_scale_;
    double angular_scale_;
    bool invert_x_;
    bool invert_y_;
    bool invert_z_;

    // ========== DIAGNOSTICS ==========
    std::atomic<uint64_t> msgs_received_{0};
    int consecutive_timeouts_ = 0;
};

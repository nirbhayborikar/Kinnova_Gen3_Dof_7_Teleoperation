#ifndef OLIVE_TELEOP_NODE_HPP
#define OLIVE_TELEOP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
//#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <deque>


using namespace std::chrono_literals;

// ==================== VELOCITY COMMAND ====================
struct VelocityCommand {
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
    
    VelocityCommand() {
        linear.setZero();
        angular.setZero();
    }
};

// ==================== OLIVE TELEOP NODE ====================
class OliveTeleopNode : public rclcpp::Node {
public:
    explicit OliveTeleopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~OliveTeleopNode();

private:
    // ===== INITIALIZATION =====
    void loadParameters();
    void setupROS();
    
    // ===== CALLBACKS =====
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void accelCallback(const geometry_msgs::msg::AccelStamped::SharedPtr msg);
    void controlLoopCallback();
    
    // ===== PROCESSING =====
    VelocityCommand quaternionToVelocity(const Eigen::Quaterniond& q);
    void quaternionToEuler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);
    
    // ===== FILTERING =====
    Eigen::Vector3d applyDeadzone(const Eigen::Vector3d& input);
    Eigen::Vector3d applySmoothing(const Eigen::Vector3d& current, const Eigen::Vector3d& target);
    Eigen::Vector3d limitAcceleration(const Eigen::Vector3d& current, const Eigen::Vector3d& target, double dt);
    Eigen::Vector3d saturateVelocity(const Eigen::Vector3d& vel, double max_vel);
    
    // ===== CALIBRATION =====
    void calibrate(const Eigen::Quaterniond& q);
    Eigen::Quaterniond getRelativeOrientation(const Eigen::Quaterniond& current);
    
    // ===== SAFETY =====
    void checkTimeout();
    void stopRobot();
    
    // ===== DIAGNOSTICS =====
    void logStatus();
    
    // ===== ROS INTERFACES =====
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr accel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    
    // ===== DATA =====
    Eigen::Quaterniond current_orientation_;
    Eigen::Quaterniond initial_orientation_;
    Eigen::Vector3d current_linear_accel_;
    Eigen::Vector3d current_angular_vel_;
    VelocityCommand current_velocity_;
    
    bool calibrated_;
    bool data_received_;
    std::chrono::steady_clock::time_point last_imu_time_;
    
    // ===== STATISTICS =====
    uint64_t packets_received_;
    std::deque<double> loop_times_;
    
    // ===== PARAMETERS =====
    std::string olive_imu_topic_;
    std::string olive_accel_topic_;
    double control_rate_hz_;
    double max_linear_vel_;
    double max_angular_vel_;
    double max_linear_accel_;
    double max_angular_accel_;
    double deadzone_;
    double smoothing_factor_;
    double timeout_ms_;
    
    // Scaling factors
    double roll_to_linear_y_scale_;
    double pitch_to_linear_x_scale_;
    double yaw_to_linear_z_scale_;
    double roll_to_angular_x_scale_;
    double pitch_to_angular_y_scale_;
    double yaw_to_angular_z_scale_;
    
    // Coordinate mapping
    bool invert_roll_;
    bool invert_pitch_;
    bool invert_yaw_;
    
    // Use acceleration for additional control
    bool use_acceleration_;
    double accel_scale_;
};

#endif  // OLIVE_TELEOP_NODE_HPP
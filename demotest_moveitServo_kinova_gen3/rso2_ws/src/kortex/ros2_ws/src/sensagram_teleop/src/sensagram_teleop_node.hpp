#ifndef SENSAGRAM_TELEOP_NODE_HPP
#define SENSAGRAM_TELEOP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// home position action
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <std_msgs/msg/int8.hpp>

// for gripper
#include "control_msgs/action/gripper_command.hpp"

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

// ==================== IMU DATA ====================
struct IMUData {
    Eigen::Quaterniond quaternion;
    bool has_quaternion;
    std::chrono::steady_clock::time_point timestamp;
    
    IMUData() : has_quaternion(false) {
        quaternion = Eigen::Quaterniond::Identity();
        timestamp = std::chrono::steady_clock::now();
    }
};

// ==================== VELOCITY COMMAND ====================
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
    bool isPhoneFlat(double roll, double pitch, double yaw);
    
    // ===== FILTERING =====
    Eigen::Vector3d applyDeadzone(const Eigen::Vector3d& input);
    Eigen::Vector3d applySmoothing(const Eigen::Vector3d& current, const Eigen::Vector3d& target);
    Eigen::Vector3d limitAcceleration(const Eigen::Vector3d& current, const Eigen::Vector3d& target, double dt);
    Eigen::Vector3d saturateVelocity(const Eigen::Vector3d& vel, double max_vel);
    
    // ===== SAFETY =====
    bool isRobotStuck(const Eigen::Vector3d& commanded_vel);
    void checkTimeout();
    void stopRobot();
    geometry_msgs::msg::TwistStamped createZeroTwist();
    
    // ===== PUBLISHING =====
    void publishTwist(const geometry_msgs::msg::TwistStamped& twist);
    
    // ===== DIAGNOSTICS =====
    void logStatus();
    
    // ===== ROS INTERFACES =====
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_status_sub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr servo_mode_client_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    // Define the Action types for easier reading
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

    // The Action Client
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
    
    // Timer to handle the initial opening
    rclcpp::TimerBase::SharedPtr gripper_init_timer_;

    // Function to send the goal
    void openGripperAtStart();

    // the home position recovery function

    // --- ACTION TYPES ---
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    // --- MEMBER VARIABLES ---
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr home_action_client_;
    bool is_recovering_; 

    // --- FUNCTIONS ---
    void goHomeRecovery();

    // ===== UDP SOCKET =====
    int udp_socket_;
    struct sockaddr_in server_addr_;
    std::thread udp_thread_;
    std::atomic<bool> running_;
    
    // ===== DATA =====
    std::mutex imu_mutex_;
    IMUData latest_imu_;
    VelocityCommand current_velocity_;
    std::chrono::steady_clock::time_point last_packet_time_;
    
    // ===== JOINT STATE =====
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
    
    // ===== STUCK DETECTION =====
    double max_velocity_seen_;
    std::chrono::steady_clock::time_point last_reset_time_;
    int servo_status_code_;
    static constexpr int SERVO_ERROR_THRESHOLD = 25;
    int consecutive_servo_errors_;
    std::chrono::steady_clock::time_point startup_time_;
    
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
    double flat_phone_threshold_;
    bool invert_x_, invert_y_, invert_z_;
};

#endif  // SENSAGRAM_TELEOP_NODE_HPP
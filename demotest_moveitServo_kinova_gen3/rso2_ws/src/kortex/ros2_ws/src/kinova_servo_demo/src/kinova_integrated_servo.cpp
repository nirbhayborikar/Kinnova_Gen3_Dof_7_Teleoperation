#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

class IntegratedKinovaServo : public rclcpp::Node {
public:
  IntegratedKinovaServo(const rclcpp::NodeOptions & options) 
      : Node("integrated_kinova_servo", options) {
    
    // 1. Setup Publishers (Internal topics that Servo listens to)
    // We publish to the topics defined in the config file, usually:
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "delta_twist_cmds", 10);
    
    // 2. Setup Planning Scene Monitor (The "Brain")
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        this->shared_from_this(), "robot_description", tf_buffer_, "planning_scene_monitor");

    // Start monitoring the actual robot state
    if (planning_scene_monitor_->getPlanningScene()) {
      planning_scene_monitor_->startStateMonitor("/joint_states");
      planning_scene_monitor_->startSceneMonitor();
      planning_scene_monitor_->providePlanningSceneService();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning scene not configured!");
      return;
    }

    // 3. Initialize Servo
    // This loads parameters from the node's parameters (passed via Launch file)
    auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(this->shared_from_this());
    if (!servo_parameters) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load servo parameters!");
      return; 
    }

    // Create and Start Servo
    servo_ = std::make_unique<moveit_servo::Servo>(this->shared_from_this(), servo_parameters, planning_scene_monitor_);
    servo_->start();
    
    // 4. Setup Logic Timer
    start_time_ = this->now();
    current_state_ = State::WAIT_FOR_SERVO;
    
    timer_ = this->create_wall_timer(
        50ms, std::bind(&IntegratedKinovaServo::timerCallback, this));
        
    RCLCPP_INFO(this->get_logger(), "Integrated Node Started. Waiting for Servo to settle...");
  }

private:
  enum class State {
    WAIT_FOR_SERVO,
    MOVE_X_POSITIVE,
    MOVE_X_NEGATIVE,
    COMPLETE
  };

  void timerCallback() {
    auto current_time = this->now();
    auto elapsed = current_time - start_time_;
    
    // Create the message
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "base_link"; // Ensure this frame exists in your TF tree!
    
    double velocity = 0.02; // m/s

    switch (current_state_) {
      case State::WAIT_FOR_SERVO:
        // Give Servo a moment to start up
        if (elapsed > std::chrono::seconds(4)) {
           RCLCPP_INFO(this->get_logger(), "Starting Motion!");
           start_time_ = this->now();
           current_state_ = State::MOVE_X_POSITIVE;
        }
        break;

      case State::MOVE_X_POSITIVE:
        msg->twist.linear.x = velocity;
        twist_pub_->publish(std::move(msg));
        
        if (elapsed > std::chrono::seconds(5)) {
          start_time_ = this->now();
          current_state_ = State::MOVE_X_NEGATIVE;
          RCLCPP_INFO(this->get_logger(), "Switching to -X");
        }
        break;

      case State::MOVE_X_NEGATIVE:
        msg->twist.linear.x = -velocity;
        twist_pub_->publish(std::move(msg));

        if (elapsed > std::chrono::seconds(5)) {
          current_state_ = State::COMPLETE;
          RCLCPP_INFO(this->get_logger(), "Motion Complete.");
        }
        break;
        
      case State::COMPLETE:
        // Do nothing
        break;
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::unique_ptr<moveit_servo::Servo> servo_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  State current_state_;
  rclcpp::Time start_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true); 
  
  auto node = std::make_shared<IntegratedKinovaServo>(node_options);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();
  
  rclcpp::shutdown();
  return 0;
}

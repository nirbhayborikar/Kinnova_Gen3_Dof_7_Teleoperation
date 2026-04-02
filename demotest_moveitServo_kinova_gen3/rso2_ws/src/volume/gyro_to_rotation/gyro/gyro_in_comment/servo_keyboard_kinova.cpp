// #include <chrono>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist_stamped.hpp>
// #include <control_msgs/msg/joint_jog.hpp>
// #include <moveit_msgs/srv/servo_command_type.hpp>
// #include <future>

// // Constants for Kinova Gen3 Kortex
// namespace {
// const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
// const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
// const std::string SWITCH_SERVICE = "/servo_node/switch_command_type";
// const size_t ROS_QUEUE_SIZE = 10;
// const std::string PLANNING_FRAME_ID = "world";
// const std::string EE_FRAME_ID = "grasping_frame";
// }  // namespace

// class AutomatedSlowMotion : public rclcpp::Node {
// public:
//   AutomatedSlowMotion() : Node("automated_slow_motion") {
//     // Create publishers (like Code 1)
//     twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
//         TWIST_TOPIC, ROS_QUEUE_SIZE);
//     joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
//         JOINT_TOPIC, ROS_QUEUE_SIZE);
    
//     // Service client to switch modes (like Code 1)
//     switch_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(SWITCH_SERVICE);
    
//     // Timer for automated motion (like Code 2)
//     // 50ms = 20Hz update rate (smooth motion)
//     timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(50),
//         std::bind(&AutomatedSlowMotion::timerCallback, this));
    
//     // Parameters for slow motion
//     linear_velocity_ = 0.02;  // 2 cm/s - VERY SLOW
//     angular_velocity_ = 0.05; // 0.05 rad/s - SLOW
    
//     // State machine
//     current_state_ = State::START;
//     motion_duration_ = std::chrono::seconds(5);  // 5 seconds per motion
//     start_time_ = this->now();
    
//     RCLCPP_INFO(this->get_logger(), "Automated Slow Motion Controller Started");
//     RCLCPP_INFO(this->get_logger(), "Linear velocity: %.3f m/s", linear_velocity_);
//     RCLCPP_INFO(this->get_logger(), "Motion sequence will begin in 2 seconds...");
//   }

// private:
//   enum class State {
//     START,
//     SWITCH_TO_TWIST,
//     // MOVE_X_POSITIVE,   // +X direction
//     MOVE_X_NEGATIVE,   // -X direction  
//     // MOVE_Y_POSITIVE,   // +Y direction
//     // MOVE_Y_NEGATIVE,   // -Y direction
//     // MOVE_Z_POSITIVE,   // +Z direction
//     // MOVE_Z_NEGATIVE,   // -Z direction
//     SWITCH_TO_JOINT,
//     MOVE_JOINT1,
//     MOVE_JOINT2,
//     COMPLETE
//   };
  
//   void timerCallback() {
//     auto current_time = this->now();
//     auto elapsed = current_time - start_time_;
    
//     switch (current_state_) {
//       case State::START:
//         // Wait 2 seconds before starting
//         if (elapsed > std::chrono::seconds(2)) {
//           switchToTwistMode();
//           current_state_ = State::SWITCH_TO_TWIST;
//           RCLCPP_INFO(this->get_logger(), "Switching to TWIST mode...");
//         }
//         break;
        
//       case State::SWITCH_TO_TWIST:
//         // Wait for service response or just proceed after delay
//         if (elapsed > std::chrono::seconds(3)) {
//           start_time_ = this->now();
//           current_state_ = State::MOVE_X_NEGATIVE;
//           RCLCPP_INFO(this->get_logger(), "Moving slowly in -X direction...");
//         }
//         break;
        
//       // case State::MOVE_X_POSITIVE:
//       //   moveInDirection(linear_velocity_, 0.0, 0.0, 0.0, 0.0, 0.0);
//       //   if (elapsed > motion_duration_) {
//       //     start_time_ = this->now();
//       //     current_state_ = State::MOVE_X_NEGATIVE;
//       //     RCLCPP_INFO(this->get_logger(), "Moving slowly in -X direction...");
//       //   }
//       //   break;
        
//       case State::MOVE_X_NEGATIVE:
//         moveInDirection(-linear_velocity_, 0.0, 0.0, 0.0, 0.0, 0.0);
//         if (elapsed > motion_duration_) {
//           start_time_ = this->now();
//           // current_state_ = State::MOVE_Y_POSITIVE;
//           RCLCPP_INFO(this->get_logger(), "Moving slowly in +Y direction...");
//         }
//         break;
        
//       // case State::MOVE_Y_POSITIVE:
//       //   moveInDirection(0.0, linear_velocity_, 0.0, 0.0, 0.0, 0.0);
//       //   if (elapsed > motion_duration_) {
//       //     start_time_ = this->now();
//       //     // current_state_ = State::MOVE_Y_NEGATIVE;
//       //     RCLCPP_INFO(this->get_logger(), "Moving slowly in -Y direction...");
//       //   }
//       //   break;
        
//       // case State::MOVE_Y_NEGATIVE:
//       //   moveInDirection(0.0, -linear_velocity_, 0.0, 0.0, 0.0, 0.0);
//       //   if (elapsed > motion_duration_) {
//       //     start_time_ = this->now();
//       //     current_state_ = State::MOVE_Z_POSITIVE;
//       //     RCLCPP_INFO(this->get_logger(), "Moving slowly in +Z direction...");
//       //   }
//       //   break;
        
//       // case State::MOVE_Z_POSITIVE:
//       //   moveInDirection(0.0, 0.0, linear_velocity_, 0.0, 0.0, 0.0);
//       //   if (elapsed > motion_duration_) {
//       //     start_time_ = this->now();
//       //     current_state_ = State::MOVE_Z_NEGATIVE;
//       //     RCLCPP_INFO(this->get_logger(), "Moving slowly in -Z direction...");
//       //   }
//       //   break;
        
//       // case State::MOVE_Z_NEGATIVE:
//       //   moveInDirection(0.0, 0.0, -linear_velocity_, 0.0, 0.0, 0.0);
//       //   if (elapsed > motion_duration_) {
//       //     // Stop all motion
//       //     moveInDirection(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//       //     current_state_ = State::COMPLETE;
//       //     RCLCPP_INFO(this->get_logger(), "Motion sequence complete!");
//       //     RCLCPP_INFO(this->get_logger(), "Robot stopped.");
//       //     timer_->cancel();  // Stop the timer
//       //   }
//       //   break;
        
//       default:
//         break;
//     }
//   }
  
//   void moveInDirection(double vx, double vy, double vz, 
//                        double wx, double wy, double wz) {
//     auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    
//     twist_msg->header.stamp = this->now();
//     twist_msg->header.frame_id = PLANNING_FRAME_ID;  // Move relative to world frame
    
//     // Set linear velocities (VERY SLOW)
//     twist_msg->twist.linear.x = vx;
//     twist_msg->twist.linear.y = vy;
//     twist_msg->twist.linear.z = vz;
    
//     // Set angular velocities
//     twist_msg->twist.angular.x = wx;
//     twist_msg->twist.angular.y = wy;
//     twist_msg->twist.angular.z = wz;
    
//     // Publish the command (like Code 1)
//     twist_pub_->publish(std::move(twist_msg));
//   }
  
//   void switchToTwistMode() {
//     if (!switch_client_->wait_for_service(std::chrono::seconds(2))) {
//         RCLCPP_WARN(this->get_logger(), "Service not available, assuming TWIST mode...");
//         return;
//     }
    
//     auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();  
//     request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;  

//     // Simple synchronous call (blocks for max 1 second)
//     auto future = switch_client_->async_send_request(request);
    
//     // Wait for result with timeout
//     if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
//         try {
//             auto response = future.get();
//             if (response->success) {
//                 RCLCPP_INFO(this->get_logger(), "Successfully switched to TWIST mode");
//             } else {
//                 RCLCPP_WARN(this->get_logger(), "Failed to switch to TWIST mode");
//             }
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
//           }
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Service call timeout, continuing anyway...");
//       }
//   }
  
//   // Member variables
//   rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
//   rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
//   rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_client_;
//   rclcpp::TimerBase::SharedPtr timer_;
  
//   State current_state_;
//   double linear_velocity_;
//   double angular_velocity_;
//   std::chrono::nanoseconds motion_duration_;
//   rclcpp::Time start_time_;
// };

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
  
//   // Print what the program will do
//   std::cout << "==========================================" << std::endl;
//   std::cout << "AUTOMATED SLOW MOTION DEMO" << std::endl;
//   std::cout << "==========================================" << std::endl;
//   std::cout << "This program will:" << std::endl;
//   std::cout << "1. Switch servo to TWIST mode" << std::endl;
//   std::cout << "2. Move slowly in +X direction for 5 seconds" << std::endl;
//   std::cout << "3. Move slowly in -X direction for 5 seconds" << std::endl;
//   std::cout << "4. Move slowly in +Y direction for 5 seconds" << std::endl;
//   std::cout << "5. Move slowly in -Y direction for 5 seconds" << std::endl;
//   std::cout << "6. Move slowly in +Z direction for 5 seconds" << std::endl;
//   std::cout << "7. Move slowly in -Z direction for 5 seconds" << std::endl;
//   std::cout << "8. Stop all motion" << std::endl;
//   std::cout << "==========================================" << std::endl;
//   std::cout << "Starting in 2 seconds..." << std::endl;
  
//   auto node = std::make_shared<AutomatedSlowMotion>();
  
//   // Use MultiThreadedExecutor for smooth operation
//   auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
//   executor->add_node(node);
//   executor->spin();
  
//   rclcpp::shutdown();
//   return 0;
// }












// starts movement at end aftr success


// #include <chrono>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist_stamped.hpp>
// #include <control_msgs/msg/joint_jog.hpp>
// #include <moveit_msgs/srv/servo_command_type.hpp>
// #include <future>

// namespace {
// const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
// const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
// const std::string SWITCH_SERVICE = "/servo_node/switch_command_type";
// const std::string PLANNING_FRAME_ID = "world";
// }

// class AutomatedSlowMotion : public rclcpp::Node {
// public:
//     AutomatedSlowMotion() : Node("automated_slow_motion") {
//         // High-priority QoS to prevent command buffering/lag
//         auto qos = rclcpp::SensorDataQoS();

//         twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, qos);
//         joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, qos);
        
//         switch_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(SWITCH_SERVICE);
        
//         // 20Hz update rate for smooth Servo operation
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(50),
//             std::bind(&AutomatedSlowMotion::timerCallback, this));

//         linear_velocity_ = 0.05; // Slightly faster for visibility (5cm/s)
//         motion_duration_ = std::chrono::seconds(5);
        
//         // Start immediately by requesting the mode switch
//         current_state_ = State::SWITCH_TO_TWIST;
//         switchToTwistMode();

//         RCLCPP_INFO(this->get_logger(), "Node started. Requesting TWIST mode...");
//     }

// private:
//     enum class State {
//         SWITCH_TO_TWIST,
//         MOVE_X_POSITIVE,
//         MOVE_X_NEGATIVE,
//         // SWITCH_TO_JOINT, // Commented out as requested
//         // MOVE_JOINT,      // Commented out as requested
//         COMPLETE
//     };

//     void timerCallback() {
//         // We only process movement if we aren't in the switching phase
//         if (current_state_ == State::SWITCH_TO_TWIST || current_state_ == State::COMPLETE) {
//             return;
//         }

//         auto current_time = this->now();
//         auto elapsed = current_time - start_time_;

//         switch (current_state_) {
//             case State::MOVE_X_POSITIVE:
//                 moveInDirection(linear_velocity_, 0.0, 0.0, 0.0, 0.0, 0.0);
//                 if (elapsed > motion_duration_) {
//                     RCLCPP_INFO(this->get_logger(), "Moving X Negative...");
//                     start_time_ = this->now();
//                     current_state_ = State::MOVE_X_NEGATIVE;
//                 }
//                 break;

//             case State::MOVE_X_NEGATIVE:
//                 moveInDirection(-linear_velocity_, 0.0, 0.0, 0.0, 0.0, 0.0);
//                 if (elapsed > motion_duration_) {
//                     RCLCPP_INFO(this->get_logger(), "Sequence Complete. Stopping.");
//                     moveInDirection(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//                     current_state_ = State::COMPLETE;
//                 }
//                 break;

//             default:
//                 break;
//         }
//     }

//     void moveInDirection(double vx, double vy, double vz, double wx, double wy, double wz) {
//         auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
//         twist_msg->header.stamp = this->now();
//         twist_msg->header.frame_id = PLANNING_FRAME_ID;
        
//         twist_msg->twist.linear.x = vx;
//         twist_msg->twist.linear.y = vy;
//         twist_msg->twist.linear.z = vz;
//         twist_msg->twist.angular.x = wx;
//         twist_msg->twist.angular.y = wy;
//         twist_msg->twist.angular.z = wz;

//         twist_pub_->publish(std::move(twist_msg));
//     }

//     /* // JOINT JOGGING FUNCTION (Commented Out)
//     void jogJoint(std::string joint_name, double velocity) {
//         auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
//         joint_msg->header.stamp = this->now();
//         joint_msg->header.frame_id = PLANNING_FRAME_ID;
//         joint_msg->joint_names.push_back(joint_name);
//         joint_msg->velocities.push_back(velocity);
//         joint_pub_->publish(std::move(joint_msg));
//     }
//     */

//     void switchToTwistMode() {
//         // timer_->cancel();
      
//         if (!switch_client_->wait_for_service(std::chrono::seconds(5))) {
//             RCLCPP_ERROR(this->get_logger(), "Servo Service not found!");
//             return;
//         }

//         auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
//         request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;

//         // The key fix: The lambda function below executes ONLY when the robot replies
//         switch_client_->async_send_request(request, 
//             [this](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture future) {
//                 auto response = future.get();
//                 if (response->success) {
//                     RCLCPP_INFO(this->get_logger(), "Robot Ready. Starting +X Motion immediately.");
//                     // if (!timer_->is_canceled()) {
//                     //     timer_->reset();  // Restart the timer
//                     // }
//                     this->start_time_ = this->now(); // Reset stopwatch the moment robot says OK
//                     this->current_state_ = State::MOVE_X_POSITIVE;
//                 } else {
//                     RCLCPP_ERROR(this->get_logger(), "Robot rejected TWIST mode switch.");
//                 }
//             });
//     }

//     rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
//     rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
//     rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_client_;
//     rclcpp::TimerBase::SharedPtr timer_;
    
//     State current_state_;
//     double linear_velocity_;
//     std::chrono::nanoseconds motion_duration_;
//     rclcpp::Time start_time_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<AutomatedSlowMotion>();
    
//     // MultiThreadedExecutor is required to handle the service callback and timer simultaneously
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     executor.spin();
    
//     rclcpp::shutdown();
//     return 0;
// }










// #include <chrono>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist_stamped.hpp>
// #include <control_msgs/msg/joint_jog.hpp>
// #include <moveit_msgs/srv/servo_command_type.hpp>

// namespace {
// const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
// const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
// const std::string SWITCH_SERVICE = "/servo_node/switch_command_type";
// const std::string PLANNING_FRAME_ID = "world";
// }

// class AutomatedSlowMotion : public rclcpp::Node {
// public:
//     AutomatedSlowMotion() : Node("automated_slow_motion"), is_ready_(false) {
//         auto qos = rclcpp::SensorDataQoS();
//         twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, qos);
//         joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, qos);
//         switch_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(SWITCH_SERVICE);
        
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(50),
//             std::bind(&AutomatedSlowMotion::timerCallback, this));

//         linear_velocity_ = 0.05; 
//         motion_duration_ = std::chrono::seconds(5);
//         current_state_ = State::MOVE_X_POSITIVE;

//         RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for service...");
        
//         // Use a timer to trigger the service call once to ensure the node is fully spun up
//         trigger_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(500), [this]() {
//                 this->switchToTwistMode();
//                 this->trigger_timer_->cancel(); 
//             });
//     }

// private:
//     enum class State { MOVE_X_POSITIVE, MOVE_X_NEGATIVE, COMPLETE };

//     void timerCallback() {
//         if (!is_ready_ || current_state_ == State::COMPLETE) {
//             return;
//         }

//         auto current_time = this->now();
//         auto elapsed = current_time - start_time_;

//         if (current_state_ == State::MOVE_X_POSITIVE) {
//             moveInDirection(linear_velocity_);
//             if (elapsed > motion_duration_) {
//                 RCLCPP_INFO(this->get_logger(), ">> 5s Finished. Reversing to -X");
//                 start_time_ = this->now();
//                 current_state_ = State::MOVE_X_NEGATIVE;
//             }
//         } 
//         else if (current_state_ == State::MOVE_X_NEGATIVE) {
//             moveInDirection(-linear_velocity_);
//             if (elapsed > motion_duration_) {
//                 moveInDirection(0.0);
//                 current_state_ = State::COMPLETE;
//                 RCLCPP_INFO(this->get_logger(), ">> Sequence Complete.");
//             }
//         }
//     }

//     void moveInDirection(double vx) {
//         auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
//         twist_msg->header.stamp = this->now();
//         twist_msg->header.frame_id = PLANNING_FRAME_ID;
//         twist_msg->twist.linear.x = vx;
//         twist_pub_->publish(std::move(twist_msg));
//     }

//     void switchToTwistMode() {
//         RCLCPP_INFO(this->get_logger(), "Attempting to switch Servo mode...");
        
//         if (!switch_client_->wait_for_service(std::chrono::seconds(2))) {
//             RCLCPP_ERROR(this->get_logger(), "Service /servo_node/switch_command_type not available!");
//             return;
//         }

//         auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
//         request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;

//         switch_client_->async_send_request(request, 
//             [this](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture future) {
//                 auto response = future.get();
//                 if (response->success) {
//                     RCLCPP_INFO(this->get_logger(), "Successfully switched to TWIST. ROBOT SHOULD MOVE NOW.");
//                     this->start_time_ = this->now();
//                     this->is_ready_ = true;
//                 } else {
//                     RCLCPP_ERROR(this->get_logger(), "Robot REJECTED the switch request.");
//                 }
//             });
//     }

//     rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
//     rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
//     rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_client_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::TimerBase::SharedPtr trigger_timer_;
    
//     State current_state_;
//     bool is_ready_;
//     double linear_velocity_;
//     std::chrono::nanoseconds motion_duration_;
//     rclcpp::Time start_time_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<AutomatedSlowMotion>();
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     executor.spin();
//     rclcpp::shutdown();
//     return 0;
// }




// stras movement at -x 


#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>

namespace {
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string SWITCH_SERVICE = "/servo_node/switch_command_type";
const std::string PLANNING_FRAME_ID = "base_link";
}

class AutomatedSlowMotion : public rclcpp::Node {
public:
    AutomatedSlowMotion() : Node("automated_slow_motion"), is_ready_(false) {
        // Use Reliability RELIABLE but Durability VOLATILE to ensure delivery without backlog
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        // auto qos = rclcpp::QoS(1).best_effort().durability_volatile();

        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, qos);
        joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, qos);
        switch_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(SWITCH_SERVICE);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // Faster update (50Hz) helps MoveIt stay "awake"
            std::bind(&AutomatedSlowMotion::timerCallback, this));

        linear_velocity_ = 0.1; 
        motion_duration_ = std::chrono::seconds(4);
        current_state_ = State::MOVE_X_POSITIVE;

        // Give the system a tiny moment to discover the service, then call it
        trigger_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() {
                this->switchToTwistMode();
                this->trigger_timer_->cancel(); 
            });
    }

private:
    enum class State { MOVE_X_POSITIVE, MOVE_X_NEGATIVE, COMPLETE };

    void timerCallback() {
        if (!is_ready_ || current_state_ == State::COMPLETE) {
            return;
        }

        auto current_time = this->now();
        auto elapsed = current_time - start_time_;

        if (current_state_ == State::MOVE_X_POSITIVE) {
            sendTwist(linear_velocity_);
            if (elapsed > motion_duration_) {
                RCLCPP_INFO(this->get_logger(), ">> Reversing to -X");
                start_time_ = this->now();
                current_state_ = State::MOVE_X_NEGATIVE;
            }
        } 
        else if (current_state_ == State::MOVE_X_NEGATIVE) {
            sendTwist(-linear_velocity_);
            if (elapsed > motion_duration_) {
                RCLCPP_INFO(this->get_logger(), ">> Sequence Complete.");
                sendTwist(0.0); // Final stop
                current_state_ = State::COMPLETE;
                timer_->cancel();
            }
        }
    }

    void sendTwist(double vx) {
        auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        // msg->header.stamp = this->get_clock()->now(); // donot use this will make robot to start aftr success msg
        msg->header.stamp = this->now(); // CRITICAL: Must be the EXACT current time
        msg->header.frame_id = PLANNING_FRAME_ID;
        msg->twist.linear.x = vx;
        twist_pub_->publish(std::move(msg));
    }

    void switchToTwistMode() {
        if (!switch_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available!");
            return;
        }

        auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
        request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;

        switch_client_->async_send_request(request, 
            [this](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture future) {
                if (future.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "TWIST MODE ACTIVE.");
                    this->start_time_ = this->now();
                    this->is_ready_ = true;
                }
            });
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr trigger_timer_;
    
    State current_state_;
    bool is_ready_;
    double linear_velocity_;
    std::chrono::nanoseconds motion_duration_;
    rclcpp::Time start_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutomatedSlowMotion>();
    
    // Using SingleThreadedExecutor sometimes helps with simple logic synchronization
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



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
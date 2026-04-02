
// //      
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "moveit_msgs/srv/servo_command_type.hpp"
#include "json.hpp" 

using json = nlohmann::json;
using namespace std::chrono_literals;

class SensagramServoNode : public rclcpp::Node {
public:
    SensagramServoNode() : Node("sensagram_gyro_servo"), is_ready_(false) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", qos);
        switch_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>("/servo_node/switch_command_type");

        setup_udp(5005);

        // Main control loop at 50Hz
        timer_ = this->create_wall_timer(20ms, std::bind(&SensagramServoNode::timer_callback, this));

        activate_servo();
    }

private:
    void setup_udp(int port) {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        struct sockaddr_in servaddr;
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(port);
        
        bind(sockfd_, (const struct sockaddr *)&servaddr, sizeof(servaddr));
        fcntl(sockfd_, F_SETFL, O_NONBLOCK);
        RCLCPP_INFO(this->get_logger(), "UDP Listening on port %d", port);
    }


    void timer_callback() {
        if (!is_ready_) return;

        struct sockaddr_in cliaddr;
        socklen_t len = sizeof(cliaddr);
        char buffer[2048];
        bool data_received = false;

        // Flush buffer to get only the LATEST packet
        while (recvfrom(sockfd_, (char *)buffer, 2048, 0, (struct sockaddr *)&cliaddr, &len) > 0) {
            last_received_data_ = std::string(buffer);
            data_received = true;
        }

        if (data_received) {
            try {
                auto j = json::parse(last_received_data_);
                
                // SAFETY CHECK: Does "values" exist and is it an array with 3 items?
                if (j.contains("values") && j["values"].is_array() && j["values"].size() >= 3) {
                    
                    double gx = j["values"][0];
                    double gy = j["values"][1];
                    double gz = j["values"][2];

                    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
                    msg->header.stamp = this->now();
                    msg->header.frame_id = "base_link";

                    // Mapping Hand Gyro -> Linear Velocity
                    msg->twist.linear.x = clamp(apply_deadzone(gy) * 0.15, 0.3); 
                    msg->twist.linear.y = clamp(apply_deadzone(gx) * 0.15, 0.3); 
                    msg->twist.linear.z = clamp(apply_deadzone(gz) * 0.15, 0.3); 

                    twist_pub_->publish(std::move(msg));

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Latest Cmd -> X: %.2f Y: %.2f Z: %.2f", 
                        msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "Received JSON but 'values' key is missing or wrong size!");
                }

            } catch (const json::parse_error& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "JSON Parse Error: %s", e.what());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Unexpected Error: %s", e.what());
            }
        }
    }

    double apply_deadzone(double v) { return (std::abs(v) < 0.05) ? 0.0 : v; }
    double clamp(double v, double limit) { return std::max(-limit, std::min(limit, v)); }

    void activate_servo() {
        activation_timer_ = this->create_wall_timer(1s, [this]() {
            if (!switch_client_->service_is_ready()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Servo service...");
                return;
            }

            auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
            request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;

            switch_client_->async_send_request(request, 
                [this](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture future) {
                    auto result = future.get();
                    if (result->success) {
                        this->is_ready_ = true;
                        RCLCPP_INFO(this->get_logger(), "Successfully connected! MOVEIT SERVO READY.");
                        this->activation_timer_->cancel(); 
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Servo node rejected TWIST mode request.");
                    }
                });
        });
    }

    // --- Private Variables ---
    int sockfd_;
    std::string last_received_data_;
    bool is_ready_;
    
    rclcpp::TimerBase::SharedPtr activation_timer_; 
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensagramServoNode>());
    rclcpp::shutdown();
    return 0;
}





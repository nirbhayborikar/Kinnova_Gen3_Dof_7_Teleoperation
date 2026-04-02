#include "olive_imu_control/olive_teleop_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    auto node = std::make_shared<OliveTeleopNode>(options);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
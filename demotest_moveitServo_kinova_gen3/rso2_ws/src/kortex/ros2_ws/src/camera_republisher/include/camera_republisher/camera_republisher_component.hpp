#ifndef CAMERA_REPUBLISHER__CAMERA_REPUBLISHER_COMPONENT_HPP_
#define CAMERA_REPUBLISHER__CAMERA_REPUBLISHER_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <atomic>

namespace camera_republisher
{

class CameraRepublisherComponent : public rclcpp::Node
{
public:
  explicit CameraRepublisherComponent(const rclcpp::NodeOptions & options);

private:
  // Callbacks for images (no synchronization needed)
  void cameraImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image);
  void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & depth_image);
  
  // Callbacks for camera info (cache the first one received)
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info);
  void depthInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & depth_info);

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;

  // Simple subscribers (no message_filters needed)
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;

  // Cached camera info messages (set once, reused forever)
  sensor_msgs::msg::CameraInfo::SharedPtr cached_camera_info_;
  sensor_msgs::msg::CameraInfo::SharedPtr cached_depth_info_;
  std::atomic<bool> camera_info_received_{false};
  std::atomic<bool> depth_info_received_{false};
};

}  // namespace camera_republisher

#endif  // CAMERA_REPUBLISHER__CAMERA_REPUBLISHER_COMPONENT_HPP_
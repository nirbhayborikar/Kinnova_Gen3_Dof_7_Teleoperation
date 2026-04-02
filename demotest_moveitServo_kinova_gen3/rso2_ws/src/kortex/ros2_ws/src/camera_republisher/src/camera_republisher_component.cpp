#include "camera_republisher/camera_republisher_component.hpp"

namespace camera_republisher
{

CameraRepublisherComponent::CameraRepublisherComponent(const rclcpp::NodeOptions & options)
: Node("camera_republisher", options)
{
  // Use fixed topic names - remapping in launch file will handle routing
  std::string input_camera_topic = "camera/image_raw";
  std::string input_camera_info_topic = "camera/camera_info";
  std::string input_depth_topic = "depth/image_raw";
  std::string input_depth_info_topic = "depth/camera_info";
  std::string output_camera_topic = "camera_out/image_raw";
  std::string output_camera_info_topic = "camera_out/camera_info";
  std::string output_depth_topic = "camera_out/depth/image_raw";
  std::string output_depth_info_topic = "camera_out/depth/camera_info";

  // Create publishers with RELIABLE QoS to match most camera drivers
  auto qos_profile = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
  
  camera_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    output_camera_topic, qos_profile);
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    output_camera_info_topic, qos_profile);
  depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    output_depth_topic, qos_profile);
  depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    output_depth_info_topic, qos_profile);

  // Create simple subscribers (no synchronization)
  auto reliable_qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
  
  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_camera_topic, reliable_qos,
    std::bind(&CameraRepublisherComponent::cameraImageCallback, this, std::placeholders::_1));
    
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    input_camera_info_topic, reliable_qos,
    std::bind(&CameraRepublisherComponent::cameraInfoCallback, this, std::placeholders::_1));
    
  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_depth_topic, reliable_qos,
    std::bind(&CameraRepublisherComponent::depthImageCallback, this, std::placeholders::_1));
    
  depth_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    input_depth_info_topic, reliable_qos,
    std::bind(&CameraRepublisherComponent::depthInfoCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Camera Republisher Component initialized");
  RCLCPP_INFO(this->get_logger(), "Using remapping for topic routing - check launch file for actual topic names");
  RCLCPP_INFO(this->get_logger(), "Camera info will be cached from first message and reused");
}

void CameraRepublisherComponent::cameraImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image)
{
  // Republish image immediately
  auto image_copy = std::make_unique<sensor_msgs::msg::Image>(*image);
  camera_pub_->publish(std::move(image_copy));
  
  // If we have cached camera info, republish it too
  if (camera_info_received_.load()) {
    auto info_copy = std::make_unique<sensor_msgs::msg::CameraInfo>(*cached_camera_info_);
    // Update timestamp to match the image
    info_copy->header.stamp = image->header.stamp;
    camera_info_pub_->publish(std::move(info_copy));
    RCLCPP_DEBUG(this->get_logger(), "Republished camera image and info");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Republished camera image (waiting for camera info)");
  }
}

void CameraRepublisherComponent::depthImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_image)
{
  // Republish depth image immediately
  auto depth_copy = std::make_unique<sensor_msgs::msg::Image>(*depth_image);
  depth_pub_->publish(std::move(depth_copy));
  
  // If we have cached depth info, republish it too
  if (depth_info_received_.load()) {
    auto info_copy = std::make_unique<sensor_msgs::msg::CameraInfo>(*cached_depth_info_);
    // Update timestamp to match the depth image
    info_copy->header.stamp = depth_image->header.stamp;
    depth_info_pub_->publish(std::move(info_copy));
    RCLCPP_DEBUG(this->get_logger(), "Republished depth image and info");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Republished depth image (waiting for depth info)");
  }
}

void CameraRepublisherComponent::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info)
{
  if (!camera_info_received_.load()) {
    // Cache the first camera info message
    cached_camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
    camera_info_received_.store(true);
    RCLCPP_INFO(this->get_logger(), "Cached camera info - will reuse for all future images");
  }
  // Ignore subsequent camera info messages since calibration doesn't change
}

void CameraRepublisherComponent::depthInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & depth_info)
{
  if (!depth_info_received_.load()) {
    // Cache the first depth info message
    cached_depth_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*depth_info);
    depth_info_received_.store(true);
    RCLCPP_INFO(this->get_logger(), "Cached depth info - will reuse for all future images");
  }
  // Ignore subsequent depth info messages since calibration doesn't change
}

}  // namespace camera_republisher

RCLCPP_COMPONENTS_REGISTER_NODE(camera_republisher::CameraRepublisherComponent)
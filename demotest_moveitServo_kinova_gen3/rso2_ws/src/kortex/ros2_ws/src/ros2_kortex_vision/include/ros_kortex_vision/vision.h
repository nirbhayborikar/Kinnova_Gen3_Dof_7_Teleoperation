#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace CameraTypes {
enum CameraType {
  Unknown = 0,
  Color = 1,
  Depth = 2,
};
}

namespace ros_kortex_vision {

struct FrameData {
  std::shared_ptr<sensor_msgs::msg::Image> image;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info;
  rclcpp::Time timestamp;
  std::atomic<bool> ready{false};
};

class Vision : public rclcpp::Node {
public:
  explicit Vision(const rclcpp::NodeOptions &options);
  ~Vision();

  // Public methods for external access
  void quit();

private:
  bool configure();
  bool initialize();
  bool start();
  bool loadCameraInfo();
  void stop();
  bool changePipelineState(GstState state);

  // Threading functions
  void gstreamerLoop();
  void publisherLoop();
  bool processFrame(GstSample *sample);
  void publishLatestFrame();
  void printPerformanceStats();
  void setupQoS();

private:
  // Core ROS2 components with optimized QoS and image_transport
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  // For color images: image_transport (supports compression)
  std::unique_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::Publisher image_publisher_;

  // For depth images: regular publisher (16UC1 doesn't compress well)
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_raw_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_publisher_;

  // Threading components
  std::thread gstreamer_thread_;
  std::thread publisher_thread_;
  std::atomic<bool> running_;
  std::atomic<bool> quit_requested_;

  // Lock-free frame buffer (triple buffering for zero-copy)
  static constexpr size_t FRAME_BUFFER_SIZE = 3;
  std::array<FrameData, FRAME_BUFFER_SIZE> frame_buffer_;
  std::atomic<size_t> write_index_;
  std::atomic<size_t> read_index_;
  std::atomic<bool> has_new_frame_;

  // GStreamer components
  GstElement *gst_pipeline_;
  GstElement *gst_sink_;
  std::atomic<bool> gst_initialized_;

  // Configuration
  std::string camera_config_;
  std::string camera_name_;
  std::string camera_info_;
  std::string frame_id_;
  std::string image_encoding_;
  std::string base_frame_id_;
  int retry_count_;
  int camera_type_;
  double time_offset_;
  int image_width_;
  int image_height_;
  int pixel_size_;
  bool use_gst_timestamps_;

  // Performance monitoring
  std::atomic<size_t> frames_dropped_;
  std::atomic<size_t> frames_published_;
  rclcpp::Time last_stats_time_;

  // Publisher rate limiting
  double max_pub_rate_hz_;
  std::chrono::steady_clock::time_point last_publish_time_;

  // QoS profiles for zero-copy performance
  rclcpp::QoS image_qos_;
  rclcpp::QoS camera_info_qos_;
};

} // namespace ros_kortex_vision

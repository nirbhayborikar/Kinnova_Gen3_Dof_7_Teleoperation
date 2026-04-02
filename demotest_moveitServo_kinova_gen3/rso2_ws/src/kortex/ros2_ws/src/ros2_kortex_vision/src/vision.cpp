#include <ros_kortex_vision/vision.h>

#include <cstring>
#include <stdio.h>

namespace {
constexpr auto NODE_NAME = "kinova_vision";
constexpr auto CAM_INFO_DEFAULT_URL_MAX_SIZE = 128;

constexpr auto DEFAULT_BASE_FRAME_ID = "camera_link";
constexpr auto DEFAULT_DEPTH_FRAME_ID = "camera_depth_frame";
constexpr auto DEFAULT_COLOR_FRAME_ID = "camera_color_frame";

const int RETRY_INTERVAL = 3;
const unsigned int STATE_CHANGE_ASYNC_TIMEOUT = 15;

// ROS Parameter names
const std::string STREAM_CONFIG_PARAM = "stream_config";
const std::string CAMERA_TYPE_PARAM = "camera_type";
const std::string CAMERA_NAME_PARAM = "camera_name";
const std::string FRAME_ID_PARAM = "frame_id";
const std::string CAMERA_INFO_URL_USER_PARAM = "camera_info_url_user";
const std::string CAMERA_INFO_URL_DEFAULT_PARAM = "camera_info_url_default";
const std::string MAX_PUB_RATE_PARAM = "max_pub_rate";
} // namespace

namespace ros_kortex_vision {

Vision::Vision(const rclcpp::NodeOptions &options)
    : Node(NODE_NAME, options),
      camera_info_manager_(
          std::make_shared<camera_info_manager::CameraInfoManager>(this)),
      running_(false), quit_requested_(false), write_index_(0), read_index_(0),
      has_new_frame_(false), gst_pipeline_(nullptr), gst_sink_(nullptr),
      gst_initialized_(false), base_frame_id_(DEFAULT_BASE_FRAME_ID),
      retry_count_(0), camera_type_(CameraTypes::Unknown), time_offset_(0),
      image_width_(0), image_height_(0), pixel_size_(0),
      use_gst_timestamps_(false), frames_dropped_(0), frames_published_(0),
      max_pub_rate_hz_(30.0), image_qos_(1) // Initialize with depth of 1
      ,
      camera_info_qos_(1) // Initialize with depth of 1
{
  last_stats_time_ = this->now();
  last_publish_time_ = std::chrono::steady_clock::now();

  // Configure and start immediately if this is a component
  if (configure()) {
    setupQoS();
    if (initialize() && start()) {
      loadCameraInfo();
      running_ = true;

      // Start threads
      gstreamer_thread_ = std::thread(&Vision::gstreamerLoop, this);
      publisher_thread_ = std::thread(&Vision::publisherLoop, this);

      RCLCPP_INFO(this->get_logger(),
                  "Optimized vision component started successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to start vision component");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure vision component");
  }
}

Vision::~Vision() { quit(); }

void Vision::setupQoS() {
  image_qos_ =
      rclcpp::QoS(1) // Depth of 1 for minimal queue
          .reliability(rclcpp::ReliabilityPolicy::Reliable)
          .durability(rclcpp::DurabilityPolicy::Volatile) // No historical data
          .history(rclcpp::HistoryPolicy::KeepLast);      // Only keep latest

  camera_info_qos_ = rclcpp::QoS(1) // Depth of 1 for minimal queue
                         .reliability(rclcpp::ReliabilityPolicy::Reliable)
                         .durability(rclcpp::DurabilityPolicy::Volatile)
                         .history(rclcpp::HistoryPolicy::KeepLast);

  // DIFFERENT HANDLING FOR DEPTH vs COLOR
  if (camera_type_ == CameraTypes::Color) {
    // Color images: Use image_transport for automatic compression
    try {
      image_transport_ =
          std::make_unique<image_transport::ImageTransport>(shared_from_this());
    } catch (const std::bad_weak_ptr &e) {
      // Fallback for composable nodes
      image_transport_ = std::make_unique<image_transport::ImageTransport>(
          std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {}));
    }

    auto qos_profile = image_qos_.get_rmw_qos_profile();
    image_publisher_ = image_transport_->advertise("image_raw", qos_profile);

  } else {
    // Depth images: Use regular publisher (16UC1 doesn't compress well anyway)
    image_publisher_raw_ = this->create_publisher<sensor_msgs::msg::Image>(
        "image_raw", image_qos_);

    RCLCPP_INFO(this->get_logger(),
                "Using raw publisher for depth images (16UC1 format)");
  }

  camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", camera_info_qos_);
}

bool Vision::configure() {
  this->declare_parameter<std::string>(STREAM_CONFIG_PARAM);
  if (!this->get_parameter<std::string>(STREAM_CONFIG_PARAM, camera_config_)) {
    RCLCPP_FATAL(this->get_logger(),
                 "'%s' parameter is not set. This is needed to set up a "
                 "gstreamer pipeline.",
                 STREAM_CONFIG_PARAM.c_str());
    return false;
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Using gstreamer config: \"" << camera_config_ << "\"");
  }

  std::string camera_type;
  this->declare_parameter<std::string>(CAMERA_TYPE_PARAM);
  if (!this->get_parameter<std::string>(CAMERA_TYPE_PARAM, camera_type)) {
    RCLCPP_FATAL(this->get_logger(),
                 "'%s' parameter is not set. This param is required!",
                 CAMERA_TYPE_PARAM.c_str());
    return false;
  }

  // Set encoding related to camera type
  if (camera_type == "color") {
    camera_type_ = CameraTypes::Color;
    image_encoding_ = sensor_msgs::image_encodings::RGB8;
  } else if (camera_type == "depth") {
    camera_type_ = CameraTypes::Depth;
    image_encoding_ = sensor_msgs::image_encodings::TYPE_16UC1;
  } else {
    RCLCPP_FATAL(this->get_logger(),
                 "%s: '%s' is invalid! Must be 'color' or 'depth'",
                 CAMERA_TYPE_PARAM.c_str(), camera_type.c_str());
    return false;
  }

  pixel_size_ = sensor_msgs::image_encodings::numChannels(image_encoding_) *
                (sensor_msgs::image_encodings::bitDepth(image_encoding_) / 8);

  this->declare_parameter<std::string>(CAMERA_NAME_PARAM, "camera");
  this->get_parameter<std::string>(CAMERA_NAME_PARAM, camera_name_);
  camera_info_manager_->setCameraName(camera_name_);

  this->declare_parameter<std::string>(FRAME_ID_PARAM, "camera_frame");
  this->get_parameter<std::string>(FRAME_ID_PARAM, frame_id_);

  this->declare_parameter<double>(MAX_PUB_RATE_PARAM, 30.0);
  this->get_parameter<double>(MAX_PUB_RATE_PARAM, max_pub_rate_hz_);

  RCLCPP_INFO(this->get_logger(), "Camera: %s, Frame: %s, Rate: %.1f Hz",
              camera_name_.c_str(), frame_id_.c_str(), max_pub_rate_hz_);

  return true;
}

bool Vision::initialize() {
  if (!gst_is_initialized()) {
    gst_init(0, 0);
  }

  GError *error = nullptr;
  gst_pipeline_ = gst_parse_launch(camera_config_.c_str(), &error);
  if (!gst_pipeline_) {
    RCLCPP_FATAL(this->get_logger(), "GStreamer pipeline creation failed: %s",
                 error ? error->message : "Unknown error");
    if (error)
      g_error_free(error);
    return false;
  }

  // Create optimized appsink
  gst_sink_ = gst_element_factory_make("appsink", nullptr);
  if (!gst_sink_) {
    RCLCPP_FATAL(this->get_logger(), "Failed to create appsink");
    return false;
  }

  // Configure appsink
  g_object_set(G_OBJECT(gst_sink_), "max-buffers", 1, // Only 1 buffer
               "max-bytes", 0,                        // No byte limit
               "max-time", 0,                         // No time limit
               "drop", TRUE,                // Drop frames aggressively
               "sync", FALSE,               // Never sync to clock
               "async", FALSE,              // Don't wait for state changes
               "emit-signals", FALSE,       // No signals (faster)
               "enable-last-sample", FALSE, // Don't keep last sample
               "throttle-time", 0,          // No throttling
               nullptr);

  // Set up caps for the specific encoding
  std::string gst_encoding =
      (image_encoding_ == sensor_msgs::image_encodings::TYPE_16UC1)
          ? "GRAY16_LE"
          : "RGB";

  GstCaps *caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING,
                                      gst_encoding.c_str(), nullptr);
  gst_app_sink_set_caps(GST_APP_SINK(gst_sink_), caps);
  gst_caps_unref(caps);

  // Link pipeline elements
  if (GST_IS_PIPELINE(gst_pipeline_)) {
    GstPad *outpad =
        gst_bin_find_unlinked_pad(GST_BIN(gst_pipeline_), GST_PAD_SRC);
    if (!outpad) {
      RCLCPP_FATAL(this->get_logger(), "No unlinked source pad found");
      return false;
    }

    GstElement *outelement = gst_pad_get_parent_element(outpad);
    gst_object_unref(outpad);

    if (!gst_bin_add(GST_BIN(gst_pipeline_), gst_sink_) ||
        !gst_element_link(outelement, gst_sink_)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to link pipeline elements");
      gst_object_unref(outelement);
      return false;
    }
    gst_object_unref(outelement);
  }

  // Calibration between ROS node time and GST timestamps
  GstClock *clock = gst_system_clock_obtain();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  auto now = this->now();
  time_offset_ =
      now.seconds() - static_cast<double>(GST_TIME_AS_USECONDS(ct)) / 1e6;

  gst_initialized_ = true;
  return true;
}

bool Vision::start() {
  if (!gst_pipeline_)
    return false;

  GstStateChangeReturn ret =
      gst_element_set_state(gst_pipeline_, GST_STATE_PLAYING);

  switch (ret) {
  case GST_STATE_CHANGE_FAILURE:
  case GST_STATE_CHANGE_NO_PREROLL:
    RCLCPP_ERROR(this->get_logger(), "Failed to start stream");
    return false;

  case GST_STATE_CHANGE_ASYNC:
    ret = gst_element_get_state(gst_pipeline_, nullptr, nullptr,
                                STATE_CHANGE_ASYNC_TIMEOUT * GST_SECOND);
    if (ret == GST_STATE_CHANGE_FAILURE || ret == GST_STATE_CHANGE_ASYNC) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to start stream (async/timeout)");
      return false;
    }
    [[fallthrough]];

  case GST_STATE_CHANGE_SUCCESS:
    RCLCPP_INFO(this->get_logger(), "Stream started successfully");
    break;

  default:
    return false;
  }

  // Get frame dimensions
  GstPad *pad = gst_element_get_static_pad(gst_sink_, "sink");
  const GstCaps *caps = gst_pad_get_current_caps(pad);
  if (caps) {
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(structure, "width", &image_width_);
    gst_structure_get_int(structure, "height", &image_height_);
    gst_object_unref(pad);
  }

  return true;
}
void Vision::gstreamerLoop() {
  RCLCPP_INFO(this->get_logger(), "Starting GStreamer capture thread");

  while (running_ && !quit_requested_) {

    GstSample *sample =
        gst_app_sink_try_pull_sample(GST_APP_SINK(gst_sink_), GST_SECOND / 10);
    if (!sample) {
      continue; // No sample available, continue (non-blocking)
    }

    if (processFrame(sample)) {
      has_new_frame_ = true;
    } else {
      frames_dropped_++;
    }

    gst_sample_unref(sample);

    // Yield CPU more frequently to be nice to other processes
    std::this_thread::yield();
  }

  RCLCPP_INFO(this->get_logger(), "GStreamer capture thread stopped");
}

bool Vision::processFrame(GstSample *sample) {
  GstBuffer *buf = gst_sample_get_buffer(sample);
  if (!buf)
    return false;

  GstMapInfo map;
  if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
    return false;
  }

  // Get frame dimensions from caps if not already set
  if (image_width_ == 0 || image_height_ == 0) {
    const GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(structure, "width", &image_width_);
    gst_structure_get_int(structure, "height", &image_height_);

    // VALIDATION: Check for reasonable dimensions
    if (image_width_ <= 0 || image_height_ <= 0 || image_width_ > 4096 ||
        image_height_ > 4096) {
      RCLCPP_ERROR(this->get_logger(), "Invalid image dimensions: %dx%d",
                   image_width_, image_height_);
      gst_buffer_unmap(buf, &map);
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Detected image size: %dx%d for %s camera",
                image_width_, image_height_, camera_name_.c_str());
  }

  // Calculate expected frame size
  unsigned int expected_frame_size = image_width_ * image_height_ * pixel_size_;

  if (map.size != expected_frame_size) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Frame size mismatch: expected %u, got %zu",
                         expected_frame_size, map.size);
    gst_buffer_unmap(buf, &map);
    return false;
  }

  // VALIDATION: Check for reasonable frame size
  if (map.size == 0 || map.size > 50 * 1024 * 1024) { // 50MB max
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Suspicious frame size: %zu bytes", map.size);
    gst_buffer_unmap(buf, &map);
    return false;
  }

  // Lock-free write to circular buffer
  size_t write_idx = write_index_.load();
  size_t next_write_idx = (write_idx + 1) % FRAME_BUFFER_SIZE;

  auto &frame_data = frame_buffer_[write_idx];

  // Create image message
  frame_data.image = std::make_shared<sensor_msgs::msg::Image>();
  frame_data.image->header.stamp = this->now();
  frame_data.image->header.frame_id = frame_id_;
  frame_data.image->width = image_width_;
  frame_data.image->height = image_height_;
  frame_data.image->encoding = image_encoding_;
  frame_data.image->is_bigendian = false;
  frame_data.image->step = image_width_ * pixel_size_;

  // Efficient memory copy with validation
  try {
    frame_data.image->data.resize(map.size);
    std::memcpy(frame_data.image->data.data(), map.data, map.size);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Memory allocation failed: %s", e.what());
    gst_buffer_unmap(buf, &map);
    return false;
  }

  // Create camera info
  frame_data.camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>(
      camera_info_manager_->getCameraInfo());
  frame_data.camera_info->header = frame_data.image->header;
  frame_data.camera_info->width = image_width_;
  frame_data.camera_info->height = image_height_;

  frame_data.timestamp = frame_data.image->header.stamp;
  frame_data.ready = true;

  gst_buffer_unmap(buf, &map);

  // Atomically update write index
  write_index_.store(next_write_idx);

  return true;
}

void Vision::publisherLoop() {
  RCLCPP_INFO(this->get_logger(), "Starting publisher thread");

  auto rate_limiter =
      std::chrono::microseconds(static_cast<int64_t>(1e6 / max_pub_rate_hz_));

  while (running_ && !quit_requested_) {
    if (has_new_frame_.load()) {
      publishLatestFrame();
      has_new_frame_ = false;
    }

    // Rate limiting with high precision
    auto now = std::chrono::steady_clock::now();
    auto time_since_last = now - last_publish_time_;
    if (time_since_last < rate_limiter) {
      std::this_thread::sleep_for(rate_limiter - time_since_last);
    }
    last_publish_time_ = std::chrono::steady_clock::now();

    // Periodic stats
    printPerformanceStats();
  }

  RCLCPP_INFO(this->get_logger(), "Publisher thread stopped");
}

void Vision::publishLatestFrame() {
  size_t read_idx = read_index_.load();
  size_t write_idx = write_index_.load();

  if (read_idx == write_idx) {
    return; // No new frames
  }

  auto &frame_data = frame_buffer_[read_idx];

  if (frame_data.ready && frame_data.image && frame_data.camera_info) {
    try {
      if (camera_type_ == CameraTypes::Color && image_publisher_) {
        // Color: use image_transport (automatic compression)
        image_publisher_.publish(*frame_data.image);
      } else if (camera_type_ == CameraTypes::Depth && image_publisher_raw_) {
        // Depth: use raw publisher (no compression errors)
        image_publisher_raw_->publish(*frame_data.image);
      }

      camera_info_publisher_->publish(*frame_data.camera_info);

      frames_published_++;
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Failed to publish frame: %s", e.what());
    }

    frame_data.ready = false;
  }

  // Update read index
  read_index_.store((read_idx + 1) % FRAME_BUFFER_SIZE);
}

void Vision::printPerformanceStats() {
  auto now = this->now();
  if ((now - last_stats_time_).seconds() > 10.0) // Every 10 seconds
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s] Performance: Published %zu frames, Dropped %zu frames",
                camera_name_.c_str(), frames_published_.load(),
                frames_dropped_.load());
    frames_published_ = 0;
    frames_dropped_ = 0;
    last_stats_time_ = now;
  }
}

bool Vision::loadCameraInfo() {
  std::string cam_info_default;
  char cam_info_default_resolved[CAM_INFO_DEFAULT_URL_MAX_SIZE];

  this->declare_parameter<std::string>(CAMERA_INFO_URL_USER_PARAM, "");
  this->get_parameter<std::string>(CAMERA_INFO_URL_USER_PARAM, camera_info_);

  if (camera_info_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Custom camera information file not set, "
                                    "using default based on resolution");

    this->declare_parameter<std::string>(CAMERA_INFO_URL_DEFAULT_PARAM, "");
    this->get_parameter<std::string>(CAMERA_INFO_URL_DEFAULT_PARAM,
                                     cam_info_default);

    if (!cam_info_default.empty()) {
      snprintf(cam_info_default_resolved, CAM_INFO_DEFAULT_URL_MAX_SIZE,
               cam_info_default.c_str(), image_width_, image_height_);
      camera_info_.assign(cam_info_default_resolved);
    }
  }

  if (camera_info_manager_->validateURL(camera_info_)) {
    if (camera_info_manager_->loadCameraInfo(camera_info_)) {
      RCLCPP_INFO(this->get_logger(), "Loaded camera calibration from '%s'",
                  camera_info_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Camera info at '%s' not found. Using uncalibrated config.",
                  camera_info_.c_str());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Camera info URL syntax not supported.");
  }

  return true;
}

void Vision::stop() {
  running_ = false;

  if (gst_pipeline_) {
    changePipelineState(GST_STATE_PAUSED);
    changePipelineState(GST_STATE_READY);
    changePipelineState(GST_STATE_NULL);

    gst_object_unref(gst_pipeline_);
    gst_pipeline_ = nullptr;
  }
}

bool Vision::changePipelineState(GstState state) {
  if (!gst_pipeline_)
    return false;

  GstStateChangeReturn ret = gst_element_set_state(gst_pipeline_, state);
  if (GST_STATE_CHANGE_ASYNC == ret) {
    ret = gst_element_get_state(gst_pipeline_, nullptr, nullptr,
                                STATE_CHANGE_ASYNC_TIMEOUT * GST_SECOND);
  }

  switch (ret) {
  case GST_STATE_CHANGE_SUCCESS:
  case GST_STATE_CHANGE_NO_PREROLL:
    return true;

  case GST_STATE_CHANGE_FAILURE:
    RCLCPP_ERROR(this->get_logger(), "Failed to change pipeline state to %s",
                 gst_element_state_get_name(state));
    return false;

  case GST_STATE_CHANGE_ASYNC:
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to change pipeline state to %s (timeout)",
                 gst_element_state_get_name(state));
    return false;

  default:
    RCLCPP_ERROR(this->get_logger(),
                 "Unknown state change return value when trying to change "
                 "pipeline state to %s",
                 gst_element_state_get_name(state));
    return false;
  }
}

void Vision::quit() {
  if (quit_requested_)
    return;

  quit_requested_ = true;
  running_ = false;

  RCLCPP_INFO(this->get_logger(), "Shutting down vision component");

  if (gst_pipeline_) {
    // Send EOS to unblock gst_app_sink_pull_sample()
    GstEvent *event = gst_event_new_eos();
    gst_element_send_event(gst_pipeline_, event);
  }

  // Wait for threads to finish
  if (gstreamer_thread_.joinable()) {
    gstreamer_thread_.join();
  }
  if (publisher_thread_.joinable()) {
    publisher_thread_.join();
  }

  stop();

  RCLCPP_INFO(this->get_logger(), "Vision component shutdown complete");
}

} // namespace ros_kortex_vision

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros_kortex_vision::Vision)

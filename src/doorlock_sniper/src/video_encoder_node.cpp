#include "doorlock_sniper/video_encoder_node.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>  // 为 memcpy/memset
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace doorlock_sniper
{

VideoEncoderNode::VideoEncoderNode(const rclcpp::NodeOptions & options)
: Node("video_encoder_node", options),
  pipeline_(nullptr),
  appsrc_(nullptr),
  appsink_(nullptr),
  bus_(nullptr),
  packet_sequence_id_(0),    // 初始化顺序与声明一致
  frame_count_(0),
  display_running_(false)    // 最后初始化
{
  constexpr int kVideoPacketBytes = 300;

  param_input_topic_ = this->declare_parameter("input_topic", "/image_raw");
  param_crop_size_ = this->declare_parameter("crop_size", 800);
  param_output_size_ = this->declare_parameter("output_size", 400);
  param_output_fps_ = this->declare_parameter("output_fps", 60);
  param_target_bitrate_ = this->declare_parameter("target_bitrate", 40);
  param_packet_size_ = this->declare_parameter("packet_size", kVideoPacketBytes);
  param_static_simplify_ = this->declare_parameter("static_simplify", true);
  param_motion_threshold_ = this->declare_parameter("motion_threshold", 14);
  param_motion_erode_px_ = this->declare_parameter("motion_erode_px", 1);
  param_motion_dilate_px_ = this->declare_parameter("motion_dilate_px", 2);
  param_motion_trail_frames_ = this->declare_parameter("motion_trail_frames", 3);
  param_trail_disable_motion_ratio_ = this->declare_parameter("trail_disable_motion_ratio", 0.30);
  param_bg_update_alpha_ = this->declare_parameter("bg_update_alpha", 0.01);
  param_bg_blur_sigma_ = this->declare_parameter("bg_blur_sigma", 1.2);
  param_center_clear_size_ = this->declare_parameter("center_clear_size", 100);
  param_force_monochrome_ = this->declare_parameter("force_monochrome", false);
  param_bandwidth_limit_kbytes_ = this->declare_parameter("bandwidth_limit_kbytes", 7.0);
  param_bandwidth_window_s_ = this->declare_parameter("bandwidth_window_s", 2.0);
  param_max_tx_delay_s_ = this->declare_parameter("max_tx_delay_s", 1.0);
  param_enable_display_ = this->declare_parameter("enable_display", true);
  param_fixed_test_payload_mode_ = this->declare_parameter("fixed_test_payload_mode", false);
  param_x264_preset_ = this->declare_parameter("x264_preset", std::string("auto"));
  param_debug_dump_enable_ = this->declare_parameter("debug_dump_enable", false);
  param_debug_dump_every_n_frames_ = this->declare_parameter("debug_dump_every_n_frames", 20);
  param_debug_dump_save_raw_ = this->declare_parameter("debug_dump_save_raw", true);
  param_debug_dump_save_roi_ = this->declare_parameter("debug_dump_save_roi", true);
  param_debug_dump_save_static_ = this->declare_parameter("debug_dump_save_static", true);
  param_debug_dump_save_final_ = this->declare_parameter("debug_dump_save_final", true);
  param_debug_dump_dir_ = this->declare_parameter("debug_dump_dir", std::string("sniper_debug_imgs"));
  param_com_port_ = this->declare_parameter("com_port", "/dev/ttyACM0");
  param_baudrate_ = this->declare_parameter("baudrate", 921600);
  param_send_inner_packet_only_ = this->declare_parameter("send_inner_packet_only", false);

  if (param_output_fps_ < 1) {
    RCLCPP_WARN(this->get_logger(), "Invalid output_fps=%d, clamp to 1", param_output_fps_);
    param_output_fps_ = 1;
  }
  if (param_output_fps_ > 60) {
    RCLCPP_WARN(this->get_logger(), "output_fps=%d too high, clamp to 60", param_output_fps_);
    param_output_fps_ = 60;
  }

  if (param_packet_size_ != kVideoPacketBytes) {
    RCLCPP_WARN(
      this->get_logger(),
      "VideoPacket.msg payload is fixed to %d bytes, override packet_size %d -> %d",
      kVideoPacketBytes, param_packet_size_, kVideoPacketBytes);
    param_packet_size_ = kVideoPacketBytes;
  }

  if (param_target_bitrate_ < 200) {
    RCLCPP_WARN(
      this->get_logger(),
      "Very low bitrate (%d kbps) detected, using low-bitrate optimized pipeline",
      param_target_bitrate_);
  }
  if (param_motion_trail_frames_ < 0) {
    RCLCPP_WARN(
      this->get_logger(), "motion_trail_frames=%d invalid, clamp to 0",
      param_motion_trail_frames_);
    param_motion_trail_frames_ = 0;
  }
  if (param_motion_trail_frames_ > 15) {
    RCLCPP_WARN(
      this->get_logger(), "motion_trail_frames=%d too high, clamp to 15",
      param_motion_trail_frames_);
    param_motion_trail_frames_ = 15;
  }
  if (param_trail_disable_motion_ratio_ < 0.0) {
    RCLCPP_WARN(
      this->get_logger(), "trail_disable_motion_ratio=%.3f invalid, clamp to 0.0",
      param_trail_disable_motion_ratio_);
    param_trail_disable_motion_ratio_ = 0.0;
  }
  if (param_trail_disable_motion_ratio_ > 1.0) {
    RCLCPP_WARN(
      this->get_logger(), "trail_disable_motion_ratio=%.3f invalid, clamp to 1.0",
      param_trail_disable_motion_ratio_);
    param_trail_disable_motion_ratio_ = 1.0;
  }

  if (param_motion_erode_px_ < 0) {
    RCLCPP_WARN(
      this->get_logger(), "motion_erode_px=%d invalid, clamp to 0",
      param_motion_erode_px_);
    param_motion_erode_px_ = 0;
  }
  if (param_motion_erode_px_ > 20) {
    RCLCPP_WARN(
      this->get_logger(), "motion_erode_px=%d too high, clamp to 20",
      param_motion_erode_px_);
    param_motion_erode_px_ = 20;
  }
  if (param_motion_dilate_px_ < 0) {
    RCLCPP_WARN(
      this->get_logger(), "motion_dilate_px=%d invalid, clamp to 0",
      param_motion_dilate_px_);
    param_motion_dilate_px_ = 0;
  }
  if (param_motion_dilate_px_ > 20) {
    RCLCPP_WARN(
      this->get_logger(), "motion_dilate_px=%d too high, clamp to 20",
      param_motion_dilate_px_);
    param_motion_dilate_px_ = 20;
  }

  if (param_bandwidth_limit_kbytes_ < 1.0) {
    RCLCPP_WARN(
      this->get_logger(), "bandwidth_limit_kbytes=%.2f too low, clamp to 1.0",
      param_bandwidth_limit_kbytes_);
    param_bandwidth_limit_kbytes_ = 1.0;
  }
  if (param_bandwidth_window_s_ < 0.2) {
    RCLCPP_WARN(
      this->get_logger(), "bandwidth_window_s=%.2f too low, clamp to 0.2",
      param_bandwidth_window_s_);
    param_bandwidth_window_s_ = 0.2;
  }
  if (param_max_tx_delay_s_ < 0.05) {
    RCLCPP_WARN(
      this->get_logger(), "max_tx_delay_s=%.2f too low, clamp to 0.05",
      param_max_tx_delay_s_);
    param_max_tx_delay_s_ = 0.05;
  }
  if (param_debug_dump_every_n_frames_ < 1) {
    RCLCPP_WARN(
      this->get_logger(), "debug_dump_every_n_frames=%d invalid, clamp to 1",
      param_debug_dump_every_n_frames_);
    param_debug_dump_every_n_frames_ = 1;
  }
  if (param_debug_dump_enable_) {
    const bool any_encoder_save =
      param_debug_dump_save_raw_ || param_debug_dump_save_roi_ ||
      param_debug_dump_save_static_ || param_debug_dump_save_final_;
    if (!any_encoder_save) {
      RCLCPP_WARN(
        this->get_logger(),
        "debug_dump_enable=true but all encoder dump switches are off");
    } else {
      const std::filesystem::path dump_dir = std::filesystem::path(param_debug_dump_dir_) / "encoder";
      std::error_code ec;
      std::filesystem::create_directories(dump_dir, ec);
      if (ec) {
        RCLCPP_WARN(
          this->get_logger(),
          "Create debug dump dir failed: %s (%s), disable debug dump",
          dump_dir.string().c_str(), ec.message().c_str());
        param_debug_dump_enable_ = false;
      } else {
        RCLCPP_INFO(
          this->get_logger(),
          "Debug dump enabled: every %d frames -> %s (raw=%s roi=%s static=%s final=%s)",
          param_debug_dump_every_n_frames_,
          dump_dir.string().c_str(),
          param_debug_dump_save_raw_ ? "on" : "off",
          param_debug_dump_save_roi_ ? "on" : "off",
          param_debug_dump_save_static_ ? "on" : "off",
          param_debug_dump_save_final_ ? "on" : "off");
      }
    }
  }

  if (!param_fixed_test_payload_mode_) {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      param_input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&VideoEncoderNode::image_callback, this, std::placeholders::_1));
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "fixed_test_payload_mode enabled: bypass camera/GStreamer and send synthetic 0x0310 payloads");
  }

  packet_pub_ = this->create_publisher<doorlock_sniper::msg::VideoPacket>(
    "video_stream",
    rclcpp::QoS(rclcpp::KeepLast(3000)).reliable());

  if (!param_fixed_test_payload_mode_) {
    initialize_gstreamer();
  }
  initialize_communication();

  constexpr int kFixedTxHz = 48;
  const auto tx_period_us = std::chrono::microseconds(1000000 / kFixedTxHz);

  // 固定 48Hz 发送；可选发送完整 RM 外层包，或只发送 300B 内层包。
  send_timer_ = this->create_wall_timer(
    tx_period_us,
    std::bind(&VideoEncoderNode::send_packet_timer_callback, this));

  if (param_enable_display_ && !param_fixed_test_payload_mode_) {
    display_running_ = true;
    display_thread_ = std::thread(&VideoEncoderNode::display_loop, this);
  }

  RCLCPP_INFO(this->get_logger(),
    "VideoEncoderNode: crop=%d -> %dx%d@%dfps %dkbps, packets=%dbytes, static_simplify=%s, "
    "motion_open(y=%d,x=%d), trail=%df disable@%.0f%% mono=%s, "
    "tx_limit=%.2fkB/s@%.2fs max_delay=%.2fs tx_rate=%dHz tx_period=%ldus wire_packet=%zub tx_mode=%s source=%s x264_preset=%s, "
    "comm_port=%s baudrate=%d",
    param_crop_size_, param_output_size_, param_output_size_,
    param_output_fps_, param_target_bitrate_, param_packet_size_,
    param_static_simplify_ ? "on" : "off",
    param_motion_erode_px_, param_motion_dilate_px_,
    param_motion_trail_frames_, param_trail_disable_motion_ratio_ * 100.0,
    param_force_monochrome_ ? "on" : "off",
    param_bandwidth_limit_kbytes_, param_bandwidth_window_s_, param_max_tx_delay_s_,
    kFixedTxHz, tx_period_us.count(), sizeof(doorlock_sniper::CommFrame),
    param_send_inner_packet_only_ ? "inner_only" : "rm_outer",
    param_fixed_test_payload_mode_ ? "synthetic" : "hevc",
    param_x264_preset_.c_str(),
    param_com_port_.c_str(), param_baudrate_);
}

VideoEncoderNode::~VideoEncoderNode()
{
  if (param_enable_display_) {
    display_running_ = false;
    if (display_thread_.joinable()) display_thread_.join();
    cv::destroyAllWindows();
  }
  shutdown_gstreamer();
}

void VideoEncoderNode::initialize_gstreamer()
{
  gst_init(nullptr, nullptr);

  pipeline_ = gst_pipeline_new("encoder_pipe");
  appsrc_ = gst_element_factory_make("appsrc", "source");
  appsink_ = gst_element_factory_make("appsink", "sink");
  GstElement *convert = gst_element_factory_make("videoconvert", "convert");
  GstElement *encoder = gst_element_factory_make("x265enc", "encoder");
  GstElement *parser = gst_element_factory_make("h265parse", "parser");

  if (!pipeline_ || !appsrc_ || !appsink_ || !convert || !encoder || !parser) {
    RCLCPP_FATAL(this->get_logger(), "GStreamer element creation failed (x265enc requires gstreamer1.0-plugins-bad)");
    return;
  }

  GstCaps *caps = gst_caps_new_simple(
    "video/x-raw",
    "format", G_TYPE_STRING, "BGR",
    "width", G_TYPE_INT, param_output_size_,
    "height", G_TYPE_INT, param_output_size_,
    "framerate", GST_TYPE_FRACTION, param_output_fps_, 1,
    nullptr);
  g_object_set(G_OBJECT(appsrc_),
    "caps", caps,
    "stream-type", 0,
    "format", GST_FORMAT_TIME,
    "is-live", TRUE,
    "do-timestamp", TRUE,
    nullptr);
  gst_caps_unref(caps);

  const bool low_bitrate_mode = (param_target_bitrate_ <= 80);
  const int key_int = low_bitrate_mode ?
    std::max(param_output_fps_, 30) :
    std::max(param_output_fps_ / 2, 20);

  std::string preset_str = param_x264_preset_;
  if (preset_str.empty() || preset_str == "auto") {
    preset_str = low_bitrate_mode ? "slow" : "medium";
  }

  // x265enc speed-preset is enum, map name to integer value
  int speed_preset = 6; // default medium
  if (preset_str == "ultrafast") speed_preset = 1;
  else if (preset_str == "superfast") speed_preset = 2;
  else if (preset_str == "veryfast") speed_preset = 3;
  else if (preset_str == "faster") speed_preset = 4;
  else if (preset_str == "fast") speed_preset = 5;
  else if (preset_str == "medium") speed_preset = 6;
  else if (preset_str == "slow") speed_preset = 7;
  else if (preset_str == "slower") speed_preset = 8;
  else if (preset_str == "veryslow") speed_preset = 9;
  else if (preset_str == "placebo") speed_preset = 10;

  // Tune x265 for low-latency low-bitrate operation on the 0x0310 link.
  // We favor a steadier AU output rate and more frequent recovery points over
  // absolute compression efficiency so the custom client sees a smoother stream.
  g_object_set(
    G_OBJECT(encoder),
    "bitrate", param_target_bitrate_,
    "key-int-max", key_int,
    "speed-preset", speed_preset,
    "tune", 4,  // zerolatency
    "option-string",
    low_bitrate_mode ?
      "bframes=0:rc-lookahead=8:repeat-headers=1:aud=1:scenecut=0:aq-mode=2:aq-strength=1.0:info=0"
      : "bframes=0:rc-lookahead=10:repeat-headers=1:aud=1:scenecut=0:aq-mode=2:aq-strength=1.0:info=0",
    nullptr);

  // 确保下游看到可流式重组的 Annex-B 字节流，并周期重复 VPS/SPS/PPS
  g_object_set(
    G_OBJECT(parser),
    "config-interval", -1,
    "disable-passthrough", TRUE,
    nullptr);

  GstCaps *h265_caps = gst_caps_new_simple(
    "video/x-h265",
    "stream-format", G_TYPE_STRING, "byte-stream",
    "alignment", G_TYPE_STRING, "au",
    nullptr);

  g_object_set(G_OBJECT(appsink_),
    "caps", h265_caps,
    "max-buffers", 5,
    "drop", FALSE,
    "emit-signals", FALSE,
    "sync", FALSE,
    nullptr);
  gst_caps_unref(h265_caps);

  gst_bin_add_many(GST_BIN(pipeline_), appsrc_, convert, encoder, parser, appsink_, nullptr);
  if (!gst_element_link_many(appsrc_, convert, encoder, parser, appsink_, nullptr)) {
    RCLCPP_FATAL(this->get_logger(), "GStreamer pipeline link failed");
    return;
  }

  GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_FATAL(this->get_logger(), "GStreamer pipeline start failed");
    return;
  }
  
  bus_ = gst_element_get_bus(pipeline_);
  RCLCPP_INFO(
    this->get_logger(),
    "GStreamer HEVC encoder ready (%s mode, byte-stream)",
    low_bitrate_mode ? "low-bitrate" : "low-latency");
}

void VideoEncoderNode::shutdown_gstreamer()
{
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (bus_) gst_object_unref(bus_);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
}

void VideoEncoderNode::poll_gstreamer_bus()
{
  if (!bus_) return;

  while (true) {
    GstMessage *msg = gst_bus_pop(bus_);
    if (!msg) break;

    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR: {
        GError *err = nullptr;
        gchar *debug = nullptr;
        gst_message_parse_error(msg, &err, &debug);
        RCLCPP_ERROR(
          this->get_logger(),
          "GStreamer error from %s: %s (%s)",
          GST_OBJECT_NAME(msg->src),
          err ? err->message : "unknown",
          debug ? debug : "no debug");
        if (err) g_error_free(err);
        if (debug) g_free(debug);
        break;
      }
      case GST_MESSAGE_WARNING: {
        GError *err = nullptr;
        gchar *debug = nullptr;
        gst_message_parse_warning(msg, &err, &debug);
        RCLCPP_WARN(
          this->get_logger(),
          "GStreamer warning from %s: %s (%s)",
          GST_OBJECT_NAME(msg->src),
          err ? err->message : "unknown",
          debug ? debug : "no debug");
        if (err) g_error_free(err);
        if (debug) g_free(debug);
        break;
      }
      case GST_MESSAGE_EOS:
        RCLCPP_WARN(this->get_logger(), "GStreamer pipeline reached EOS");
        break;
      case GST_MESSAGE_STATE_CHANGED:
        if (GST_MESSAGE_SRC(msg) == GST_OBJECT(pipeline_)) {
          GstState old_state;
          GstState new_state;
          GstState pending_state;
          gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
          RCLCPP_INFO(
            this->get_logger(),
            "GStreamer pipeline state: %s -> %s (pending %s)",
            gst_element_state_get_name(old_state),
            gst_element_state_get_name(new_state),
            gst_element_state_get_name(pending_state));
        }
        break;
      default:
        break;
    }

    gst_message_unref(msg);
  }
}

void VideoEncoderNode::initialize_communication()
{
  try {
    comm_ = std::make_unique<DoorlockComm>(
      param_com_port_,
      param_baudrate_,
      param_send_inner_packet_only_ ?
        TxFrameMode::INNER_PACKET_ONLY :
        TxFrameMode::RM_OUTER_FRAME);
    RCLCPP_INFO(
      this->get_logger(),
      "Communication initialized successfully: %s @ %d baud (%s)",
      param_com_port_.c_str(),
      param_baudrate_,
      param_send_inner_packet_only_ ? "inner-only" : "rm-outer");
  } catch (const std::exception & e) {
    comm_.reset();
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to initialize communication: %s. Video streaming will continue without external comms.",
      e.what());
  }
}

cv::Mat VideoEncoderNode::preprocess_image(
  const cv::Mat & input,
  cv::Mat * roi_downsample,
  cv::Mat * static_removed)
{
  int x = (input.cols - param_crop_size_) / 2;
  int y = (input.rows - param_crop_size_) / 2;
  x = std::max(0, x);
  y = std::max(0, y);
  int w = std::min(param_crop_size_, input.cols - x);
  int h = std::min(param_crop_size_, input.rows - y);

  cv::Mat cropped = input(cv::Rect(x, y, w, h));
  cv::Mat resized;
  cv::resize(cropped, resized, cv::Size(param_output_size_, param_output_size_), 
             0, 0, cv::INTER_LINEAR);
  if (roi_downsample) {
    resized.copyTo(*roi_downsample);
  }
  cv::Mat working = resized;
  if (param_force_monochrome_) {
    cv::Mat gray_full;
    cv::cvtColor(working, gray_full, cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray_full, working, cv::COLOR_GRAY2BGR);
  }

  if (!param_static_simplify_) {
    if (static_removed) {
      working.copyTo(*static_removed);
    }
    return working;
  }

  cv::Mat gray;
  cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
  if (background_gray_f32_.empty()) {
    gray.convertTo(background_gray_f32_, CV_32F);
    return working;
  }

  cv::Mat bg_u8;
  cv::convertScaleAbs(background_gray_f32_, bg_u8);

  cv::Mat diff;
  cv::absdiff(gray, bg_u8, diff);

  cv::Mat motion_mask;
  cv::threshold(diff, motion_mask, param_motion_threshold_, 255, cv::THRESH_BINARY);
  if (param_motion_erode_px_ > 0) {
    if (motion_erode_kernel_.empty()) {
      const int k = 2 * param_motion_erode_px_ + 1;
      motion_erode_kernel_ = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(k, k));
    }
    cv::erode(motion_mask, motion_mask, motion_erode_kernel_, cv::Point(-1, -1), 1);
  }
  if (param_motion_dilate_px_ > 0) {
    if (motion_dilate_kernel_.empty()) {
      const int k = 2 * param_motion_dilate_px_ + 1;
      motion_dilate_kernel_ = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(k, k));
    }
    cv::dilate(motion_mask, motion_mask, motion_dilate_kernel_, cv::Point(-1, -1), 1);
  }
  const double motion_ratio_raw =
    static_cast<double>(cv::countNonZero(motion_mask)) / static_cast<double>(motion_mask.total());
  const bool suppress_trail = (motion_ratio_raw >= param_trail_disable_motion_ratio_);

  // 中心区域保护：不做静态模糊
  if (param_center_clear_size_ > 0) {
    const int clear_size = std::min({param_center_clear_size_, working.cols, working.rows});
    const int x0 = std::max(0, working.cols / 2 - clear_size / 2);
    const int y0 = std::max(0, working.rows / 2 - clear_size / 2);
    const int cw = std::min(clear_size, working.cols - x0);
    const int ch = std::min(clear_size, working.rows - y0);
    cv::rectangle(motion_mask, cv::Rect(x0, y0, cw, ch), cv::Scalar(255), cv::FILLED);
  }

  cv::Mat static_base = working.clone();
  if (!param_force_monochrome_ && param_target_bitrate_ <= 80) {
    cv::Mat gray_bg;
    cv::cvtColor(static_base, gray_bg, cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray_bg, static_base, cv::COLOR_GRAY2BGR);
  }
  cv::Mat blurred_static;
  cv::GaussianBlur(
    static_base,
    blurred_static,
    cv::Size(),
    std::max(0.0, param_bg_blur_sigma_),
    std::max(0.0, param_bg_blur_sigma_));

  cv::Mat focused = blurred_static.clone();
  working.copyTo(focused, motion_mask);
  if (static_removed) {
    focused.copyTo(*static_removed);
  }

  // 运动拖影：简单时域 max（当前+历史N帧），仅作用在运动区域联合掩码
  if (param_motion_trail_frames_ > 0) {
    motion_mask_history_.push_back(motion_mask.clone());
    trail_frame_history_.push_back(working.clone());
    const size_t max_history = static_cast<size_t>(param_motion_trail_frames_ + 1);
    while (motion_mask_history_.size() > max_history) {
      motion_mask_history_.pop_front();
    }
    while (trail_frame_history_.size() > max_history) {
      trail_frame_history_.pop_front();
    }

    const size_t history_size = motion_mask_history_.size();
    if (!suppress_trail && history_size > 1 && history_size == trail_frame_history_.size()) {
      cv::Mat trail_mask = motion_mask.clone();
      cv::Mat trail_img = working.clone();
      for (size_t i = 0; i < history_size - 1; ++i) {
        cv::bitwise_or(trail_mask, motion_mask_history_[i], trail_mask);
        cv::max(trail_img, trail_frame_history_[i], trail_img);
      }
      trail_img.copyTo(focused, trail_mask);
    }
  } else {
    motion_mask_history_.clear();
    trail_frame_history_.clear();
  }

  cv::accumulateWeighted(gray, background_gray_f32_, std::clamp(param_bg_update_alpha_, 0.001, 0.2));
  return focused;
}

void VideoEncoderNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // Always throttle encoder input to the configured output fps so the
    // GStreamer pipeline does not outrun the 0x0310 transport budget.
    const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    const int64_t frame_interval_ns = 1000000000LL / std::max(param_output_fps_, 1);
    const int64_t now_ns = (stamp_ns > 0) ? stamp_ns : this->now().nanoseconds();
    if (last_encode_stamp_ns_ > 0 && (now_ns - last_encode_stamp_ns_) < frame_interval_ns) {
      return;
    }
    last_encode_stamp_ns_ = now_ns;

    image_callback_count_++;
    const int64_t callback_now_ns = this->now().nanoseconds();
    if (callback_now_ns - last_image_log_ns_ > 1000000000LL) {
      RCLCPP_INFO(
        this->get_logger(),
        "Image callback alive: count=%lu stamp=%u.%u size=%ux%u",
        image_callback_count_,
        msg->header.stamp.sec,
        msg->header.stamp.nanosec,
        msg->width,
        msg->height);
      last_image_log_ns_ = callback_now_ns;
    }

    cv::Mat input = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat roi_downsample;
    cv::Mat static_removed;
    cv::Mat processed = preprocess_image(input, &roi_downsample, &static_removed);
    
    // 处理目标检测并发送通信命令
    process_target_detection(processed);
    
    if (param_enable_display_) {
      cv::Mat raw_preview;
      cv::resize(
        input,
        raw_preview,
        cv::Size(std::max(1, input.cols / 2), std::max(1, input.rows / 2)),
        0,
        0,
        cv::INTER_AREA);
      std::lock_guard<std::mutex> lock(frame_mutex_);
      raw_preview.copyTo(display_raw_frame_);
      roi_downsample.copyTo(display_roi_frame_);
      static_removed.copyTo(display_static_frame_);
      processed.copyTo(display_frame_);
    }
    
    push_frame_to_gstreamer(processed);
    poll_gstreamer_bus();
    pull_stream_and_packetize();
    poll_gstreamer_bus();
    
    frame_count_++;
    
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
  }
}

void VideoEncoderNode::push_frame_to_gstreamer(const cv::Mat & frame)
{
  if (!appsrc_ || frame.empty()) return;

  size_t size = frame.total() * frame.elemSize();
  GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
  
  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    memcpy(map.data, frame.data, size);
    gst_buffer_unmap(buffer, &map);
    
    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
    if (ret != GST_FLOW_OK) {
      RCLCPP_WARN(this->get_logger(), "Push buffer failed: %d", ret);
    }
  }
  gst_buffer_unref(buffer);
}

// 固定 20ms 发送一包（8字节头部 + 292字节视频数据）
void VideoEncoderNode::send_packet_timer_callback()
{
  constexpr size_t kHeaderBytes = 2 + 2 + 4;  // frame_no(2) + frag_no(2) + total_bytes(4)
  constexpr size_t kPayloadBytes = 292;       // 纯视频数据部分
  constexpr size_t kPacketBytes = kHeaderBytes + kPayloadBytes;  // 300字节

  doorlock_sniper::msg::VideoPacket pkt;
  pkt.frame_no = 0;
  pkt.frag_no = 0;
  pkt.total_bytes = 0;
  pkt.payload.fill(0);
  if (param_fixed_test_payload_mode_) {
    constexpr size_t kSyntheticFrameBytes = 64;
    const uint16_t frame_no = current_frame_no_++;
    pkt.frame_no = frame_no;
    pkt.frag_no = 0;
    pkt.total_bytes = kSyntheticFrameBytes;

    std::ostringstream oss;
    oss << "RM0310-TEST frame=" << frame_no
        << " tick=" << packet_sequence_id_;
    const std::string text = oss.str();
    const size_t text_size = std::min(text.size(), kSyntheticFrameBytes);
    memcpy(pkt.payload.data(), text.data(), text_size);
    for (size_t i = text_size; i < kSyntheticFrameBytes; ++i) {
      pkt.payload[i] = static_cast<uint8_t>((frame_no + i) & 0xFF);
    }
  } else {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    // 如果当前没有正在发送的帧，从队列取一帧
    if (current_send_frame_.empty() && !frame_queue_.empty()) {
      current_send_frame_ = std::move(frame_queue_.front());
      frame_queue_.pop_front();
      current_send_offset_ = 0;
      current_frag_no_ = 0;
    }

    if (!current_send_frame_.empty()) {
      size_t remaining = current_send_frame_.size() - current_send_offset_;
      size_t copy_size = std::min(kPayloadBytes, remaining);

      pkt.frame_no = current_frame_no_;
      pkt.frag_no = current_frag_no_++;
      pkt.total_bytes = static_cast<uint32_t>(current_send_frame_.size());
      memcpy(pkt.payload.data(), current_send_frame_.data() + current_send_offset_, copy_size);
      current_send_offset_ += copy_size;

      // 当前帧是否发送完毕
      if (current_send_offset_ >= current_send_frame_.size()) {
        current_send_frame_.clear();
        current_send_offset_ = 0;
        current_frag_no_ = 0;
        current_frame_no_++;
      }
    }
  }

  if (pkt.total_bytes == 0) {
    const int64_t now_ns = this->now().nanoseconds();
    if (now_ns - last_send_idle_log_ns_ > 1000000000LL) {
      size_t queued_frames = 0;
      size_t queued_bytes = 0;
      {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        queued_frames = frame_queue_.size();
        for (const auto &f : frame_queue_) queued_bytes += f.size();
        if (!current_send_frame_.empty()) queued_bytes += current_send_frame_.size();
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Send idle: no packet ready, aus=%lu queued_frames=%zu queued_bytes=%zuB current_offset=%zu",
        encoded_au_count_, queued_frames, queued_bytes, current_send_offset_);
      last_send_idle_log_ns_ = now_ns;
    }
    return;
  }

  if (!param_fixed_test_payload_mode_) {
    packet_pub_->publish(pkt);
  }

  if (comm_) {
    std::array<uint8_t, kPacketBytes> raw_packet;
    raw_packet.fill(0);
    raw_packet[0] = pkt.frame_no & 0xFF;
    raw_packet[1] = (pkt.frame_no >> 8) & 0xFF;
    raw_packet[2] = pkt.frag_no & 0xFF;
    raw_packet[3] = (pkt.frag_no >> 8) & 0xFF;
    raw_packet[4] = pkt.total_bytes & 0xFF;
    raw_packet[5] = (pkt.total_bytes >> 8) & 0xFF;
    raw_packet[6] = (pkt.total_bytes >> 16) & 0xFF;
    raw_packet[7] = (pkt.total_bytes >> 24) & 0xFF;
    memcpy(raw_packet.data() + kHeaderBytes, pkt.payload.data(), kPayloadBytes);
    comm_->send(raw_packet.data(), kPacketBytes);
  }
}

// 从 appsink 拉取编码数据并加入 frame_queue_，由 20ms 定时器统一发送
void VideoEncoderNode::pull_stream_and_packetize()
{
  if (!appsink_) return;

  const size_t max_backlog_bytes = static_cast<size_t>(
    param_bandwidth_limit_kbytes_ * 1000.0 * param_max_tx_delay_s_);
  bool pulled_any_sample = false;

  while (true) {
    GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 0);
    if (!sample) break;
    pulled_any_sample = true;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
      gst_sample_unref(sample);
      continue;
    }

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      std::lock_guard<std::mutex> lock(buffer_mutex_);

      // h265parse alignment=au 保证每个 buffer 是一整帧，直接入队
      std::vector<uint8_t> frame_data(map.size);
      memcpy(frame_data.data(), map.data, map.size);
      frame_queue_.push_back(std::move(frame_data));
      encoded_au_count_++;

      // 排队时延上限：超限时从队列头部丢弃旧帧
      size_t total_bytes = 0;
      for (const auto &f : frame_queue_) total_bytes += f.size();
      if (!current_send_frame_.empty()) total_bytes += current_send_frame_.size();

      while (total_bytes > max_backlog_bytes && !frame_queue_.empty()) {
        size_t dropped = frame_queue_.front().size();
        frame_queue_.pop_front();
        total_bytes -= dropped;
        dropped_bytes_ += dropped;
        dropped_events_++;
        if (dropped_events_ % 20 == 1) {
          RCLCPP_WARN(
            this->get_logger(),
            "TX backlog clipped: dropped=%zuB backlog=%zuB total_dropped=%luB events=%u",
            dropped, total_bytes, dropped_bytes_, dropped_events_);
        }
      }

      const int64_t telemetry_ns = this->now().nanoseconds();
      if (telemetry_ns - last_telemetry_ns_ > 1000000000LL) {
        RCLCPP_INFO(
          this->get_logger(),
          "Encoder stats: aus=%lu backlog=%zuB dropped=%luB",
          encoded_au_count_, total_bytes, dropped_bytes_);
        last_telemetry_ns_ = telemetry_ns;
      }

      gst_buffer_unmap(buffer, &map);
    }
    gst_sample_unref(sample);
  }

  if (!pulled_any_sample) {
    const int64_t now_ns = this->now().nanoseconds();
    if (now_ns - last_appsink_empty_log_ns_ > 1000000000LL) {
      size_t queued_frames = 0;
      size_t queued_bytes = 0;
      {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        queued_frames = frame_queue_.size();
        for (const auto &f : frame_queue_) queued_bytes += f.size();
        if (!current_send_frame_.empty()) queued_bytes += current_send_frame_.size();
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Encoder idle: no appsink sample yet, aus=%lu queued_frames=%zu queued_bytes=%zuB",
        encoded_au_count_, queued_frames, queued_bytes);
      last_appsink_empty_log_ns_ = now_ns;
    }
  }
}

void VideoEncoderNode::process_target_detection(const cv::Mat & frame)
{
  (void)frame;  // 目标检测逻辑已移除，串口通信功能已禁用
}

void VideoEncoderNode::display_loop()
{
  cv::namedWindow("Doorlock Sniper Raw", cv::WINDOW_NORMAL);
  cv::namedWindow("Doorlock Sniper ROI", cv::WINDOW_NORMAL);
  cv::namedWindow("Doorlock Sniper Static", cv::WINDOW_NORMAL);
  cv::namedWindow("Doorlock Sniper", cv::WINDOW_NORMAL);
  cv::setWindowProperty("Doorlock Sniper Raw", cv::WND_PROP_ASPECT_RATIO, cv::WINDOW_KEEPRATIO);
  cv::resizeWindow("Doorlock Sniper ROI", param_output_size_, param_output_size_);
  cv::resizeWindow("Doorlock Sniper Static", param_output_size_, param_output_size_);
  cv::resizeWindow("Doorlock Sniper", param_output_size_, param_output_size_);
  
  while (display_running_ && rclcpp::ok()) {
    cv::Mat raw_frame;
    cv::Mat roi_frame;
    cv::Mat static_frame;
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!display_raw_frame_.empty()) {
        display_raw_frame_.copyTo(raw_frame);
      }
      if (!display_roi_frame_.empty()) {
        display_roi_frame_.copyTo(roi_frame);
      }
      if (!display_static_frame_.empty()) {
        display_static_frame_.copyTo(static_frame);
      }
      if (!display_frame_.empty()) {
        display_frame_.copyTo(frame);
      }
    }
    
    if (!raw_frame.empty()) {
      cv::imshow("Doorlock Sniper Raw", raw_frame);
    }
    if (!roi_frame.empty()) {
      cv::imshow("Doorlock Sniper ROI", roi_frame);
    }
    if (!static_frame.empty()) {
      cv::imshow("Doorlock Sniper Static", static_frame);
    }
    if (!frame.empty()) {
      cv::imshow("Doorlock Sniper", frame);
    }
    if (param_debug_dump_enable_ && !frame.empty()) {
      display_frame_counter_++;
      if ((display_frame_counter_ % static_cast<uint64_t>(param_debug_dump_every_n_frames_)) == 0U) {
        const std::filesystem::path dump_dir = std::filesystem::path(param_debug_dump_dir_) / "encoder";
        std::ostringstream idx;
        idx << std::setw(8) << std::setfill('0') << display_frame_counter_;
        const std::string frame_id = idx.str();
        if (param_debug_dump_save_raw_ && !raw_frame.empty()) {
          cv::imwrite((dump_dir / ("raw_" + frame_id + ".png")).string(), raw_frame);
        }
        if (param_debug_dump_save_roi_ && !roi_frame.empty()) {
          cv::imwrite((dump_dir / ("roi_" + frame_id + ".png")).string(), roi_frame);
        }
        if (param_debug_dump_save_static_ && !static_frame.empty()) {
          cv::imwrite((dump_dir / ("static_" + frame_id + ".png")).string(), static_frame);
        }
        if (param_debug_dump_save_final_) {
          cv::imwrite((dump_dir / ("final_" + frame_id + ".png")).string(), frame);
        }
      }
    }
    cv::waitKey(1);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  
  cv::destroyWindow("Doorlock Sniper Raw");
  cv::destroyWindow("Doorlock Sniper ROI");
  cv::destroyWindow("Doorlock Sniper Static");
  cv::destroyWindow("Doorlock Sniper");
}

} // namespace doorlock_sniper

RCLCPP_COMPONENTS_REGISTER_NODE(doorlock_sniper::VideoEncoderNode)

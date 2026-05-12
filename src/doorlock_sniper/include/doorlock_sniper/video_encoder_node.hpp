#ifndef DOORLOCK_SNIPER_VIDEO_ENCODER_NODE_HPP_
#define DOORLOCK_SNIPER_VIDEO_ENCODER_NODE_HPP_

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <deque>
#include <memory>

#include "doorlock_sniper/msg/video_packet.hpp"
#include "doorlock_sniper/doorlock_comm.hpp"

namespace doorlock_sniper
{

class VideoEncoderNode : public rclcpp::Node
{
public:
  explicit VideoEncoderNode(const rclcpp::NodeOptions & options);
  ~VideoEncoderNode() override;

private:
  void initialize_gstreamer();
  void shutdown_gstreamer();
  void initialize_communication();
  void poll_gstreamer_bus();
  
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  cv::Mat preprocess_image(
    const cv::Mat & input,
    cv::Mat * roi_downsample = nullptr,
    cv::Mat * static_removed = nullptr);
  
  void push_frame_to_gstreamer(const cv::Mat & frame);
  void pull_stream_and_packetize();
  void process_target_detection(const cv::Mat & frame);
  
  void display_loop();
  void send_packet_timer_callback();

  GstElement * pipeline_;
  GstElement * appsrc_;
  GstElement * appsink_;
  GstBus * bus_;
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<doorlock_sniper::msg::VideoPacket>::SharedPtr packet_pub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  
  // 统计与状态（调整顺序，与初始化列表一致）
  uint64_t packet_sequence_id_ = 0;   // 移到这里，先声明
  uint32_t frame_count_ = 0;
  
  // 显示线程
  std::thread display_thread_;
  std::atomic<bool> display_running_;  // 后声明
  
  std::mutex frame_mutex_;
  cv::Mat display_raw_frame_;
  cv::Mat display_roi_frame_;
  cv::Mat display_static_frame_;
  cv::Mat display_frame_;
  
  // 帧队列（每帧是一个完整的 HEVC NAL unit）
  std::deque<std::vector<uint8_t>> frame_queue_;
  std::vector<uint8_t> current_send_frame_;   // 当前正在发送的帧
  std::deque<cv::Mat> motion_mask_history_;
  std::deque<cv::Mat> trail_frame_history_;
  uint64_t dropped_bytes_ = 0;
  uint32_t dropped_events_ = 0;
  int64_t last_telemetry_ns_ = 0;
  int64_t last_appsink_empty_log_ns_ = 0;
  int64_t last_send_idle_log_ns_ = 0;
  uint64_t encoded_au_count_ = 0;
  std::mutex buffer_mutex_;

  // 帧分片状态
  uint16_t current_frame_no_ = 0;      // 当前帧序号
  uint16_t current_frag_no_ = 0;       // 当前分片序号
  size_t current_send_offset_ = 0;     // 当前帧已发送偏移
  std::vector<uint8_t> current_frame_buffer_;  // 编码端当前帧数据缓冲区

  // 帧率控制
  int64_t last_encode_stamp_ns_ = 0;
  uint64_t display_frame_counter_ = 0;
  uint64_t image_callback_count_ = 0;
  int64_t last_image_log_ns_ = 0;
  cv::Mat background_gray_f32_;
  cv::Mat motion_erode_kernel_;
  cv::Mat motion_dilate_kernel_;
  
  // 参数
  int param_crop_size_ = 800;
  int param_output_size_ = 400;
  int param_output_fps_ = 60;
  guint param_target_bitrate_ = 40;
  int param_packet_size_ = 300;
  bool param_static_simplify_ = true;
  int param_motion_threshold_ = 14;
  int param_motion_erode_px_ = 1;
  int param_motion_dilate_px_ = 2;
  int param_motion_trail_frames_ = 3;
  double param_trail_disable_motion_ratio_ = 0.30;
  double param_bg_update_alpha_ = 0.01;
  double param_bg_blur_sigma_ = 1.2;
  int param_center_clear_size_ = 100;
  bool param_force_monochrome_ = false;
  double param_bandwidth_limit_kbytes_ = 7.0;
  double param_bandwidth_window_s_ = 2.0;
  double param_max_tx_delay_s_ = 1.0;
  bool param_enable_display_ = true;
  bool param_fixed_test_payload_mode_ = false;
  bool param_debug_dump_enable_ = false;
  int param_debug_dump_every_n_frames_ = 20;
  bool param_debug_dump_save_raw_ = true;
  bool param_debug_dump_save_roi_ = true;
  bool param_debug_dump_save_static_ = true;
  bool param_debug_dump_save_final_ = true;
  std::string param_input_topic_;
  std::string param_x264_preset_ = "auto";
  std::string param_debug_dump_dir_ = "sniper_debug_imgs";

  std::unique_ptr<DoorlockComm> comm_;
  std::string param_com_port_ = "/dev/ttyACM0";
  int param_baudrate_ = 921600;
  bool param_send_inner_packet_only_ = false;
};

} // namespace doorlock_sniper

#endif

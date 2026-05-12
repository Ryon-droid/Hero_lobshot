#include "doorlock_sniper/doorlock_comm.hpp"

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace doorlock_sniper
{

namespace
{

std::string hex_preview(const uint8_t *data, size_t size, size_t max_bytes = 32)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  const size_t preview_len = std::min(size, max_bytes);
  for (size_t i = 0; i < preview_len; ++i) {
    if (i != 0) {
      oss << ' ';
    }
    oss << std::setw(2) << static_cast<unsigned int>(data[i]);
  }
  if (size > preview_len) {
    oss << " ...";
  }
  return oss.str();
}

}  // namespace

// CRC8/CRC16 tables and init values follow the RoboMaster 2026 protocol appendix.
static constexpr uint8_t kCrc8Init = 0xFF;
static const uint8_t crc8_table[256] = {
  0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
  0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
  0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
  0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
  0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
  0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
  0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
  0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
  0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
  0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
  0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
  0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
  0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
  0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
  0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
  0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
  0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
  0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
  0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
  0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
  0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
  0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
  0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
  0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
  0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
  0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
  0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
  0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
  0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
  0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
  0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
  0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

uint8_t crc8(const uint8_t *data, size_t length)
{
  uint8_t crc = kCrc8Init;
  for (size_t i = 0; i < length; ++i) {
    crc = crc8_table[crc ^ data[i]];
  }
  return crc;
}

static constexpr uint16_t kCrc16Init = 0xFFFF;
static const uint16_t crc16_table[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t crc16(const uint8_t *data, size_t length)
{
  uint16_t crc = kCrc16Init;
  for (size_t i = 0; i < length; ++i) {
    crc = static_cast<uint16_t>((crc >> 8) ^ crc16_table[(crc ^ data[i]) & 0x00FF]);
  }
  return crc;
}

DoorlockComm::DoorlockComm(const std::string & udp_ip, int udp_port, TxFrameMode tx_frame_mode)
{
  tx_frame_mode_ = tx_frame_mode;
#ifdef USE_UDP
  // 创建UDP socket
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    throw std::runtime_error("Failed to create UDP socket");
  }

  // 设置目标地址
  std::memset(&dest_addr_, 0, sizeof(dest_addr_));
  dest_addr_.sin_family = AF_INET;
  dest_addr_.sin_port = htons(udp_port);
  if (inet_pton(AF_INET, udp_ip.c_str(), &dest_addr_.sin_addr) <= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("Invalid UDP IP address: " + udp_ip);
  }

  RCLCPP_INFO(rclcpp::get_logger("DoorlockComm"), "UDP socket opened successfully: %s:%d", udp_ip.c_str(), udp_port);
  readback_enabled_ = false;
#else
  try {
    serial_.setPort(udp_ip); // 参数名保持一致，但实际是串口端口
    serial_.setBaudrate(udp_port); // 参数名保持一致，但实际是波特率
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);
    serial_.open();
    usleep(1000000); // 1s wait
    RCLCPP_INFO(rclcpp::get_logger("DoorlockComm"), "Serial port opened successfully: %s", udp_ip.c_str());
    readback_enabled_ = false;
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to open serial: ") + e.what());
  }
#endif

  RCLCPP_INFO(
    rclcpp::get_logger("DoorlockComm"),
    "TX frame mode: %s",
    tx_frame_mode_ == TxFrameMode::INNER_PACKET_ONLY ? "inner_packet_only" : "rm_outer_frame");

  if (readback_enabled_) {
    thread_ = std::thread(&DoorlockComm::read_thread, this);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("DoorlockComm"), "Readback disabled; running in one-way send mode");
  }
}

DoorlockComm::~DoorlockComm()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
#ifdef USE_UDP
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    RCLCPP_INFO(rclcpp::get_logger("DoorlockComm"), "UDP socket closed");
  }
#else
  if (serial_.isOpen()) {
    serial_.close();
    RCLCPP_INFO(rclcpp::get_logger("DoorlockComm"), "Serial port closed");
  }
#endif
}

DoorlockMode DoorlockComm::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

DoorlockState DoorlockComm::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string DoorlockComm::str(DoorlockMode mode) const
{
  switch (mode) {
    case DoorlockMode::IDLE:
      return "IDLE";
    case DoorlockMode::LOCKING:
      return "LOCKING";
    case DoorlockMode::LOCKED:
      return "LOCKED";
    case DoorlockMode::UNLOCKING:
      return "UNLOCKING";
    case DoorlockMode::UNLOCKED:
      return "UNLOCKED";
    default:
      return "INVALID";
  }
}

void DoorlockComm::send(const uint8_t * data, size_t size)
{
  if (data == nullptr) {
    return;
  }

  if (tx_frame_mode_ == TxFrameMode::INNER_PACKET_ONLY) {
    const size_t inner_size = size;
#ifdef USE_UDP
    if (socket_fd_ >= 0) {
      ssize_t sent = sendto(
        socket_fd_,
        data,
        inner_size,
        0,
        reinterpret_cast<struct sockaddr*>(&dest_addr_),
        sizeof(dest_addr_));
      if (sent != static_cast<ssize_t>(inner_size)) {
        RCLCPP_WARN(rclcpp::get_logger("DoorlockComm"), "Failed to send UDP inner packet");
        return;
      }
    }
#else
    try {
      if (!logged_first_tx_packet_) {
        RCLCPP_INFO(
          rclcpp::get_logger("DoorlockComm"),
          "TX raw first inner packet (%zuB): %s",
          inner_size,
          hex_preview(data, inner_size).c_str());
        logged_first_tx_packet_ = true;
      }

      const size_t bytes_written = serial_.write(data, inner_size);
      if (bytes_written != inner_size) {
        RCLCPP_WARN(
          rclcpp::get_logger("DoorlockComm"),
          "Short serial write: expected=%zuB actual=%zuB",
          inner_size, bytes_written);
        return;
      }
      tx_packet_count_++;
      tx_total_bytes_ += bytes_written;
      if (tx_packet_count_ % 200 == 0) {
        RCLCPP_INFO(
          rclcpp::get_logger("DoorlockComm"),
          "TX stats: packets=%lu bytes=%lu mode=inner packet_len=%zu",
          tx_packet_count_,
          tx_total_bytes_,
          inner_size);
        RCLCPP_INFO(
          rclcpp::get_logger("DoorlockComm"),
          "TX raw inner packet preview (%zuB): %s",
          inner_size,
          hex_preview(data, inner_size).c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("DoorlockComm"), "Failed to write serial: %s", e.what());
    }
#endif
    return;
  }

  // 构建完整的通信协议帧
  CommFrame frame{};

  // 设置帧头
  frame.header.sof = 0xA5;  // 帧起始字节
  uint16_t data_length = 300;  // data字段长度为300字节
  frame.header.data_length[0] = data_length & 0xFF;
  frame.header.data_length[1] = (data_length >> 8) & 0xFF;
  frame.header.seq = seq_++;  // 包序号自增
  // 计算帧头CRC8（仅对sof, data_length, seq计算）
  uint8_t header_crc_data[4] = {
    frame.header.sof,
    frame.header.data_length[0],
    frame.header.data_length[1],
    frame.header.seq
  };
  frame.header.crc8 = crc8(header_crc_data, sizeof(header_crc_data));

  // 0x0310: 机器人发送给自定义客户端的数据（图传链路）。
  uint16_t cmd_id = 0x0310;
  frame.cmd_id[0] = cmd_id & 0xFF;
  frame.cmd_id[1] = (cmd_id >> 8) & 0xFF;

  // 设置data字段（300字节视频数据包）
  std::memcpy(frame.data, data, 300);

  // 协议中的 CRC16 是“整包校验”，覆盖 frame_header + cmd_id + data。
  uint16_t crc = crc16(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame) - sizeof(frame.frame_tail));
  frame.frame_tail[0] = crc & 0xFF;
  frame.frame_tail[1] = (crc >> 8) & 0xFF;

#ifdef USE_UDP
  if (socket_fd_ >= 0) {
    ssize_t sent = sendto(socket_fd_, &frame, sizeof(frame), 0,
                          reinterpret_cast<struct sockaddr*>(&dest_addr_), sizeof(dest_addr_));
    if (sent != sizeof(frame)) {
      RCLCPP_WARN(rclcpp::get_logger("DoorlockComm"), "Failed to send UDP packet");
    } else {
      RCLCPP_DEBUG(rclcpp::get_logger("DoorlockComm"), "Sent UDP video packet - 309B (header:5B + cmd_id:2B + data:300B + tail:2B)");
    }
  }
#else
  try {
    const auto *frame_bytes = reinterpret_cast<const uint8_t *>(&frame);
    if (!logged_first_tx_packet_) {
      RCLCPP_INFO(
        rclcpp::get_logger("DoorlockComm"),
        "TX raw first packet (%zuB): %s",
        sizeof(frame),
        hex_preview(frame_bytes, sizeof(frame)).c_str());
      logged_first_tx_packet_ = true;
    }

    const size_t bytes_written = serial_.write(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
    if (bytes_written != sizeof(frame)) {
      RCLCPP_WARN(
        rclcpp::get_logger("DoorlockComm"),
        "Short serial write: expected=%zuB actual=%zuB",
        sizeof(frame), bytes_written);
      return;
    }

    tx_packet_count_++;
    tx_total_bytes_ += bytes_written;
    if (tx_packet_count_ % 200 == 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("DoorlockComm"),
        "TX stats: packets=%lu bytes=%lu last_seq=%u cmd_id=0x%02X%02X payload_len=%u",
        tx_packet_count_,
        tx_total_bytes_,
        static_cast<unsigned int>(frame.header.seq),
        static_cast<unsigned int>(frame.cmd_id[1]),
        static_cast<unsigned int>(frame.cmd_id[0]),
        static_cast<unsigned int>(data_length));
      RCLCPP_INFO(
        rclcpp::get_logger("DoorlockComm"),
        "TX raw packet preview (%zuB): %s",
        sizeof(frame),
        hex_preview(frame_bytes, sizeof(frame)).c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(rclcpp::get_logger("DoorlockComm"), "Failed to write serial: %s", e.what());
  }
#endif
}

bool DoorlockComm::is_readback_enabled() const
{
  return readback_enabled_;
}

bool DoorlockComm::read(uint8_t * buffer, size_t size)
{
#ifdef USE_UDP
  // UDP模式下不支持读取（单向发送）
  (void)buffer;
  (void)size;
  return false;
#else
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(rclcpp::get_logger("DoorlockComm"), "Failed to read serial: %s", e.what());
    return false;
  }
#endif
}

void DoorlockComm::read_thread()
{
  RCLCPP_INFO(rclcpp::get_logger("DoorlockComm"), "read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      RCLCPP_WARN(rclcpp::get_logger("DoorlockComm"), "Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != 'G' || rx_data_.head[1] != 'L') continue;

    if (rx_data_.tail[0] != 'E' || rx_data_.tail[1] != 'N') continue;

    error_count = 0;

    std::lock_guard<std::mutex> lock(mutex_);

    switch (rx_data_.mode) {
      case 2:
        mode_ = DoorlockMode::LOCKED;  // 吊射模式
        break;
      default:
        mode_ = DoorlockMode::IDLE;
        RCLCPP_WARN(rclcpp::get_logger("DoorlockComm"), "Unknown mode: %d", rx_data_.mode);
        break;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("DoorlockComm"), "Received GimbalToVisionLobshot - Mode: %d", rx_data_.mode);
  }

  RCLCPP_INFO(rclcpp::get_logger("DoorlockComm"), "read_thread stopped.");
}

void DoorlockComm::reconnect()
{
#ifdef USE_UDP
  // UDP模式下不需要重连
  RCLCPP_WARN(rclcpp::get_logger("DoorlockComm"), "Reconnect not supported in UDP mode");
#else
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    RCLCPP_WARN(rclcpp::get_logger("DoorlockComm"), "Reconnecting serial, attempt %d/%d...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      RCLCPP_INFO(rclcpp::get_logger("DoorlockComm"), "Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("DoorlockComm"), "Reconnect failed: %s", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
#endif
}

} // namespace doorlock_sniper

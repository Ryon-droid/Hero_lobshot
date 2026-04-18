#include "doorlock_sniper/doorlock_comm.hpp"

#include <rclcpp/rclcpp.hpp>

namespace doorlock_sniper
{

DoorlockComm::DoorlockComm(const std::string & com_port, int baudrate)
{
  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(baudrate);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);
    serial_.open();
    usleep(1000000); // 1s wait
    RCLCPP_INFO(rclcpp::get_logger("DoorlockComm"), "Serial port opened successfully: %s", com_port.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("DoorlockComm"), "Failed to open serial: %s", e.what());
    exit(1);
  }

  thread_ = std::thread(&DoorlockComm::read_thread, this);
}

DoorlockComm::~DoorlockComm()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
  RCLCPP_INFO(rclcpp::get_logger("DoorlockComm"), "Serial port closed");
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

void DoorlockComm::send(uint16_t sequence_id, const uint8_t * data, uint16_t payload_len)
{
  tx_data_.head[0] = 'V';
  tx_data_.head[1] = 'L';
  tx_data_.sequence_id = sequence_id;
  tx_data_.payload_len = payload_len;
  if (data != nullptr && payload_len > 0) {
    std::memcpy(tx_data_.data.data(), data, std::min(static_cast<size_t>(payload_len), tx_data_.data.size()));
  }
  tx_data_.tail[0] = 'E';
  tx_data_.tail[1] = 'N';
  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
    RCLCPP_DEBUG(rclcpp::get_logger("DoorlockComm"), "Sent video packet - Seq: %d, Len: %d",
                 sequence_id, payload_len);
  } catch (const std::exception & e) {
    RCLCPP_WARN(rclcpp::get_logger("DoorlockComm"), "Failed to write serial: %s", e.what());
  }
}

void DoorlockComm::send(const CommToDoorlock & comm_to_doorlock)
{
  tx_data_ = comm_to_doorlock;
  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
    RCLCPP_DEBUG(rclcpp::get_logger("DoorlockComm"), "Sent video packet - Seq: %d, Len: %d",
                 tx_data_.sequence_id, tx_data_.payload_len);
  } catch (const std::exception & e) {
    RCLCPP_WARN(rclcpp::get_logger("DoorlockComm"), "Failed to write serial: %s", e.what());
  }
}

bool DoorlockComm::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(rclcpp::get_logger("DoorlockComm"), "Failed to read serial: %s", e.what());
    return false;
  }
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
}

} // namespace doorlock_sniper
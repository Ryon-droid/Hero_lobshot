#ifndef DOORLOCK_SNIPER_DOORLOCK_COMM_HPP_
#define DOORLOCK_SNIPER_DOORLOCK_COMM_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"

namespace doorlock_sniper
{

struct __attribute__((packed)) VisionToGimbalLobshot
{
  uint8_t head[2] = {'V', 'L'};      // 帧头 "VL" (Vision->Lobshot)
  uint16_t sequence_id;               // 包序列号
  uint16_t payload_len;               // 有效数据长度
  std::array<uint8_t, 150> data;      // 视频数据 payload
  uint8_t tail[2] = {'E', 'N'};       // 帧尾
};
static_assert(sizeof(VisionToGimbalLobshot) <= 200);
using CommToDoorlock = VisionToGimbalLobshot;

struct __attribute__((packed)) GimbalToVisionLobshot
{
  uint8_t head[2] = {'G', 'L'};       // 帧头 "GL" (Gimbal->Vision Lobshot)
  uint8_t mode;                       // 2: 吊射
  uint8_t tail[2] = {'E', 'N'};       // 帧尾
};
static_assert(sizeof(GimbalToVisionLobshot) <= 16);
using DoorlockToComm = GimbalToVisionLobshot;

enum class DoorlockMode
{
  IDLE,        // 空闲
  LOCKING,     // 锁定中
  LOCKED,      // 已锁定
  UNLOCKING,   // 解锁中
  UNLOCKED     // 已解锁
};

struct DoorlockState
{
  float current_x;
  float current_y;
  uint8_t status;
  uint8_t error_code;
};

class DoorlockComm
{
public:
  DoorlockComm(const std::string & com_port, int baudrate = 115200);
  ~DoorlockComm();

  DoorlockMode mode() const;
  DoorlockState state() const;
  std::string str(DoorlockMode mode) const;

  void send(uint16_t sequence_id, const uint8_t * data, uint16_t payload_len);

  void send(const CommToDoorlock & comm_to_doorlock);

private:
  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  DoorlockToComm rx_data_;
  CommToDoorlock tx_data_;

  DoorlockMode mode_ = DoorlockMode::IDLE;
  DoorlockState state_;

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();
};

} // namespace doorlock_sniper

#endif // DOORLOCK_SNIPER_DOORLOCK_COMM_HPP_
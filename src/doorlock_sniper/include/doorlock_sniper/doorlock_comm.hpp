#ifndef DOORLOCK_SNIPER_DOORLOCK_COMM_HPP_
#define DOORLOCK_SNIPER_DOORLOCK_COMM_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>

#ifdef USE_UDP
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#else
#include "serial/serial.h"
#endif

namespace doorlock_sniper
{

// CRC8 计算（用于帧头校验）
uint8_t crc8(const uint8_t *data, size_t length);

// CRC16 计算（用于整包校验）
uint16_t crc16(const uint8_t *data, size_t length);

// 通信协议帧头格式
struct __attribute__((packed)) FrameHeader
{
  uint8_t sof = 0xA5;           // 帧起始字节，固定值0xA5
  uint8_t data_length[2] = {0}; // data长度（小端序）
  uint8_t seq = 0;              // 包序号
  uint8_t crc8 = 0;             // 帧头CRC8校验
};
static_assert(sizeof(FrameHeader) == 5);

// 完整通信协议帧格式
// [frame_header(5B)][cmd_id(2B)][data(nB)][frame_tail(2B)]
struct __attribute__((packed)) CommFrame
{
  FrameHeader header;
  uint8_t cmd_id[2] = {0};      // 命令ID（小端序）
  uint8_t data[300];            // 数据负载（300字节：8字节头部 + 292字节视频数据）
  uint8_t frame_tail[2] = {0};  // CRC16校验（小端序）
};
static_assert(sizeof(CommFrame) == 5 + 2 + 300 + 2);

struct __attribute__((packed)) VisionToGimbalLobshot
{
  std::array<uint8_t, 150> data;      // 纯 150 字节视频流 payload
};
static_assert(sizeof(VisionToGimbalLobshot) == 150);
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

enum class TxFrameMode
{
  RM_OUTER_FRAME,
  INNER_PACKET_ONLY
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
  DoorlockComm(
    const std::string & udp_ip,
    int udp_port = 5000,
    TxFrameMode tx_frame_mode = TxFrameMode::RM_OUTER_FRAME);
  ~DoorlockComm();

  DoorlockMode mode() const;
  DoorlockState state() const;
  std::string str(DoorlockMode mode) const;

  void send(const uint8_t * data, size_t size = 300);
  bool is_readback_enabled() const;

private:
#ifdef USE_UDP
  int socket_fd_ = -1;
  struct sockaddr_in dest_addr_;
#else
  serial::Serial serial_;
#endif

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  DoorlockToComm rx_data_;
  CommToDoorlock tx_data_;

  DoorlockMode mode_ = DoorlockMode::IDLE;
  DoorlockState state_;
  bool readback_enabled_ = false;
  TxFrameMode tx_frame_mode_ = TxFrameMode::RM_OUTER_FRAME;

  uint8_t seq_ = 0;  // 包序号，用于通信协议帧头
  uint64_t tx_packet_count_ = 0;
  uint64_t tx_total_bytes_ = 0;
  bool logged_first_tx_packet_ = false;

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();
};

} // namespace doorlock_sniper

#endif // DOORLOCK_SNIPER_DOORLOCK_COMM_HPP_

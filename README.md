# 【RM2026】部署模式低带宽落点图传 - 五大湖联合大学

用于 RoboMaster 2026 部署模式下，英雄机器人通过 `0x0310` 自定义客户端链路传输低带宽视频流的工程仓库。

[演示视频](https://www.bilibili.com/video/BV12aDMBcEob)

[RM 社区开源报告](https://bbs.robomaster.com/article/1883295)

<img width="502" height="323" alt="视频封面" src="https://github.com/user-attachments/assets/a72e1683-be42-44d2-a3ae-7686517728fd" />

## 仓库结构

本仓库包含两部分，彼此独立：

```text
Hero_lobshot-master/
├── src/                         # ROS 2 工作区源码
│   ├── bringup/                 # 启动文件
│   ├── hik_camera/              # 海康相机节点
│   ├── doorlock_sniper/         # 编码、分片、串口发送
│   └── doorlock_decoder/        # 本机 ROS 解码显示
├── commu/commu/                 # 独立 Python 项目：自定义客户端接收端
│   ├── README.md
│   ├── pyproject.toml
│   └── src/rm_custom_client/
├── scripts/                     # 发送/接收内层视频包调试脚本
├── 串口协议节选.md
└── 英雄图传技术开源.md
```

## 整体链路

当前代码实现的是这条链路：

1. 本机 `hik_camera` 采集海康相机图像。
2. `doorlock_sniper` 对画面做 ROI、静态区简化、HEVC 编码。
3. 编码结果被切成 `300B` 内层视频包：
   - `frame_no(2B)`
   - `frag_no(2B)`
   - `total_bytes(4B)`
   - `payload(292B)`
4. `DoorlockComm` 将内层视频包封装进 RoboMaster 外层 `0x0310` 帧，通过图传串口链路发送。
5. 官方链路转发后，自定义客户端侧通过 MQTT `CustomByteBlock.data` 收到 `0x0310` 数据。
6. `commu` 项目中的 `video_receiver.py` 从 `CustomByteBlock` 中恢复内层视频包并重组 HEVC 码流。

## ROS 2 工作区

### 环境要求

- Ubuntu Linux
- ROS 2 Jazzy
- 海康 MVS SDK
- GStreamer / OpenCV

注意：仓库旧文档里有 `kilted` 说法，但当前实际环境和代码按 **Jazzy** 使用。

### 系统依赖

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  python3-colcon-common-extensions python3-rosdep \
  python3-opencv python3-av \
  libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

然后安装 ROS 依赖：

```bash
cd /home/sentry/Desktop/Hero_lobshot-master
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 海康 SDK

`hik_camera` 依赖海康 MVS SDK，默认查找：

- `/opt/MVS/include`
- `/opt/MVS/lib/64`

### 编译

```bash
cd /home/sentry/Desktop/Hero_lobshot-master
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### 启动发送端

```bash
cd /home/sentry/Desktop/Hero_lobshot-master
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch bringup sniper.launch.py
```

### 当前默认发送参数

当前 [src/bringup/launch/sniper.launch.py](/home/sentry/Desktop/Hero_lobshot-master/src/bringup/launch/sniper.launch.py) 默认配置为：

- 编码分辨率：`300 x 300`
- 输出帧率：`40 fps`
- 目标码率：`10 kB/s`
- 发送硬上限：`12 kB/s`
- `0x0310` 内层视频包大小：`300 B`
- 发送模式：默认发送 **RM 外层帧**，不是裸 UDP 内层包
- `fixed_test_payload_mode=False`

为了提高稳定帧率，当前默认还关闭了本机编码预览窗口：

- `enable_display=False`
- `motion_trail_frames=8`
- `x264_preset='faster'`

## `commu` 自定义客户端工程

`commu/commu` 是独立 Python 项目，不是 ROS 包。

### 安装

```bash
cd /home/sentry/Desktop/Hero_lobshot-master/commu/commu
uv sync
```

### 命令行接收

```bash
cd /home/milkdragon/桌面/commu
uv run rm-custom-client --client-id 1 --topic CustomByteBlock --video-port 0 --video-output captures/video.hevc --log-level INFO
```

说明：

- `--topic CustomByteBlock`：订阅官方链路转发的 `0x0310`
- `--video-port 0`：关闭原始 UDP 图传输入，只走 MQTT `CustomByteBlock`
- `--video-output ...`：保存重组出的 HEVC

### 12 秒探针

```bash
cd /home/milkdragon/桌面/commu
uv run rm-custom-client --client-id 1 --topic CustomByteBlock --video-port 0 --video-output captures/video.hevc --no-print-payloads --probe-seconds 12 --log-level INFO
```

### GUI 接收

如果远端机器有图形桌面环境，可以运行：

```bash
cd /home/milkdragon/桌面/commu
uv run rm-custom-client-gui --client-id 1 --video-output captures/gui.hevc --log-level INFO
```

注意：

- 远端若 `DISPLAY` 为空，GUI 不会弹窗
- SSH 远程启动 GUI 时通常需要 `ssh -X` 或在远端本地图形终端中执行

## 重要实现细节

### 1. `CustomByteBlock` 解析

接收端不再把 `CustomByteBlock.data` 当成原始 UDP 包处理，而是优先按 `300B` 内层视频包解析；必要时再回退到 RM 外层帧解析。

### 2. 分片格式

内层视频包头部当前约定为小端：

- `frame_no`: 2 字节
- `frag_no`: 2 字节
- `total_bytes`: 4 字节

其中 `frag_no` 表示 **分片序号**，不是字节偏移。

### 3. 本机解码与下位机解码

- ROS 工作区中的 `doorlock_decoder` 用于本机 ROS 内部验证
- 下位机真正接收 `CustomByteBlock` 的逻辑在 `commu/commu/src/rm_custom_client/video_receiver.py`

## 当前测试结论

最近一轮实测里：

- 官方链路 `CustomByteBlock` 已能稳定收到
- 下位机能从 `CustomByteBlock` 重组出 HEVC
- 12 秒接收探针约为：
  - `mqtt_messages=360`
  - `video_packets=360`
  - `video_frames=328`
- 实际有效视频帧率约 `27 fps`

## 相关文档

- [串口协议节选.md](/home/sentry/Desktop/Hero_lobshot-master/串口协议节选.md)
- [英雄图传技术开源.md](/home/sentry/Desktop/Hero_lobshot-master/英雄图传技术开源.md)
- [commu/commu/README.md](/home/sentry/Desktop/Hero_lobshot-master/commu/commu/README.md)

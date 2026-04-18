# 【RM2026】部署模式低带宽落点图传 - 五大湖联合大学

用于RoboMaster部署模式下英雄机器人观测落点用的低带宽图传。

[演示视频](https://www.bilibili.com/video/BV12aDMBcEob)

[RM社区 - 开源报告](https://bbs.robomaster.com/article/1883295) (如果加载不出来可能是在审核中)

<img width="502" height="323" alt="视频封面" src="https://github.com/user-attachments/assets/a72e1683-be42-44d2-a3ae-7686517728fd" />

## 项目结构

```
Hero_lobshot/
├── src/
│   ├── bringup/                # 启动配置
│   │   ├── launch/
│   │   │   └── sniper.launch.py  # 主启动文件
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── doorlock_decoder/       # 视频解码节点
│   │   ├── doorlock_decoder/
│   │   │   ├── __init__.py
│   │   │   ├── py.typed
│   │   │   └── video_decoder_node.py
│   │   ├── resource/
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   └── setup.py
│   ├── doorlock_sniper/        # 视频编码节点
│   │   ├── include/
│   │   │   └── doorlock_sniper/
│   │   │       ├── doorlock_comm.hpp
│   │   │       └── video_encoder_node.hpp
│   │   ├── msg/
│   │   │   └── VideoPacket.msg  # 视频数据包消息定义
│   │   ├── src/
│   │   │   ├── doorlock_comm.cpp
│   │   │   └── video_encoder_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── hik_camera/             # 海康相机驱动
│       ├── config/
│       │   └── camera_6mm_MV-CS016-10UC.yaml  # 相机配置文件
│       ├── src/
│       │   └── hik_camera_node.cpp
│       ├── CMakeLists.txt
│       └── package.xml
├── .gitignore
└── README.md
```

## 环境要求

- Ubuntu Linux
- ROS 2 kilted（其他版本如Humble可能需要修改QoS等API）
- 海康相机的 MVS SDK

## 安装依赖

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

安装ROS相关依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 相机依赖

`hik_camera` 包依赖海康相机的MVS SDK，确保以下路径存在：

- 头文件：`/opt/MVS/include`
- 库文件：`/opt/MVS/lib/64`

## 编译启动

1. 先`source` ROS的`setup.bash`：

```bash
source /opt/ros/kilted/setup.bash
```

2. 编译项目：

```bash
colcon build
```

3. 加载编译结果：

```bash
source install/setup.bash
```

4. 启动系统：

```bash
ros2 launch bringup sniper.launch.py
```

## 启动参数说明

在`sniper.launch.py`文件中可以修改以下参数：

- 图传分辨率
- 准星位置
- 是否dump图片用于调试
- 其他相关配置

详见文件内注释。

## 工作原理

1. **hik_camera_node**：负责从海康相机采集图像
2. **video_encoder_node**：对图像进行编码压缩，生成低带宽视频流
3. **video_decoder_node**：接收并解码视频流，显示观测画面

## 注意事项

- 本工程仅为演示工程，开发过程中使用了LLM作为辅助
- 欢迎基于此思路开发更好的自定义客户端
- 如有问题请参考演示视频和社区开源报告

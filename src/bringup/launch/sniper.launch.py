from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from pathlib import Path


def generate_launch_description():
    launch_path = Path(__file__).resolve()
    project_root = launch_path.parents[3]  # 源码运行时: .../Pacific_vision
    if project_root.name == 'bringup' and (project_root / 'share').exists():
        # 安装运行时: .../Pacific_vision/install/bringup/share/bringup/launch/sniper.launch.py
        # parents[3] 会是 .../install/bringup，这里回退到工作区根目录
        project_root = project_root.parents[1]
    
    debug_dump_dir = str(project_root / 'sniper_debug_imgs')  # 调试图片保存目录
    debug_dump_enable = False          # 调试开关：每N帧保存5个窗口画面
    debug_dump_every_n_frames = 1     # 调试保存间隔(帧)
    dump_save_raw = False              # 保存编码端 Raw 窗口
    dump_save_roi = True              # 保存编码端 ROI 窗口
    dump_save_static = False           # 保存编码端 Static 窗口
    dump_save_final = True            # 保存编码端 Final 窗口
    dump_save_decoder = True          # 保存解码端窗口

    # 码率策略（单位：kB/s）
    target_bitrate_kbytes = 10.0       # 目标编码码率
    hard_max_bitrate_kbytes = 14.0     # 传输硬上限（由发送窗口限速实现）
    target_bitrate_kbps = int(target_bitrate_kbytes * 8.0)  # x264 参数单位是 kbps
    x264_preset = 'veryslow'           # x264 速度预设：slow 会比veryslow更省时延但画质/压缩效率略降
    encode_size = 300

    # 编码端容器（相机 + 编码器，同进程零拷贝）
    encoder_container = ComposableNodeContainer(
        name='sniper_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hik_camera',
                plugin='hik_camera::HikCameraNode',
                name='hik_camera',
                parameters=[
                    {'exposure_time': 12000.0},  # 曝光时间(us)
                    {'gain': 10.0}               # 模拟增益
                ],
                extra_arguments=[{'use_intra_process_comms': True}]  # 启用进程内零拷贝
            ),
            ComposableNode(
                package='doorlock_sniper',
                plugin='doorlock_sniper::VideoEncoderNode',
                name='video_encoder',
                parameters=[
                    {'input_topic': '/image_raw'},                       # 输入图像话题
                    {'target_bitrate': target_bitrate_kbps},             # 目标编码码率(kbps)，5kB/s -> 40kbps
                    {'x264_preset': x264_preset},                        # x264 preset: auto/ultrafast/.../veryslow
                    {'output_fps': 60},                                  # 输出帧率
                    {'packet_size': 300},                                # 固定分包大小(byte)
                    {'enable_display': True},                            # 编码端调试显示开关
                    {'debug_dump_enable': debug_dump_enable},            # 开启后每N帧保存编码端窗口画面
                    {'debug_dump_every_n_frames': debug_dump_every_n_frames},  # 编码端保存间隔(帧)
                    {'debug_dump_save_raw': dump_save_raw},              # 编码端 Raw 窗口保存开关
                    {'debug_dump_save_roi': dump_save_roi},              # 编码端 ROI 窗口保存开关
                    {'debug_dump_save_static': dump_save_static},        # 编码端 Static 窗口保存开关
                    {'debug_dump_save_final': dump_save_final},          # 编码端 Final 窗口保存开关
                    {'debug_dump_dir': debug_dump_dir},                  # 调试图片根目录
                    {'crop_size': 800},                                  # 中心裁剪ROI大小
                    {'output_size': encode_size},                        # 编码分辨率
                    {'static_simplify': True},                           # 静态区域简化
                    {'motion_threshold': 14},                            # 运动检测阈值
                    {'motion_erode_px': 2},                              # 运动掩码腐蚀像素(y)
                    {'motion_dilate_px': 6},                             # 运动掩码膨胀像素(x)
                    {'motion_trail_frames': 90},                         # 拖影历史帧数
                    {'trail_disable_motion_ratio': 0.30},                # 全局运动比例超阈值时临时禁用拖影显示
                    {'bg_update_alpha': 0.01},                           # 背景模型更新速度
                    {'bg_blur_sigma': 1.8},                              # 静态区模糊强度
                    {'center_clear_size': 150},                          # 中心保护区尺寸(像素)
                    {'force_monochrome': False},                         # 强制全画面灰度
                    {'bandwidth_limit_kbytes': hard_max_bitrate_kbytes}, # 发送硬上限(kB/s)
                    {'bandwidth_window_s': 2.0},                         # 限速滑动窗口时长(s)
                    {'max_tx_delay_s': 1.0}                              # 发送队列最大允许时延(s)
                ],
                extra_arguments=[{'use_intra_process_comms': True}]      # 启用进程内零拷贝
            )
        ],
        output='screen',
    )

    # 解码端（Python 节点，独立进程）
    decoder_node = Node(
        package='doorlock_decoder',       # 修正：只保留这一个 package 参数
        executable='decoder_node',        # 对应 setup.py 中的 entry point
        name='video_decoder',
        parameters=[
            {'topic': '/video_stream'},      # 订阅的视频流话题
            {'display': True},               # 解码端显示开关
            {'width': encode_size},                  # 解码期望宽度
            {'height': encode_size},                 # 解码期望高度
            {'display_scale': 2},            # 显示放大倍数(300->600)
            {'crosshair_offset_x': 0},       # 准心相对中心X偏移
            {'crosshair_offset_y': 0},       # 准心相对中心Y偏移
            {'crosshair_width': 1},          # 准心线宽(像素)
            {'debug_dump_enable': debug_dump_enable},            # 开启后每N帧保存解码窗口画面
            {'debug_dump_every_n_frames': debug_dump_every_n_frames},  # 解码端保存间隔(帧)
            {'debug_dump_save_decoder': dump_save_decoder},      # 解码端窗口保存开关
            {'debug_dump_dir': debug_dump_dir}                  # 调试图片根目录
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        encoder_container,
        decoder_node
    ])

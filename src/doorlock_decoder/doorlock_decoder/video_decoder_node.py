import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from doorlock_sniper.msg import VideoPacket
import av
import cv2
import threading
import queue
from pathlib import Path


class VideoDecoderNode(Node):
    def __init__(self):
        super().__init__('video_decoder_node')
        
        # 参数
        self.declare_parameter('topic', '/video_stream')
        self.declare_parameter('display', True)
        self.declare_parameter('width', 400)
        self.declare_parameter('height', 400)
        self.declare_parameter('display_scale', 2)
        self.declare_parameter('crosshair_offset_x', 0)
        self.declare_parameter('crosshair_offset_y', 0)
        self.declare_parameter('crosshair_width', 2)
        self.declare_parameter('debug_dump_enable', False)
        self.declare_parameter('debug_dump_every_n_frames', 20)
        self.declare_parameter('debug_dump_save_decoder', True)
        self.declare_parameter('debug_dump_dir', 'sniper_debug_imgs')
        
        topic = self.get_parameter('topic').value
        self.display = self.get_parameter('display').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.display_scale = max(1, int(self.get_parameter('display_scale').value))
        self.display_width = self.width * self.display_scale
        self.display_height = self.height * self.display_scale
        self.crosshair_offset_x = int(self.get_parameter('crosshair_offset_x').value)
        self.crosshair_offset_y = int(self.get_parameter('crosshair_offset_y').value)
        self.crosshair_width = max(1, int(self.get_parameter('crosshair_width').value))
        self.debug_dump_enable = bool(self.get_parameter('debug_dump_enable').value)
        self.debug_dump_every_n_frames = max(1, int(self.get_parameter('debug_dump_every_n_frames').value))
        self.debug_dump_save_decoder = bool(self.get_parameter('debug_dump_save_decoder').value)
        self.debug_dump_dir = Path(str(self.get_parameter('debug_dump_dir').value)) / 'decoder'
        self.display_frame_counter = 0
        if self.debug_dump_enable and self.debug_dump_save_decoder:
            self.debug_dump_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(
                f'Debug dump enabled: every {self.debug_dump_every_n_frames} frames -> {self.debug_dump_dir}'
            )
        elif self.debug_dump_enable:
            self.get_logger().warn('debug_dump_enable=true but debug_dump_save_decoder=false')

        # 流式解码器状态
        self.codec = None
        self._reset_decoder(log=False, reason='startup')
        self.frame_count = 0
        self.packet_count = 0
        self.parsed_packet_count = 0
        self.gap_count = 0
        self.last_seq = None
        
        # 显示队列
        if self.display:
            self.frame_queue = queue.Queue(maxsize=3)
            self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
            self.display_thread.start()
        
        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=3000
        )
        
        self.subscription = self.create_subscription(
            VideoPacket,
            topic,
            self._packet_callback,
            qos
        )
        
        self.get_logger().info(f'Decoder started: subscribing to {topic}')

    def _reset_decoder(self, *, log: bool = True, reason: str = ''):
        self.codec = av.CodecContext.create('h264', 'r')
        self.codec.thread_type = 'FRAME'
        self.codec.flags |= av.codec.context.Flags.LOW_DELAY
        if log:
            self.get_logger().warn(f'Reset decoder ({reason})')

    def _handle_decoded_frame(self, frame):
        if frame is None or frame.width == 0 or frame.height == 0:
            return

        img = frame.to_ndarray(format='bgr24')
        if img is None or img.size == 0:
            return

        self.frame_count += 1
        if self.display:
            try:
                self.frame_queue.put_nowait(img)
            except queue.Full:
                pass
        elif self.frame_count % 60 == 0:
            self.get_logger().info(f'Decoded {self.frame_count} frames')
        
    def _packet_callback(self, msg):
        """接收 150byte 分片，先 parse，再 decode。"""
        self.packet_count += 1

        # 丢包检测
        if self.last_seq is not None and msg.sequence_id != self.last_seq + 1:
            self.gap_count += 1
            self.get_logger().warn(
                f'Gap detected: {self.last_seq} -> {msg.sequence_id}, reset decoder')
            # 任意 150B 分片丢失都会破坏码流同步，直接重置等待下一组 SPS/PPS + IDR
            self._reset_decoder(reason='sequence gap')
        self.last_seq = msg.sequence_id

        chunk = bytes(msg.data)

        try:
            parsed_packets = self.codec.parse(chunk)
            self.parsed_packet_count += len(parsed_packets)
            for packet in parsed_packets:
                for frame in self.codec.decode(packet):
                    self._handle_decoded_frame(frame)
        except av.AVError as e:
            self.get_logger().debug(f'Decode error: {e!s}')

        if self.packet_count % 600 == 0:
            self.get_logger().info(
                f'Rx packets={self.packet_count} parsed_h264={self.parsed_packet_count} '
                f'decoded_frames={self.frame_count} gaps={self.gap_count}')
    
    def _display_loop(self):
        """独立线程显示"""
        cv2.namedWindow('Doorlock Decoder', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Doorlock Decoder', self.display_width, self.display_height)
        
        while rclpy.ok():
            try:
                img = self.frame_queue.get(timeout=0.05)
                if img is None:  # 退出信号
                    break
                if img.size > 0:  # 再次检查
                    img_disp = cv2.resize(
                        img,
                        (self.display_width, self.display_height),
                        interpolation=cv2.INTER_NEAREST
                    )
                    self._draw_overlay(img_disp)
                    cv2.imshow('Doorlock Decoder', img_disp)
                    if self.debug_dump_enable and self.debug_dump_save_decoder:
                        self.display_frame_counter += 1
                        if self.display_frame_counter % self.debug_dump_every_n_frames == 0:
                            frame_id = f'{self.display_frame_counter:08d}'
                            out_path = self.debug_dump_dir / f'decoder_{frame_id}.png'
                            cv2.imwrite(str(out_path), img_disp)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.get_logger().info('User quit')
                        rclpy.shutdown()
                        break
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Display error: {e}')
                break
        
        cv2.destroyAllWindows()

    def _draw_overlay(self, img):
        """叠加准心与中心圆点。"""
        h, w = img.shape[:2]

        # 准心位置（相对画面中心可调）
        cx = max(0, min(w - 1, w // 2 + self.crosshair_offset_x))
        cy = max(0, min(h - 1, h // 2 + self.crosshair_offset_y))

        # 淡紫色准心（横竖贯穿全屏）
        crosshair_color = (230, 190, 235)  # BGR
        cv2.line(img, (0, cy), (w - 1, cy), crosshair_color, self.crosshair_width, cv2.LINE_AA)
        cv2.line(img, (cx, 0), (cx, h - 1), crosshair_color, self.crosshair_width, cv2.LINE_AA)

        # 画面正中心固定淡绿色小圆点（不可调）
        center_color = (170, 255, 170)  # BGR
        center = (w // 2, h // 2)
        cv2.circle(img, center, 24, center_color, 1, cv2.LINE_AA)
    
    def destroy_node(self):
        if self.display:
            try:
                self.frame_queue.put_nowait(None)  # 退出信号
            except queue.Full:
                pass
            self.display_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoDecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

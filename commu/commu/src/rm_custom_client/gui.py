from __future__ import annotations

import argparse
import io
import json
import logging
import queue
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from collections import Counter, deque
from pathlib import Path
from tkinter import ttk
from typing import Any

import paho.mqtt.client as mqtt
from PIL import Image, ImageTk
from google.protobuf.message import DecodeError

from rm_custom_client.mqtt_receiver import DEFAULT_TOPICS, decode_custom_byte_block_data, parse_payload
from rm_custom_client.video_receiver import VideoReceiver

LOGGER = logging.getLogger(__name__)


class HevcPreviewDecoder:
    def __init__(self, output_queue: queue.Queue[Image.Image], width: int = 960, height: int = 540) -> None:
        self.output_queue = output_queue
        self.width = width
        self.height = height
        self.input_queue: queue.Queue[bytes] = queue.Queue(maxsize=600)
        self._stop = threading.Event()
        self._proc: subprocess.Popen[bytes] | None = None
        self._reader: threading.Thread | None = None
        self._stderr_reader: threading.Thread | None = None
        self._writer: threading.Thread | None = None
        self.frames_decoded = 0
        self.last_frame_at: float | None = None
        self.last_error = ""
        self.waiting_for_keyframe = True
        self._pending: deque[bytes] = deque(maxlen=180)
        self._pending_nal_types: set[int] = set()

    def start(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            return
        self._stop.clear()
        self._proc = subprocess.Popen(
            [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "error",
                "-analyzeduration",
                "0",
                "-probesize",
                "2048",
                "-fflags",
                "nobuffer",
                "-flags",
                "low_delay",
                "-flags2",
                "showall",
                "-f",
                "hevc",
                "-i",
                "pipe:0",
                "-an",
                "-vf",
                f"scale={self.width}:{self.height}:flags=fast_bilinear",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "pipe:1",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self._reader = threading.Thread(target=self._read_frames, daemon=True)
        self._reader.start()
        self._stderr_reader = threading.Thread(target=self._read_stderr, daemon=True)
        self._stderr_reader.start()
        if self._writer is None or not self._writer.is_alive():
            self._writer = threading.Thread(target=self._write_hevc, daemon=True)
            self._writer.start()

    def stop(self) -> None:
        self._stop.set()
        self._close_process()

    def _close_process(self) -> None:
        if self._proc is None:
            return
        if self._proc.stdin is not None:
            try:
                self._proc.stdin.close()
            except BrokenPipeError:
                pass
        self._proc.terminate()
        self._proc = None

    def _restart_process(self) -> None:
        self._close_process()
        self.waiting_for_keyframe = True
        self._pending.clear()
        self._pending_nal_types.clear()
        if not self._stop.is_set():
            return

    def feed(self, hevc_data: bytes) -> None:
        if self.waiting_for_keyframe:
            self._pending.append(hevc_data)
            self._pending_nal_types.update(_hevc_nal_types(hevc_data))
            if not _has_hevc_random_access_point(self._pending_nal_types):
                return
            self.waiting_for_keyframe = False
            self.start()
            pending_frames = list(self._pending)
            start_index = 0
            for index, pending in enumerate(pending_frames):
                if 32 in _hevc_nal_types(pending):
                    start_index = index
                    break
            for pending in pending_frames[start_index:]:
                self._put_input(pending)
            self._pending.clear()
            self._pending_nal_types.clear()
            return

        if self._proc is None or self._proc.poll() is not None:
            self._restart_process()
            self.feed(hevc_data)
            return
        self._put_input(hevc_data)

    def _put_input(self, hevc_data: bytes) -> None:
        try:
            self.input_queue.put_nowait(hevc_data)
        except queue.Full:
            try:
                self.input_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.input_queue.put_nowait(hevc_data)
            except queue.Full:
                pass

    def _read_frames(self) -> None:
        if self._proc is None or self._proc.stdout is None:
            return
        frame_size = self.width * self.height * 3
        while not self._stop.is_set():
            chunks = bytearray()
            while len(chunks) < frame_size and not self._stop.is_set():
                chunk = self._proc.stdout.read(frame_size - len(chunks))
                if not chunk:
                    break
                chunks.extend(chunk)
            if len(chunks) != frame_size:
                break
            try:
                image = Image.frombytes("RGB", (self.width, self.height), bytes(chunks))
            except Exception:
                continue
            while self.output_queue.full():
                try:
                    self.output_queue.get_nowait()
                except queue.Empty:
                    break
            self.output_queue.put(image)
            self.frames_decoded += 1
            self.last_frame_at = time.time()

    def _read_stderr(self) -> None:
        if self._proc is None or self._proc.stderr is None:
            return
        while not self._stop.is_set():
            line = self._proc.stderr.readline()
            if not line:
                break
            self.last_error = line.decode("utf-8", "replace").strip()

    def _write_hevc(self) -> None:
        while not self._stop.is_set():
            try:
                hevc_data = self.input_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if self._proc is None or self._proc.poll() is not None:
                self.start()
            if self._proc is None or self._proc.stdin is None:
                continue
            try:
                self._proc.stdin.write(hevc_data)
                self._proc.stdin.flush()
            except BrokenPipeError:
                LOGGER.warning("ffmpeg preview pipe closed")
                self._close_process()
                self.waiting_for_keyframe = True
                self._pending_nal_types.clear()
            except OSError as exc:
                LOGGER.warning("ffmpeg preview write failed: %s", exc)
                self._close_process()
                self.waiting_for_keyframe = True
                self._pending_nal_types.clear()


def _hevc_nal_types(data: bytes) -> set[int]:
    nal_types: set[int] = set()
    index = 0
    end = len(data) - 4
    while index <= end:
        start_len = 0
        if data[index:index + 4] == b"\x00\x00\x00\x01":
            start_len = 4
        elif data[index:index + 3] == b"\x00\x00\x01":
            start_len = 3
        if start_len:
            header = index + start_len
            if header < len(data):
                nal_types.add((data[header] >> 1) & 0x3F)
            index = header + 2
            continue
        index += 1
    return nal_types


def _has_hevc_random_access_point(nal_types: set[int]) -> bool:
    has_parameter_sets = {32, 33, 34}.issubset(nal_types)
    has_idr = any(16 <= nal_type <= 21 for nal_type in nal_types)
    return has_parameter_sets and has_idr


class GuiMqttClient:
    def __init__(
        self,
        host: str,
        port: int,
        client_id: str,
        event_queue: queue.Queue[dict[str, Any]],
        subscribe_all: bool,
        custom_byte_callback,
    ) -> None:
        self.host = host
        self.port = port
        self.client_id = client_id
        self.event_queue = event_queue
        self.subscribe_all = subscribe_all
        self.custom_byte_callback = custom_byte_callback
        self.connected = False
        self.messages = 0
        self.per_topic: Counter[str] = Counter()

        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id,
            protocol=mqtt.MQTTv311,
        )
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

    def start(self) -> None:
        self.client.connect(self.host, self.port, keepalive=30)
        self.client.loop_start()

    def stop(self) -> None:
        self.client.loop_stop()
        self.client.disconnect()

    def _on_connect(
        self,
        client: mqtt.Client,
        _userdata: object,
        _flags: mqtt.ConnectFlags,
        reason_code: mqtt.ReasonCode,
        _properties: mqtt.Properties | None,
    ) -> None:
        self.connected = reason_code == 0
        self.event_queue.put({"kind": "mqtt_status", "connected": self.connected, "reason": str(reason_code)})
        if not self.connected:
            return
        topics = ["#"] if self.subscribe_all else list(DEFAULT_TOPICS)
        for topic in topics:
            client.subscribe(topic, qos=1)

    def _on_disconnect(
        self,
        _client: mqtt.Client,
        _userdata: object,
        _disconnect_flags: mqtt.DisconnectFlags,
        reason_code: mqtt.ReasonCode,
        _properties: mqtt.Properties | None,
    ) -> None:
        self.connected = False
        self.event_queue.put({"kind": "mqtt_status", "connected": False, "reason": str(reason_code)})

    def _on_message(self, _client: mqtt.Client, _userdata: object, msg: mqtt.MQTTMessage) -> None:
        payload = bytes(msg.payload)
        if msg.topic == "CustomByteBlock" and self.custom_byte_callback is not None:
            raw_data = decode_custom_byte_block_data(payload)
            if raw_data:
                self.custom_byte_callback(raw_data)
        try:
            parsed = parse_payload(msg.topic, payload)
            error = None
        except (DecodeError, ValueError) as exc:
            parsed = {"hex": payload.hex(" ", 1)}
            error = str(exc)
        self.messages += 1
        self.per_topic[msg.topic] += 1
        self.event_queue.put(
            {
                "kind": "mqtt",
                "topic": msg.topic,
                "bytes": len(payload),
                "count": self.per_topic[msg.topic],
                "payload": parsed,
                "error": error,
                "time": time.strftime("%H:%M:%S"),
            }
        )


class RmCustomClientWindow:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.events: queue.Queue[dict[str, Any]] = queue.Queue()
        self.images: queue.Queue[Image.Image] = queue.Queue(maxsize=3)
        self.topic_counts: Counter[str] = Counter()
        self.latest_payloads: dict[str, dict[str, Any]] = {}
        self.recent_messages: deque[dict[str, Any]] = deque(maxlen=300)
        self.current_topic: str | None = None
        self.video_packets = 0
        self.video_frames = 0
        self.video_bytes = 0
        self._last_image: ImageTk.PhotoImage | None = None
        self._last_mqtt_count = 0
        self._last_video_packet_count = 0
        self._last_rate_at = time.time()
        self.mqtt_rate = 0.0
        self.video_rate = 0.0

        self.root = tk.Tk()
        self.root.title("红三实时监控")
        self.root.geometry("1360x820")
        self.root.minsize(1080, 680)
        self.root.configure(bg="#f6f7f9")
        self.style = ttk.Style()
        self.style.theme_use("clam")
        self._configure_style()

        self.decoder = HevcPreviewDecoder(self.images)
        self.video = VideoReceiver(
            bind_host=args.bind,
            port=args.video_port,
            output=args.video_output,
            endian=args.endian,
            frame_callback=self._on_hevc_frame,
        )
        self.mqtt = GuiMqttClient(
            host=args.host,
            port=args.mqtt_port,
            client_id=args.client_id,
            event_queue=self.events,
            subscribe_all=args.subscribe_all,
            custom_byte_callback=self.video.feed_rm_outer_bytes,
        )

        self._build_layout()
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def run(self) -> None:
        self.video.start()
        self.mqtt.start()
        self.root.after(100, self._poll_events)
        self.root.after(100, self._poll_images)
        self.root.mainloop()

    def close(self) -> None:
        self.mqtt.stop()
        self.video.stop()
        self.decoder.stop()
        self.root.destroy()

    def _build_layout(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        header = ttk.Frame(self.root, style="App.TFrame")
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(4, weight=1)
        ttk.Label(header, text="红三实时监控", style="Title.TLabel").grid(
            row=0, column=0, padx=(18, 20), pady=12, sticky="w"
        )
        self.mqtt_badge = ttk.Label(header, text="MQTT 连接中", style="Muted.TLabel")
        self.mqtt_badge.grid(row=0, column=1, padx=5, sticky="w")
        self.video_badge = ttk.Label(header, text="图传等待", style="Muted.TLabel")
        self.video_badge.grid(row=0, column=2, padx=5, sticky="w")
        self.message_badge = ttk.Label(header, text="消息 0", style="Muted.TLabel")
        self.message_badge.grid(row=0, column=3, padx=5, sticky="w")
        self.record_badge = ttk.Label(header, text=f"录制 {self.args.video_output}", style="Muted.TLabel")
        self.record_badge.grid(row=0, column=4, padx=5, sticky="w")

        main = ttk.Frame(self.root, style="App.TFrame")
        main.grid(row=1, column=0, sticky="nsew", padx=14, pady=(0, 14))
        main.columnconfigure(0, weight=0, minsize=360)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(1, weight=1)

        left = ttk.Frame(main, style="Panel.TFrame")
        left.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=(0, 8))
        left.rowconfigure(0, weight=0)
        left.rowconfigure(1, weight=1)
        left.rowconfigure(2, weight=1)
        left.columnconfigure(0, weight=1)

        self.summary_frame = ttk.Frame(left, style="Panel.TFrame")
        self.summary_frame.grid(row=0, column=0, sticky="ew", padx=14, pady=(14, 8))
        self.summary_frame.columnconfigure(0, weight=1)
        self.summary_vars: dict[str, tk.StringVar] = {}
        for index, (key, label, value) in enumerate(
            [
                ("stage", "阶段", "--"),
                ("score", "比分", "--"),
                ("time", "时间", "--"),
                ("robot", "机器人", "--"),
                ("health", "血量/热量", "--"),
                ("position", "坐标", "--"),
                ("module", "图传模块", "--"),
                ("video", "UDP 图传", "--"),
            ]
        ):
            self._add_row_stat(self.summary_frame, index, key, label, value)

        self.topic_tree = ttk.Treeview(left, columns=("topic", "count"), show="headings", height=8, style="Topic.Treeview")
        self.topic_tree.heading("topic", text="Topic")
        self.topic_tree.heading("count", text="次数")
        self.topic_tree.column("topic", width=250, stretch=True)
        self.topic_tree.column("count", width=64, anchor="e", stretch=False)
        self.topic_tree.grid(row=1, column=0, sticky="nsew", padx=14, pady=(6, 8))
        self.topic_tree.bind("<<TreeviewSelect>>", self._select_topic)

        self.message_list = tk.Listbox(
            left,
            height=7,
            bg="#ffffff",
            fg="#30343b",
            selectbackground="#dbeafe",
            selectforeground="#ffffff",
            borderwidth=0,
            highlightthickness=0,
            font=("DejaVu Sans Mono", 10),
        )
        self.message_list.grid(row=2, column=0, sticky="nsew", padx=14, pady=(0, 14))
        self.message_list.bind("<<ListboxSelect>>", self._select_recent_message)

        video_panel = ttk.Frame(main, style="Video.TFrame")
        video_panel.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=(8, 0))
        video_panel.rowconfigure(1, weight=1)
        video_panel.rowconfigure(2, weight=0)
        video_panel.columnconfigure(0, weight=1)
        ttk.Label(video_panel, text="图传", style="VideoTitle.TLabel").grid(row=0, column=0, sticky="w", padx=14, pady=(12, 8))
        self.video_label = tk.Label(
            video_panel,
            text="未收到 UDP 3334 HEVC",
            bg="#111111",
            fg="#9ca3af",
            font=("DejaVu Sans", 18),
            anchor="center",
        )
        self.video_label.grid(row=1, column=0, sticky="nsew", padx=14, pady=(0, 14))
        self.detail_title = ttk.Label(video_panel, text="详情", style="VideoTitle.TLabel")
        self.detail_title.grid(row=2, column=0, sticky="w", padx=14, pady=(0, 4))
        self.detail_text = tk.Text(
            video_panel,
            wrap="none",
            height=8,
            bg="#ffffff",
            fg="#30343b",
            insertbackground="#30343b",
            borderwidth=0,
            highlightthickness=1,
            highlightbackground="#d9dde3",
            font=("DejaVu Sans Mono", 10),
        )
        self.detail_text.grid(row=3, column=0, sticky="ew", padx=14, pady=(0, 14))

    def _configure_style(self) -> None:
        self.style.configure("App.TFrame", background="#f6f7f9")
        self.style.configure("Panel.TFrame", background="#ffffff", relief="flat")
        self.style.configure("Video.TFrame", background="#ffffff", relief="flat")
        self.style.configure("Title.TLabel", background="#f6f7f9", foreground="#171a1f", font=("DejaVu Sans", 17, "bold"))
        self.style.configure("Section.TLabel", background="#ffffff", foreground="#30343b", font=("DejaVu Sans", 11, "bold"))
        self.style.configure("VideoTitle.TLabel", background="#ffffff", foreground="#30343b", font=("DejaVu Sans", 11, "bold"))
        self.style.configure("Muted.TLabel", background="#f6f7f9", foreground="#5f6875", font=("DejaVu Sans", 10))
        self.style.configure("OkBadge.TLabel", background="#f6f7f9", foreground="#15803d", font=("DejaVu Sans", 10, "bold"))
        self.style.configure("WarnBadge.TLabel", background="#f6f7f9", foreground="#b45309", font=("DejaVu Sans", 10, "bold"))
        self.style.configure("BadBadge.TLabel", background="#f6f7f9", foreground="#b91c1c", font=("DejaVu Sans", 10, "bold"))
        self.style.configure("StatLabel.TLabel", background="#ffffff", foreground="#6b7280", font=("DejaVu Sans", 10))
        self.style.configure("StatValue.TLabel", background="#ffffff", foreground="#171a1f", font=("DejaVu Sans", 13, "bold"))
        self.style.configure(
            "Topic.Treeview",
            background="#ffffff",
            foreground="#30343b",
            fieldbackground="#ffffff",
            borderwidth=0,
            rowheight=24,
            font=("DejaVu Sans Mono", 10),
        )
        self.style.configure(
            "Topic.Treeview.Heading",
            background="#f0f2f5",
            foreground="#30343b",
            font=("DejaVu Sans", 10, "bold"),
        )
        self.style.map("Topic.Treeview", background=[("selected", "#dbeafe")], foreground=[("selected", "#111827")])

    def _add_row_stat(self, parent: ttk.Frame, row: int, key: str, label: str, value: str) -> None:
        var = tk.StringVar(value=value)
        self.summary_vars[key] = var
        ttk.Label(parent, text=label, style="StatLabel.TLabel").grid(row=row, column=0, sticky="w", pady=3)
        ttk.Label(parent, textvariable=var, style="StatValue.TLabel").grid(row=row, column=1, sticky="e", pady=3)
        parent.columnconfigure(1, weight=1)

    def _on_hevc_frame(self, hevc_frame: bytes) -> None:
        self.video_frames += 1
        self.video_bytes += len(hevc_frame)
        self.decoder.feed(hevc_frame)

    def _poll_events(self) -> None:
        changed_topics = False
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            if event["kind"] == "mqtt_status":
                pass
            elif event["kind"] == "mqtt":
                self.topic_counts[event["topic"]] = event["count"]
                self.latest_payloads[event["topic"]] = event["payload"]
                self.recent_messages.append(event)
                changed_topics = True
        if changed_topics:
            self._refresh_topic_tree()
            self._refresh_recent_messages()
            self._refresh_summary_cards()
            if self.current_topic is not None:
                self._show_payload(self.current_topic, self.latest_payloads.get(self.current_topic, {}))

        self.video_packets = self.video.stats.packets
        self._refresh_rates()
        self._refresh_badges()
        self._refresh_summary_cards()
        self.root.after(100, self._poll_events)

    def _poll_images(self) -> None:
        image = None
        while True:
            try:
                image = self.images.get_nowait()
            except queue.Empty:
                break
        if image is not None:
            width = max(320, self.video_label.winfo_width())
            height = max(240, self.video_label.winfo_height())
            image.thumbnail((width, height), Image.Resampling.LANCZOS)
            self._last_image = ImageTk.PhotoImage(image)
            self.video_label.configure(image=self._last_image, text="")
        elif self.video.stats.packets == 0:
            self.video_label.configure(text="未收到 UDP 3334 HEVC", image="")
            self._last_image = None
        elif self.video_frames > 0 and self.decoder.waiting_for_keyframe:
            self.video_label.configure(text="已收到图传，等待关键帧解码", image="")
            self._last_image = None
        elif self.video_frames > 0 and self.decoder.frames_decoded == 0:
            self.video_label.configure(text="已收到关键帧，正在启动解码", image="")
            self._last_image = None
        self.root.after(33, self._poll_images)

    def _refresh_topic_tree(self) -> None:
        selected = self.current_topic
        for item in self.topic_tree.get_children():
            self.topic_tree.delete(item)
        for topic, count in self.topic_counts.most_common():
            self.topic_tree.insert("", "end", iid=topic, values=(topic, count))
        if selected in self.topic_counts:
            self.topic_tree.selection_set(selected)

    def _refresh_recent_messages(self) -> None:
        self.message_list.delete(0, tk.END)
        for event in list(self.recent_messages)[-80:]:
            self.message_list.insert(
                tk.END,
                f"{event['time']}  {event['topic']}  {event['bytes']}B  #{event['count']}",
            )

    def _select_topic(self, _event: tk.Event[tk.Widget]) -> None:
        selected = self.topic_tree.selection()
        if not selected:
            return
        self.current_topic = selected[0]
        self._show_payload(self.current_topic, self.latest_payloads.get(self.current_topic, {}))

    def _select_recent_message(self, _event: tk.Event[tk.Widget]) -> None:
        selected = self.message_list.curselection()
        if not selected:
            return
        recent = list(self.recent_messages)[-80:]
        event = recent[selected[0]]
        self.current_topic = event["topic"]
        self._show_payload(event["topic"], event["payload"])

    def _show_payload(self, topic: str, payload: dict[str, Any]) -> None:
        self.detail_title.configure(text=f"{topic} 最新数据")
        self.detail_text.delete("1.0", tk.END)
        self.detail_text.insert(tk.END, json.dumps(payload, ensure_ascii=False, indent=2))

    def _refresh_rates(self) -> None:
        now = time.time()
        elapsed = now - self._last_rate_at
        if elapsed < 1.0:
            return
        self.mqtt_rate = (self.mqtt.messages - self._last_mqtt_count) / elapsed
        self.video_rate = (self.video.stats.packets - self._last_video_packet_count) / elapsed
        self._last_mqtt_count = self.mqtt.messages
        self._last_video_packet_count = self.video.stats.packets
        self._last_rate_at = now

    def _refresh_badges(self) -> None:
        if self.mqtt.connected:
            self.mqtt_badge.configure(text=f"MQTT 正常 {self.mqtt_rate:.1f}/s", style="OkBadge.TLabel")
        else:
            self.mqtt_badge.configure(text="MQTT 断开", style="BadBadge.TLabel")
        if self.video.stats.packets:
            if self.decoder.frames_decoded:
                self.video_badge.configure(
                    text=f"图传 {self.video_rate:.1f} pkt/s 显示 {self.decoder.frames_decoded}",
                    style="OkBadge.TLabel",
                )
            elif self.decoder.waiting_for_keyframe:
                self.video_badge.configure(text=f"图传 {self.video_rate:.1f} pkt/s 等关键帧", style="WarnBadge.TLabel")
            else:
                self.video_badge.configure(text=f"图传 {self.video_rate:.1f} pkt/s 启动解码", style="WarnBadge.TLabel")
        else:
            self.video_badge.configure(text="图传 0 包", style="WarnBadge.TLabel")
        self.message_badge.configure(text=f"消息 {self.mqtt.messages}", style="Muted.TLabel")
        self.record_badge.configure(text=f"录制 {self.args.video_output}", style="Muted.TLabel")

    def _refresh_summary_cards(self) -> None:
        game = self.latest_payloads.get("GameStatus", {})
        dynamic = self.latest_payloads.get("RobotDynamicStatus", {})
        static = self.latest_payloads.get("RobotStaticStatus", {})
        module = self.latest_payloads.get("RobotModuleStatus", {})
        position = self.latest_payloads.get("RobotPosition", {})

        stage_names = {
            0: "未开始",
            1: "准备",
            2: "自检",
            3: "倒计时",
            4: "比赛中",
            5: "结算",
        }
        stage = game.get("current_stage")
        stage_text = stage_names.get(stage, str(stage)) if stage is not None else "--"
        if game.get("is_paused"):
            stage_text += " 暂停"
        self.summary_vars["stage"].set(stage_text)
        self.summary_vars["score"].set(f"红 {game.get('red_score', '--')} : 蓝 {game.get('blue_score', '--')}")
        countdown = game.get("stage_countdown_sec", "--")
        elapsed = game.get("stage_elapsed_sec", "--")
        self.summary_vars["time"].set(f"剩 {countdown}s / 已 {elapsed}s")

        robot_id = static.get("robot_id", self.args.client_id)
        level = static.get("level", "--")
        alive = static.get("alive_state", "--")
        self.summary_vars["robot"].set(f"ID {robot_id}  Lv {level}  状态 {alive}")
        hp = dynamic.get("current_health", "--")
        heat = dynamic.get("current_heat", "--")
        ammo = dynamic.get("remaining_ammo", "--")
        self.summary_vars["health"].set(f"HP {hp}  热 {heat}  弹 {ammo}")
        x = position.get("x", "--")
        y = position.get("y", "--")
        yaw = position.get("yaw", "--")
        self.summary_vars["position"].set(f"x {self._fmt_num(x)}  y {self._fmt_num(y)}  yaw {self._fmt_num(yaw)}")
        vt = module.get("video_transmission")
        vt_text = {0: "离线", 1: "在线", 2: "安装异常"}.get(vt, "--")
        self.summary_vars["module"].set(vt_text)
        decoded = self.decoder.frames_decoded
        self.summary_vars["video"].set(f"{self.video.stats.packets} 包 / {self.video_frames} 帧 / 显示 {decoded}")

    @staticmethod
    def _fmt_num(value: Any) -> str:
        if isinstance(value, float):
            return f"{value:.2f}"
        return str(value)


def _tcp_probe(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=2.0):
            return True
    except OSError:
        return False


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="RoboMaster custom client realtime GUI")
    parser.add_argument("--host", default="192.168.12.1")
    parser.add_argument("--mqtt-port", type=int, default=3333)
    parser.add_argument("--video-port", type=int, default=3334)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--client-id", default="3")
    parser.add_argument("--subscribe-all", action="store_true")
    parser.add_argument("--video-output", type=Path, default=Path("captures/red3_gui.hevc"))
    parser.add_argument("--endian", choices=("little", "big"), default="little")
    parser.add_argument("--headless-test", action="store_true")
    parser.add_argument("--log-level", default="INFO")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.video_output.exists() and args.video_output.is_dir():
        args.video_output = args.video_output / "red3_gui.hevc"
    elif str(args.video_output).endswith("/"):
        args.video_output = args.video_output / "red3_gui.hevc"
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    if args.headless_test:
        ok = _tcp_probe(args.host, args.mqtt_port)
        print(json.dumps({"mqtt_tcp_reachable": ok, "host": args.host, "port": args.mqtt_port}))
        return 0 if ok else 2
    RmCustomClientWindow(args).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

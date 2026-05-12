from __future__ import annotations

import logging
import socket
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

LOGGER = logging.getLogger(__name__)

RM_OUTER_SOF = 0xA5
RM_OUTER_CMD_ID = 0x0310
RM_OUTER_DATA_LEN = 300
RM_OUTER_FRAME_LEN = 5 + 2 + RM_OUTER_DATA_LEN + 2
MAX_VIDEO_BYTES = 20 * 1024 * 1024
MAX_OUTER_BUFFER_BYTES = RM_OUTER_FRAME_LEN * 32


def _crc8(data: bytes) -> int:
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = ((crc >> 1) ^ 0x8C) & 0xFF
            else:
                crc = (crc >> 1) & 0xFF
    return crc


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = ((crc >> 1) ^ 0x8408) & 0xFFFF
            else:
                crc = (crc >> 1) & 0xFFFF
    return crc


@dataclass
class VideoStats:
    chunks: int = 0
    packets: int = 0
    bytes: int = 0
    outer_frames: int = 0
    frames_completed: int = 0
    frames_dropped: int = 0
    first_packet_at: float | None = None
    last_packet_at: float | None = None
    last_addr: tuple[str, int] | None = None


@dataclass
class _FrameBuffer:
    frame_no: int
    total_bytes: int
    chunks: dict[int, bytes] = field(default_factory=dict)
    created_at: float = field(default_factory=time.time)
    updated_at: float = field(default_factory=time.time)

    def add(self, frag_no: int, payload: bytes) -> None:
        self.chunks[frag_no] = payload
        self.updated_at = time.time()

    def assembled(self) -> bytes:
        return b"".join(self.chunks[index] for index in sorted(self.chunks))


class VideoReceiver(threading.Thread):
    def __init__(
        self,
        bind_host: str,
        port: int,
        output: Path | None,
        preview: bool = False,
        endian: str = "little",
        frame_timeout_sec: float = 0.5,
        frame_callback: Callable[[bytes], None] | None = None,
        receive_buffer_bytes: int = 32 * 1024 * 1024,
    ) -> None:
        super().__init__(name="video-receiver", daemon=True)
        self.bind_host = bind_host
        self.port = port
        self.output = output
        self.preview = preview
        self.endian = endian
        self.frame_timeout_sec = frame_timeout_sec
        self.frame_callback = frame_callback
        self.receive_buffer_bytes = receive_buffer_bytes
        self.stats = VideoStats()
        self._stopping = threading.Event()
        self._frames: dict[int, _FrameBuffer] = {}
        self._outer_buffer = bytearray()
        self._file = None
        self._ffplay: subprocess.Popen[bytes] | None = None
        self._last_file_flush = 0.0
        self._packet_lock = threading.Lock()
        self._output_lock = threading.Lock()

    def stop(self) -> None:
        self._stopping.set()

    def feed_rm_outer_bytes(self, data: bytes, addr: tuple[str, int] | None = None) -> None:
        if not data:
            return

        with self._packet_lock:
            source_addr = addr or ("mqtt", 0)
            self._note_chunk_locked(data, source_addr)
            completed = self._consume_direct_inner_packets_locked(data, source_addr)
            if not completed:
                completed = self._consume_rm_outer_stream_locked(data, source_addr)
            self._drop_stale_frames_locked()

        for frame_data in completed:
            self._write_hevc(frame_data)

    def run(self) -> None:
        if self.output is not None:
            self.output.parent.mkdir(parents=True, exist_ok=True)
            self._file = self.output.open("ab")
            LOGGER.info("append HEVC stream to %s", self.output)

        if self.preview:
            self._ffplay = subprocess.Popen(
                [
                    "ffplay",
                    "-hide_banner",
                    "-loglevel",
                    "warning",
                    "-fflags",
                    "nobuffer",
                    "-flags",
                    "low_delay",
                    "-f",
                    "hevc",
                    "-i",
                    "pipe:0",
                ],
                stdin=subprocess.PIPE,
            )
            LOGGER.info("started ffplay preview")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.receive_buffer_bytes)
        sock.bind((self.bind_host, self.port))
        sock.settimeout(0.2)
        LOGGER.info("listening UDP HEVC on %s:%s", self.bind_host, self.port)

        try:
            while not self._stopping.is_set():
                try:
                    packet, addr = sock.recvfrom(65535)
                except socket.timeout:
                    with self._packet_lock:
                        self._drop_stale_frames_locked()
                    continue

                self._handle_udp_packet(packet, addr)
        finally:
            sock.close()
            if self._file is not None:
                self._file.flush()
                self._file.close()
            if self._ffplay is not None:
                if self._ffplay.stdin is not None:
                    self._ffplay.stdin.close()
                self._ffplay.terminate()

    def _note_chunk_locked(self, data: bytes, addr: tuple[str, int]) -> None:
        now = time.time()
        self.stats.chunks += 1
        self.stats.bytes += len(data)
        self.stats.first_packet_at = self.stats.first_packet_at or now
        self.stats.last_packet_at = now
        self.stats.last_addr = addr

    def _handle_udp_packet(self, packet: bytes, addr: tuple[str, int]) -> None:
        with self._packet_lock:
            self._note_chunk_locked(packet, addr)
            completed = self._consume_rm_outer_stream_locked(packet, addr)
            if not completed and not self._outer_buffer:
                complete = self._consume_inner_packet_locked(packet, addr)
                completed = [] if complete is None else [complete]
            self._drop_stale_frames_locked()

        for frame_data in completed:
            self._write_hevc(frame_data)

    def _looks_like_rm_outer_frame(self, packet: bytes) -> bool:
        if len(packet) < RM_OUTER_FRAME_LEN or packet[0] != RM_OUTER_SOF:
            return False

        data_length = int.from_bytes(packet[1:3], "little")
        if data_length != RM_OUTER_DATA_LEN:
            return False

        if _crc8(packet[:4]) != packet[4]:
            return False

        total_len = 5 + 2 + data_length + 2
        return len(packet) >= total_len

    def _consume_rm_outer_stream_locked(
        self,
        data: bytes,
        addr: tuple[str, int],
    ) -> list[bytes]:
        self._outer_buffer.extend(data)
        completed: list[bytes] = []

        while True:
            sof_index = self._outer_buffer.find(bytes((RM_OUTER_SOF,)))
            if sof_index < 0:
                if len(self._outer_buffer) > RM_OUTER_FRAME_LEN:
                    LOGGER.debug("drop %d non-frame bytes from RM outer parser", len(self._outer_buffer))
                    self._outer_buffer.clear()
                break

            if sof_index > 0:
                del self._outer_buffer[:sof_index]

            if len(self._outer_buffer) < 5:
                break

            data_length = int.from_bytes(self._outer_buffer[1:3], "little")
            total_len = 5 + 2 + data_length + 2

            if data_length != RM_OUTER_DATA_LEN:
                del self._outer_buffer[0]
                continue

            if len(self._outer_buffer) < total_len:
                if len(self._outer_buffer) > MAX_OUTER_BUFFER_BYTES:
                    LOGGER.warning(
                        "RM outer parser buffer grew to %d bytes without a full frame; clearing",
                        len(self._outer_buffer),
                    )
                    self._outer_buffer.clear()
                break

            candidate = bytes(self._outer_buffer[:total_len])
            if _crc8(candidate[:4]) != candidate[4]:
                del self._outer_buffer[0]
                continue

            expected_crc16 = int.from_bytes(candidate[-2:], "little")
            actual_crc16 = _crc16(candidate[:-2])
            if actual_crc16 != expected_crc16:
                LOGGER.debug(
                    "ignore RM outer frame with bad CRC16 expected=0x%04x actual=0x%04x",
                    expected_crc16,
                    actual_crc16,
                )
                del self._outer_buffer[0]
                continue

            cmd_id = int.from_bytes(candidate[5:7], "little")
            if cmd_id != RM_OUTER_CMD_ID:
                LOGGER.debug("ignore RM outer frame cmd_id=0x%04x", cmd_id)
                del self._outer_buffer[:total_len]
                continue

            inner_packet = candidate[7:-2]
            del self._outer_buffer[:total_len]
            self.stats.outer_frames += 1
            complete = self._consume_inner_packet_locked(inner_packet, addr)
            if complete is not None:
                completed.append(complete)

        return completed

    def _consume_direct_inner_packets_locked(
        self,
        data: bytes,
        addr: tuple[str, int],
    ) -> list[bytes]:
        if len(data) < 8 or len(data) % RM_OUTER_DATA_LEN != 0:
            return []

        completed: list[bytes] = []
        for offset in range(0, len(data), RM_OUTER_DATA_LEN):
            packet = data[offset : offset + RM_OUTER_DATA_LEN]
            if not self._looks_like_valid_inner_packet(packet):
                return []

            complete = self._consume_inner_packet_locked(packet, addr)
            if complete is not None:
                completed.append(complete)

        return completed

    def _looks_like_valid_inner_packet(self, packet: bytes) -> bool:
        if len(packet) < 8:
            return False

        total_bytes = int.from_bytes(packet[4:8], self.endian)
        if total_bytes <= 0 or total_bytes > MAX_VIDEO_BYTES:
            return False

        frag_no = int.from_bytes(packet[2:4], self.endian)
        payload_len = len(packet) - 8
        if payload_len <= 0:
            return False

        # Sender-side frag_no is a fragment index (0, 1, 2, ...), not a byte offset.
        # Each inner packet always carries a fixed 292-byte payload region and the last
        # fragment is zero-padded, so total_bytes can be smaller than payload_len.
        expected_fragments = max(1, (total_bytes + payload_len - 1) // payload_len)
        return frag_no < expected_fragments

    def _consume_inner_packet_locked(self, packet: bytes, addr: tuple[str, int]) -> bytes | None:
        if len(packet) < 8:
            LOGGER.warning("short UDP packet from %s: %d bytes", addr, len(packet))
            return None

        frame_no = int.from_bytes(packet[0:2], self.endian)
        frag_no = int.from_bytes(packet[2:4], self.endian)
        total_bytes = int.from_bytes(packet[4:8], self.endian)
        payload = packet[8:]

        if total_bytes <= 0 or total_bytes > MAX_VIDEO_BYTES:
            LOGGER.warning(
                "invalid video header addr=%s frame=%d frag=%d total=%d len=%d",
                addr,
                frame_no,
                frag_no,
                total_bytes,
                len(packet),
            )
            return None

        frame = self._frames.get(frame_no)
        if frame is None or frame.total_bytes != total_bytes:
            frame = _FrameBuffer(frame_no=frame_no, total_bytes=total_bytes)
            self._frames[frame_no] = frame
        frame.add(frag_no, payload)
        self.stats.packets += 1

        data = frame.assembled()
        if len(data) >= total_bytes:
            complete = data[:total_bytes]
            self._frames.pop(frame_no, None)
            self.stats.frames_completed += 1
            LOGGER.debug(
                "video frame=%d fragments=%d bytes=%d from=%s",
                frame_no,
                len(frame.chunks),
                len(complete),
                addr,
            )
            return complete

        return None

    def _write_hevc(self, data: bytes) -> None:
        with self._output_lock:
            if self._file is not None:
                self._file.write(data)
                now = time.time()
                if now - self._last_file_flush >= 1.0:
                    self._file.flush()
                    self._last_file_flush = now
            if self.frame_callback is not None:
                self.frame_callback(data)
            if self._ffplay is not None and self._ffplay.stdin is not None:
                try:
                    self._ffplay.stdin.write(data)
                    self._ffplay.stdin.flush()
                except BrokenPipeError:
                    LOGGER.warning("ffplay pipe closed")
                    self._ffplay = None

    def _drop_stale_frames_locked(self) -> None:
        now = time.time()
        stale = [
            frame_no
            for frame_no, frame in self._frames.items()
            if now - frame.updated_at > self.frame_timeout_sec
        ]
        for frame_no in stale:
            frame = self._frames.pop(frame_no)
            self.stats.frames_dropped += 1
            LOGGER.debug(
                "drop stale frame=%d fragments=%d total=%d",
                frame.frame_no,
                len(frame.chunks),
                frame.total_bytes,
            )

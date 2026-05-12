#!/usr/bin/env python3
"""Receive 300-byte inner video packets from serial and rebuild HEVC frames."""

from __future__ import annotations

import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

import serial
import serial.tools.list_ports


SERIAL_BAUDRATE = 921600
VIDEO_FORMAT = "hevc"
SAVE_OUTPUT = "serial_video.hevc"
PREVIEW = True

PACKET_SIZE = 300
HEADER_SIZE = 8
PAYLOAD_SIZE = 292
MAX_FRAME_BYTES = 20 * 1024 * 1024
FRAME_TIMEOUT_SEC = 0.5
STATS_EVERY_SEC = 1.0


@dataclass
class FrameBuffer:
    frame_no: int
    total_bytes: int
    chunks: dict[int, bytes] = field(default_factory=dict)
    updated_at: float = field(default_factory=time.time)

    def add(self, frag_no: int, payload: bytes) -> None:
        self.chunks[frag_no] = payload
        self.updated_at = time.time()

    def assembled(self) -> bytes:
        return b"".join(self.chunks[index] for index in sorted(self.chunks))


def find_serial_port() -> str | None:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial devices found")
        return None
    print("Detected serial devices:")
    for i, port in enumerate(ports, start=1):
        print(f"  {i}. {port.device} - {port.description}")
    selected = ports[0].device
    print(f"Auto-select serial port: {selected}")
    return selected


def start_ffplay_preview() -> subprocess.Popen[bytes]:
    cmd = [
        "ffplay",
        "-hide_banner",
        "-loglevel",
        "warning",
        "-fflags",
        "nobuffer",
        "-flags",
        "low_delay",
        "-f",
        VIDEO_FORMAT,
        "-i",
        "pipe:0",
        "-window_title",
        "Serial Video Preview",
        "-autoexit",
    ]
    return subprocess.Popen(
        cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def drop_stale_frames(frames: dict[int, FrameBuffer], dropped: list[int]) -> None:
    now = time.time()
    stale = [
        frame_no
        for frame_no, frame in frames.items()
        if now - frame.updated_at > FRAME_TIMEOUT_SEC
    ]
    for frame_no in stale:
        frame = frames.pop(frame_no)
        dropped[0] += 1
        print(
            f"\nDrop stale frame={frame.frame_no} "
            f"fragments={len(frame.chunks)} total={frame.total_bytes}"
        )


def main() -> None:
    port = find_serial_port()
    if not port:
        return

    try:
        ser = serial.Serial(
            port=port,
            baudrate=SERIAL_BAUDRATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.05,
        )
        print(f"Serial opened: {port} @ {SERIAL_BAUDRATE}")
    except Exception as exc:
        print(f"Failed to open serial: {exc}")
        return

    video_path = Path(SAVE_OUTPUT)
    video_file = video_path.open("wb")
    print(f"Saving video to: {video_path}")

    ffplay = None
    if PREVIEW:
        ffplay = start_ffplay_preview()
        print("Preview started")

    frames: dict[int, FrameBuffer] = {}
    buffer = bytearray()
    completed_frames = 0
    dropped_frames = [0]
    total_video_bytes = 0
    packets = 0
    last_stats_at = time.time()

    print("\nReceiving inner packets. Press Ctrl+C to stop.\n")

    try:
        while True:
            incoming = ser.read(4096)
            if incoming:
                buffer.extend(incoming)
            else:
                drop_stale_frames(frames, dropped_frames)
                continue

            while len(buffer) >= PACKET_SIZE:
                packet = bytes(buffer[:PACKET_SIZE])
                del buffer[:PACKET_SIZE]
                packets += 1

                frame_no = int.from_bytes(packet[0:2], "little")
                frag_no = int.from_bytes(packet[2:4], "little")
                total_bytes = int.from_bytes(packet[4:8], "little")
                payload = packet[HEADER_SIZE : HEADER_SIZE + PAYLOAD_SIZE]

                if total_bytes <= 0 or total_bytes > MAX_FRAME_BYTES:
                    print(
                        f"\nInvalid header: frame={frame_no} frag={frag_no} total={total_bytes}"
                    )
                    frames.clear()
                    buffer.clear()
                    break

                frame = frames.get(frame_no)
                if frame is None or frame.total_bytes != total_bytes:
                    frame = FrameBuffer(frame_no=frame_no, total_bytes=total_bytes)
                    frames[frame_no] = frame
                frame.add(frag_no, payload)

                assembled = frame.assembled()
                if len(assembled) >= total_bytes:
                    complete = assembled[:total_bytes]
                    frames.pop(frame_no, None)
                    completed_frames += 1
                    total_video_bytes += len(complete)

                    video_file.write(complete)
                    if ffplay is not None and ffplay.stdin is not None:
                        try:
                            ffplay.stdin.write(complete)
                            ffplay.stdin.flush()
                        except BrokenPipeError:
                            ffplay = None

                now = time.time()
                if now - last_stats_at >= STATS_EVERY_SEC:
                    print(
                        "\r"
                        f"packets={packets} frames={completed_frames} "
                        f"dropped={dropped_frames[0]} "
                        f"video={total_video_bytes // 1024}KB "
                        f"buffer={len(buffer)}B cached_frames={len(frames)}",
                        end="",
                    )
                    video_file.flush()
                    last_stats_at = now

            drop_stale_frames(frames, dropped_frames)

    except KeyboardInterrupt:
        print("\n\nStopped by user")

    finally:
        ser.close()
        video_file.flush()
        video_file.close()
        if ffplay is not None:
            if ffplay.stdin is not None:
                ffplay.stdin.close()
            ffplay.terminate()
        print(f"Saved video to: {video_path}")


if __name__ == "__main__":
    main()

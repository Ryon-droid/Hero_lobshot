#!/usr/bin/env python3
"""Send only the 300-byte inner video packet over serial for testing.

Inner packet layout (little-endian):
  - frame_no:    2 bytes
  - frag_no:     2 bytes
  - total_bytes: 4 bytes
  - payload:   292 bytes

This script intentionally bypasses the outer RoboMaster frame and writes the
inner packet bytes directly to the serial port.

For real transmission tests, use --input-file to send actual encoded video
bytes split into inner packets:
  frame_no(2) + frag_no(2) + total_bytes(4) + payload(292)
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial is required. Install it with: sudo apt install python3-serial"
    ) from exc


HEADER_BYTES = 8
PAYLOAD_BYTES = 292
PACKET_BYTES = HEADER_BYTES + PAYLOAD_BYTES


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send raw 300-byte inner video packets over serial."
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port path")
    parser.add_argument("--baudrate", type=int, default=921600, help="Serial baudrate")
    parser.add_argument("--hz", type=float, default=48.0, help="Send rate in Hz")
    parser.add_argument(
        "--frame-no", type=int, default=0, help="Initial 16-bit frame number"
    )
    parser.add_argument(
        "--frag-no", type=int, default=0, help="Initial 16-bit fragment number"
    )
    parser.add_argument(
        "--total-bytes",
        type=int,
        default=168,
        help="4-byte total frame byte count written into the inner header",
    )
    parser.add_argument(
        "--pattern",
        choices=("zero", "counter", "hex"),
        default="counter",
        help="Payload fill pattern",
    )
    parser.add_argument(
        "--payload-hex",
        default="",
        help="Hex payload bytes when --pattern hex is used, e.g. '01 02 aa ff'",
    )
    parser.add_argument(
        "--payload-file",
        default="",
        help="Optional file containing payload bytes; supports raw binary or hex text",
    )
    parser.add_argument(
        "--input-file",
        default="",
        help="Real binary frame file to send, e.g. .h265/.bin; split into 292B payload fragments",
    )
    parser.add_argument(
        "--loop-file",
        action="store_true",
        help="Loop the input file forever; each pass uses a new frame_no",
    )
    parser.add_argument(
        "--frame-step",
        type=int,
        default=1,
        help="frame_no increment after each send",
    )
    parser.add_argument(
        "--frag-step",
        type=int,
        default=0,
        help="frag_no increment after each send",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="Number of packets to send; 0 means send forever",
    )
    parser.add_argument(
        "--preview-every",
        type=int,
        default=50,
        help="Print one packet preview every N packets; 0 disables preview",
    )
    return parser.parse_args()


def _load_payload_from_file(path_str: str) -> bytes:
    path = Path(path_str)
    raw = path.read_bytes()
    try:
        text = raw.decode("ascii")
    except UnicodeDecodeError:
        return raw

    stripped = "".join(text.split())
    if not stripped:
        return b""
    if len(stripped) % 2 != 0:
        raise ValueError(f"hex text in {path} has odd length")
    return bytes.fromhex(stripped)


def _build_payload(args: argparse.Namespace, packet_index: int) -> bytes:
    if args.payload_file:
        src = _load_payload_from_file(args.payload_file)
    elif args.pattern == "hex":
        src = bytes.fromhex("".join(args.payload_hex.split()))
    elif args.pattern == "zero":
        src = bytes(PAYLOAD_BYTES)
    else:
        src = bytes(((packet_index + i) & 0xFF) for i in range(PAYLOAD_BYTES))

    if len(src) >= PAYLOAD_BYTES:
        return src[:PAYLOAD_BYTES]
    return src + bytes(PAYLOAD_BYTES - len(src))


def build_packet(
    frame_no: int,
    frag_no: int,
    total_bytes: int,
    payload: bytes,
) -> bytes:
    if len(payload) != PAYLOAD_BYTES:
        raise ValueError(f"payload must be {PAYLOAD_BYTES} bytes")

    packet = bytearray(PACKET_BYTES)
    packet[0:2] = (frame_no & 0xFFFF).to_bytes(2, "little")
    packet[2:4] = (frag_no & 0xFFFF).to_bytes(2, "little")
    packet[4:8] = (total_bytes & 0xFFFFFFFF).to_bytes(4, "little")
    packet[8:] = payload
    return bytes(packet)


def hex_preview(data: bytes, limit: int = 32) -> str:
    shown = " ".join(f"{b:02x}" for b in data[:limit])
    if len(data) > limit:
        shown += " ..."
    return shown


def _iter_file_packets(
    frame_bytes: bytes,
    frame_no: int,
) -> list[tuple[int, int, int, bytes]]:
    total_bytes = len(frame_bytes)
    packets: list[tuple[int, int, int, bytes]] = []
    frag_no = 0
    offset = 0

    while offset < total_bytes:
        chunk = frame_bytes[offset : offset + PAYLOAD_BYTES]
        if len(chunk) < PAYLOAD_BYTES:
            chunk = chunk + bytes(PAYLOAD_BYTES - len(chunk))
        packets.append((frame_no, frag_no, total_bytes, chunk))
        frag_no += 1
        offset += PAYLOAD_BYTES

    if not packets:
        packets.append((frame_no, 0, 0, bytes(PAYLOAD_BYTES)))

    return packets


def main() -> int:
    args = parse_args()
    if args.hz <= 0:
        raise SystemExit("--hz must be > 0")
    if args.total_bytes < 0:
        raise SystemExit("--total-bytes must be >= 0")
    if args.input_file and (args.pattern != "counter" or args.payload_hex or args.payload_file):
        print(
            "--input-file is set; pattern/payload-file/payload-hex are ignored",
            file=sys.stderr,
        )

    period = 1.0 / args.hz
    sent = 0
    frame_no = args.frame_no & 0xFFFF
    frag_no = args.frag_no & 0xFFFF
    next_deadline = time.monotonic()

    file_packets: list[tuple[int, int, int, bytes]] = []
    if args.input_file:
        frame_bytes = Path(args.input_file).read_bytes()
        if not frame_bytes:
            raise SystemExit(f"input file is empty: {args.input_file}")
        file_packets = _iter_file_packets(frame_bytes, frame_no)

    with serial.Serial(args.port, args.baudrate, timeout=0) as ser:
        print(
            f"Sending inner packets on {args.port} @ {args.baudrate}, "
            f"{args.hz:.3f}Hz, packet={PACKET_BYTES}B"
        )
        if args.input_file:
            print(
                f"Input frame: {args.input_file}, bytes={len(frame_bytes)}, "
                f"fragments={len(file_packets)}, loop={'on' if args.loop_file else 'off'}"
            )

        file_packet_index = 0
        while args.count == 0 or sent < args.count:
            if args.input_file:
                if file_packet_index >= len(file_packets):
                    if not args.loop_file:
                        break
                    frame_no = (frame_no + args.frame_step) & 0xFFFF
                    file_packets = _iter_file_packets(frame_bytes, frame_no)
                    file_packet_index = 0

                tx_frame_no, tx_frag_no, tx_total_bytes, payload = file_packets[file_packet_index]
                file_packet_index += 1
                packet = build_packet(tx_frame_no, tx_frag_no, tx_total_bytes, payload)
            else:
                payload = _build_payload(args, sent)
                tx_frame_no = frame_no
                tx_frag_no = frag_no
                tx_total_bytes = args.total_bytes
                packet = build_packet(tx_frame_no, tx_frag_no, tx_total_bytes, payload)

            written = ser.write(packet)
            ser.flush()
            if written != PACKET_BYTES:
                print(
                    f"short write: expected {PACKET_BYTES}, got {written}",
                    file=sys.stderr,
                )

            sent += 1
            if args.preview_every > 0 and (sent == 1 or sent % args.preview_every == 0):
                print(
                    f"tx#{sent} frame={tx_frame_no} frag={tx_frag_no} total={tx_total_bytes} "
                    f"preview={hex_preview(packet)}"
                )

            if not args.input_file:
                frame_no = (frame_no + args.frame_step) & 0xFFFF
                frag_no = (frag_no + args.frag_step) & 0xFFFF

            next_deadline += period
            sleep_time = next_deadline - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_deadline = time.monotonic()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

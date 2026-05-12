from __future__ import annotations

import argparse
import logging
import socket
import sys
import time
from pathlib import Path

from rm_custom_client.mqtt_receiver import DEFAULT_TOPICS, MqttReceiver
from rm_custom_client.video_receiver import VideoReceiver


def _tcp_probe(host: str, port: int, timeout: float) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError as exc:
        logging.error("TCP probe failed %s:%s: %s", host, port, exc)
        return False


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Receive RoboMaster custom-client MQTT protobuf data and UDP HEVC video.",
    )
    parser.add_argument("--host", default="192.168.12.1", help="official client/server IP")
    parser.add_argument("--mqtt-port", type=int, default=3333)
    parser.add_argument("--video-port", type=int, default=3334)
    parser.add_argument("--bind", default="0.0.0.0", help="local UDP bind address")
    parser.add_argument(
        "--client-id",
        default="1",
        help="MQTT client ID; protocol says this should be the connected robot ID",
    )
    parser.add_argument("--mqtt-only", action="store_true")
    parser.add_argument("--video-only", action="store_true")
    parser.add_argument("--subscribe-all", action="store_true", help="subscribe MQTT wildcard #")
    parser.add_argument(
        "--topic",
        action="append",
        dest="topics",
        help="MQTT topic to subscribe; may be repeated. Defaults to known server topics.",
    )
    parser.add_argument(
        "--no-print-payloads",
        action="store_true",
        help="only print counters/logs, not decoded protobuf JSON payloads",
    )
    parser.add_argument(
        "--video-output",
        type=Path,
        default=Path("captures/video.hevc"),
        help="append reconstructed HEVC stream here; use 'none' to disable",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="pipe reconstructed HEVC stream to ffplay for live preview",
    )
    parser.add_argument(
        "--endian",
        choices=("little", "big"),
        default="little",
        help="video inner header endian after extracting HEVC chunks; use big only if your sender differs",
    )
    parser.add_argument(
        "--probe-seconds",
        type=float,
        default=0.0,
        help="run for N seconds then exit with receive summary",
    )
    parser.add_argument("--log-level", default="INFO")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    if args.video_output is not None:
        if args.video_output.exists() and args.video_output.is_dir():
            args.video_output = args.video_output / "video.hevc"
        elif str(args.video_output).endswith("/"):
            args.video_output = args.video_output / "video.hevc"

    if args.mqtt_only and args.video_only:
        raise SystemExit("--mqtt-only and --video-only cannot be used together")

    mqtt_receiver: MqttReceiver | None = None
    video_receiver: VideoReceiver | None = None

    if not args.mqtt_only:
        video_output = None if str(args.video_output).lower() == "none" else args.video_output
        video_receiver = VideoReceiver(
            bind_host=args.bind,
            port=args.video_port,
            output=video_output,
            preview=args.preview,
            endian=args.endian,
        )
        video_receiver.start()

    if not args.video_only:
        _tcp_probe(args.host, args.mqtt_port, timeout=2.0)
        topics = args.topics or list(DEFAULT_TOPICS)
        mqtt_receiver = MqttReceiver(
            host=args.host,
            port=args.mqtt_port,
            client_id=args.client_id,
            topics=topics,
            subscribe_all=args.subscribe_all,
            print_payloads=not args.no_print_payloads,
            custom_byte_callback=None if video_receiver is None else video_receiver.feed_rm_outer_bytes,
        )
        mqtt_receiver.start()

    started = time.time()
    try:
        while True:
            time.sleep(2.0)
            mqtt_total = mqtt_receiver.stats.total_messages if mqtt_receiver else 0
            video_packets = video_receiver.stats.packets if video_receiver else 0
            video_frames = video_receiver.stats.frames_completed if video_receiver else 0
            logging.info(
                "summary mqtt_messages=%d video_packets=%d video_frames=%d",
                mqtt_total,
                video_packets,
                video_frames,
            )

            if args.probe_seconds and time.time() - started >= args.probe_seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if mqtt_receiver is not None:
            mqtt_receiver.stop()
        if video_receiver is not None:
            video_receiver.stop()
            video_receiver.join(timeout=2.0)

    if args.probe_seconds:
        mqtt_ok = None if mqtt_receiver is None else mqtt_receiver.stats.total_messages > 0
        video_ok = None if video_receiver is None else video_receiver.stats.packets > 0
        failed = mqtt_ok is False or video_ok is False
        if failed:
            logging.error(
                "probe did not receive expected data: mqtt=%s video=%s",
                "disabled" if mqtt_ok is None else mqtt_ok,
                "disabled" if video_ok is None else video_ok,
            )
            return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

from __future__ import annotations

import json
import logging
import threading
import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Any, Callable

import paho.mqtt.client as mqtt
from google.protobuf.json_format import MessageToDict
from google.protobuf.message import DecodeError, Message

from rm_custom_client.protobuf_tools import decode_wire_fields
from rm_custom_client.proto import rm_custom_client_pb2 as pb

LOGGER = logging.getLogger(__name__)


TOPIC_TYPES: dict[str, type[Message]] = {
    "KeyboardMouseControl": pb.KeyboardMouseControl,
    "CustomControl": pb.CustomControl,
    "GameStatus": pb.GameStatus,
    "GlobalUnitStatus": pb.GlobalUnitStatus,
    "GlobalLogisticsStatus": pb.GlobalLogisticsStatus,
    "GlobalSpecialMechanism": pb.GlobalSpecialMechanism,
    "Event": pb.Event,
    "RobotInjuryStat": pb.RobotInjuryStat,
    "RobotRespawnStatus": pb.RobotRespawnStatus,
    "RobotStaticStatus": pb.RobotStaticStatus,
    "RobotDynamicStatus": pb.RobotDynamicStatus,
    "RobotModuleStatus": pb.RobotModuleStatus,
    "RobotPosition": pb.RobotPosition,
    "Buff": pb.Buff,
    "PenaltyInfo": pb.PenaltyInfo,
    "RobotPathPlanInfo": pb.RobotPathPlanInfo,
    "MapClickInfoNotify": pb.MapClickInfoNotify,
    "RadarInfoToClient": pb.RadarInfoToClient,
    "CustomByteBlock": pb.CustomByteBlock,
    "TechCoreMotionStateSync": pb.TechCoreMotionStateSync,
    "RobotPerformanceSelectionSync": pb.RobotPerformanceSelectionSync,
    "DeployModeStatusSync": pb.DeployModeStatusSync,
    "RuneStatusSync": pb.RuneStatusSync,
    "SentryStatusSync": pb.SentryStatusSync,
    "DartSelectTargetStatusSync": pb.DartSelectTargetStatusSync,
    "SentryCtrlResult": pb.SentryCtrlResult,
    "AirSupportStatusSync": pb.AirSupportStatusSync,
}

DEFAULT_TOPICS = tuple(TOPIC_TYPES)


@dataclass
class MqttStats:
    connected: bool = False
    first_message_at: float | None = None
    last_message_at: float | None = None
    total_messages: int = 0
    total_bytes: int = 0
    per_topic: Counter[str] = field(default_factory=Counter)


def parse_payload(topic: str, payload: bytes) -> dict[str, Any]:
    message_type = TOPIC_TYPES.get(topic)
    if message_type is None:
        return {"raw_wire": decode_wire_fields(payload)}

    message = message_type()
    message.ParseFromString(payload)
    return MessageToDict(
        message,
        preserving_proto_field_name=True,
        always_print_fields_with_no_presence=False,
    )


def decode_custom_byte_block_data(payload: bytes) -> bytes | None:
    message = pb.CustomByteBlock()
    try:
        message.ParseFromString(payload)
    except DecodeError:
        return None
    return bytes(message.data) if message.HasField("data") else None


class MqttReceiver:
    def __init__(
        self,
        host: str,
        port: int,
        client_id: str,
        topics: list[str],
        subscribe_all: bool = False,
        print_payloads: bool = True,
        custom_byte_callback: Callable[[bytes], None] | None = None,
    ) -> None:
        self.host = host
        self.port = port
        self.client_id = client_id
        self.topics = topics
        self.subscribe_all = subscribe_all
        self.print_payloads = print_payloads
        self.custom_byte_callback = custom_byte_callback
        self.stats = MqttStats()
        self._stop = threading.Event()

        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id,
            protocol=mqtt.MQTTv311,
        )
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

    def start(self) -> None:
        LOGGER.info("connecting MQTT %s:%s client_id=%s", self.host, self.port, self.client_id)
        self.client.connect(self.host, self.port, keepalive=30)
        self.client.loop_start()

    def stop(self) -> None:
        self._stop.set()
        self.client.loop_stop()
        self.client.disconnect()

    def wait_until_stopped(self) -> None:
        try:
            while not self._stop.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.stop()

    def _on_connect(
        self,
        client: mqtt.Client,
        _userdata: object,
        _flags: mqtt.ConnectFlags,
        reason_code: mqtt.ReasonCode,
        _properties: mqtt.Properties | None,
    ) -> None:
        if reason_code != 0:
            LOGGER.error("MQTT connect failed: %s", reason_code)
            return

        self.stats.connected = True
        LOGGER.info("MQTT connected")
        topics = ["#"] if self.subscribe_all else self.topics
        for topic in topics:
            LOGGER.info("subscribe %s", topic)
            client.subscribe(topic, qos=1)

    def _on_disconnect(
        self,
        _client: mqtt.Client,
        _userdata: object,
        _disconnect_flags: mqtt.DisconnectFlags,
        reason_code: mqtt.ReasonCode,
        _properties: mqtt.Properties | None,
    ) -> None:
        self.stats.connected = False
        LOGGER.warning("MQTT disconnected: %s", reason_code)

    def _on_message(self, _client: mqtt.Client, _userdata: object, msg: mqtt.MQTTMessage) -> None:
        now = time.time()
        topic = msg.topic
        payload = bytes(msg.payload)
        self.stats.total_messages += 1
        self.stats.total_bytes += len(payload)
        self.stats.per_topic[topic] += 1
        self.stats.first_message_at = self.stats.first_message_at or now
        self.stats.last_message_at = now

        try:
            parsed = parse_payload(topic, payload)
            status = "parsed"
        except (DecodeError, ValueError) as exc:
            parsed = {
                "decode_error": str(exc),
                "hex": payload.hex(" ", 1),
            }
            status = "raw"

        LOGGER.info(
            "mqtt %s topic=%s bytes=%d count=%d",
            status,
            topic,
            len(payload),
            self.stats.per_topic[topic],
        )
        if topic == "CustomByteBlock" and self.custom_byte_callback is not None:
            raw_data = decode_custom_byte_block_data(payload)
            if raw_data:
                self.custom_byte_callback(raw_data)
        if self.print_payloads:
            print(json.dumps({"topic": topic, "payload": parsed}, ensure_ascii=False), flush=True)

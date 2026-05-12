from __future__ import annotations

import base64
import struct
from typing import Any


def _read_varint(data: bytes, offset: int) -> tuple[int, int]:
    shift = 0
    value = 0
    while offset < len(data):
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, offset
        shift += 7
        if shift >= 64:
            break
    raise ValueError("truncated varint")


def decode_wire_fields(data: bytes) -> list[dict[str, Any]]:
    """Best-effort protobuf wire decoder for unknown or mismatched topics."""
    fields: list[dict[str, Any]] = []
    offset = 0
    while offset < len(data):
        start = offset
        key, offset = _read_varint(data, offset)
        field_no = key >> 3
        wire_type = key & 0x07
        item: dict[str, Any] = {"field": field_no, "wire_type": wire_type, "offset": start}

        if wire_type == 0:
            value, offset = _read_varint(data, offset)
            item["value"] = value
        elif wire_type == 1:
            if offset + 8 > len(data):
                raise ValueError("truncated fixed64")
            raw = data[offset : offset + 8]
            offset += 8
            item["uint64"] = int.from_bytes(raw, "little")
            item["double"] = struct.unpack("<d", raw)[0]
        elif wire_type == 2:
            length, offset = _read_varint(data, offset)
            if offset + length > len(data):
                raise ValueError("truncated length-delimited field")
            raw = data[offset : offset + length]
            offset += length
            item["len"] = length
            try:
                text = raw.decode("utf-8")
                if text.isprintable():
                    item["utf8"] = text
                else:
                    item["base64"] = base64.b64encode(raw).decode("ascii")
            except UnicodeDecodeError:
                item["base64"] = base64.b64encode(raw).decode("ascii")
        elif wire_type == 5:
            if offset + 4 > len(data):
                raise ValueError("truncated fixed32")
            raw = data[offset : offset + 4]
            offset += 4
            item["uint32"] = int.from_bytes(raw, "little")
            item["float"] = struct.unpack("<f", raw)[0]
        else:
            raise ValueError(f"unsupported wire type {wire_type}")

        fields.append(item)

    return fields


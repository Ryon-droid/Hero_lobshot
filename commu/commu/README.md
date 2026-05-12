# RoboMaster Custom Client Receiver

Python receiver for the RoboMaster 2026 custom-client link.

It does two jobs:

- MQTT `192.168.12.1:3333`: subscribe to official-client/server topics and decode Protobuf payloads.
- Video input:
  receive HEVC either from raw inner packets on UDP `3334`, or from `CustomByteBlock.data`
  carrying RoboMaster outer 309-byte frames.

## Network Checklist

The Ubuntu custom-client host should be `192.168.12.2/24`.
The official-client host should expose the custom-client link as `192.168.12.1/24`.

The official client must already show the robot video image before video bytes are expected.

## Install

```bash
uv sync
```

## Quick Probe

Use the robot ID that the official client is connected to as `--client-id`.

Examples:

```bash
uv run rm-custom-client --client-id 1 --probe-seconds 10 --no-print-payloads
uv run rm-custom-client --client-id 101 --probe-seconds 10 --no-print-payloads
```

If MQTT returns `Client identifier not valid`, the TCP service is reachable but the official endpoint rejected that client ID. Check that the official client and custom client are using the same robot ID and that custom-client mode is enabled in the official client.

## Receive MQTT Only

```bash
uv run rm-custom-client --client-id 1 --mqtt-only
```

By default it subscribes to the known server-to-custom-client topics in the protocol. To inspect everything the broker will allow:

```bash
uv run rm-custom-client --client-id 1 --mqtt-only --subscribe-all
```

Decoded Protobuf payloads are printed as JSON lines.

## Receive Video Only

Save reconstructed HEVC:

```bash
uv run rm-custom-client --video-only --video-output captures/video.hevc
```

Preview with ffplay:

```bash
uv run rm-custom-client --video-only --preview --video-output captures/video.hevc
```

The inner video header is parsed as little-endian by default:

- 2 bytes: frame number
- 2 bytes: fragment number within the frame
- 4 bytes: total bytes in the frame

If logs show invalid frame totals, try the other byte order:

```bash
uv run rm-custom-client --video-only --endian little
```

## Full Receiver

```bash
uv run rm-custom-client --client-id 1 --video-output captures/video.hevc
```

## Realtime GUI

For red infantry 3:

```bash
uv run rm-custom-client-gui --client-id 3 --video-output captures/red3_gui.hevc
```

The window is arranged as a realtime dashboard: topic counters and recent messages on the left, key match/robot cards in the center, a large video panel on the right, and full decoded JSON below the status cards. Video can arrive either as raw inner packets on UDP `3334` or inside `CustomByteBlock.data` as RoboMaster outer 309-byte frames; both paths are reconstructed and decoded for display.

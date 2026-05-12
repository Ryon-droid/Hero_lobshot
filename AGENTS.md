# Agent Notes

## Repo Overview

- **ROS 2 workspace** at repository root (`src/`).
- **Standalone Python project** in `commu/` (RoboMaster custom-client receiver). Do not mix with the ROS workspace.

## ROS 2 Workspace

- **Distro**: The environment has **Jazzy** installed (`/opt/ros/jazzy`, `$ROS_DISTRO=jazzy`). The README mentions Kilted; trust the executable environment.
- **Build**:
  ```bash
  source /opt/ros/jazzy/setup.bash
  colcon build
  source install/setup.bash
  ```
- **Launch**: `ros2 launch bringup sniper.launch.py`
- **System deps**: `rosdep install --from-paths src --ignore-src -r -y` (requires `python3-rosdep`).

### Package Roles

| Package | Type | Notes |
|---|---|---|
| `bringup` | `ament_cmake` | Contains `launch/sniper.launch.py` only. |
| `hik_camera` | `ament_cmake` | C++ composable node. **Requires Hik MVS SDK** at `/opt/MVS/include` and `/opt/MVS/lib/64`. Links `MvCameraControl`, `FormatConversion`, etc. |
| `doorlock_sniper` | `ament_cmake` | C++ composable node. Generates custom `msg/VideoPacket.msg`. Uses GStreamer + OpenCV. Linked to generated `rosidl_typesupport_cpp` target. |
| `doorlock_decoder` | `ament_python` | Python node. Depends on `doorlock_sniper` for `VideoPacket` message type. Entry point: `decoder_node`. |

### Architecture Quirks

- **Zero-copy container**: `hik_camera` and `doorlock_sniper` are loaded into the same `ComposableNodeContainer` with `use_intra_process_comms=True`. They must remain composable nodes; do not convert them to standalone `Node` actions without losing zero-copy.
- **Separate decoder**: `doorlock_decoder` runs as a standalone `Node` process (Python), subscribing to `/video_stream`.
- **Transport**: `doorlock_sniper` defaults to **serial** (`/dev/ttyACM0`, baud `961200`). To use UDP instead, build with `-DUSE_UDP=ON`.

### Debug Dumps

The launch file can save encoder/decoder window frames to `sniper_debug_imgs/` under the workspace root. Controlled by `debug_dump_*` parameters in `sniper.launch.py`.

## Python Project (`commu/`)

- **Toolchain**: `uv` (not `pip` / `conda`). `uv.lock` is checked in.
- **Setup**:
  ```bash
  cd commu
  uv sync
  ```
- **Entry points**:
  - `uv run rm-custom-client --client-id <ID> ...`
  - `uv run rm-custom-client-gui --client-id <ID> ...`
- **Network assumptions**: Host IP `192.168.12.2/24`, official client at `192.168.12.1:3333` (MQTT) and UDP `3334` (HEVC). The official client must already show robot video before UDP packets will arrive.
- **Do not** run `colcon build` inside `commu/`; it is not a ROS package.

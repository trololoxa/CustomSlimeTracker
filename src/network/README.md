# Network layer

This directory owns low-level transport primitives only. Higher-level packet formats and output policy live outside `network/`.

Current state:

- `TrackerWifiManager` owns non-blocking Wi-Fi connection/reconnect/backoff state and is host-testable through `IWifiStationAdapter`.
- `Esp32WifiStationAdapter` is the thin ESP32 wrapper around `WiFi.h`. It applies the board-specific TX power workaround after `WiFi.begin()`.
- `UdpTransport` / `Esp32UdpTransport` provide UDP socket send/receive primitives.
- `TrackerNetworkConfig` stores Wi-Fi credentials and SlimeVR server settings in a separate NVS namespace.
- Serial CLI commands under `net ...` configure, persist and inspect Wi-Fi state.

Split by responsibility:

- `network/`: Wi-Fi connection and UDP transport primitives.
- `output/`: SlimeVR packet writer and protocol constants.
- `runtime/`: SlimeVR output runtime that consumes prepared quaternion/status snapshots.
- `serial/`: CLI commands that configure and expose status.

Do not place AHRS/FIFO logic in this directory. Network code must not read sensors directly.

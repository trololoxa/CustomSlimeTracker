# Network layer

This directory contains the first real transport primitive: a non-blocking Wi-Fi station manager.

Current state:

- `TrackerWifiManager` owns connection/reconnect/backoff state and is host-testable through `IWifiStationAdapter`.
- `Esp32WifiStationAdapter` is the thin ESP32 wrapper around `WiFi.h`.
- `TrackerNetworkConfig` stores Wi-Fi credentials and future SlimeVR server settings in a separate NVS namespace.
- Serial CLI commands under `net ...` can configure, persist and inspect Wi-Fi state.

Do not add placeholder SlimeVR output here. The expected split is:

- `network/`: Wi-Fi connection and UDP transport primitives.
- `output/` or `runtime/`: packet formatting/output runtime that consumes a prepared quaternion/status snapshot.
- `serial/`: CLI commands that enable/configure the backend and expose status.

UDP and SlimeVR runtime are still not wired yet, so `output mode binary` and `output mode slimevr` must remain `NOT_IMPLEMENTED`.

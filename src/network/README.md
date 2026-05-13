# Network layer

This directory now contains the first real transport primitive: a non-blocking Wi-Fi station manager.

Do not add placeholder SlimeVR output here. The expected split is:

- `network/`: Wi-Fi connection and UDP transport primitives. `TrackerWifiManager` owns connection/reconnect state and `Esp32WifiStationAdapter` is the ESP32 Wi-Fi wrapper.
- `output/` or `runtime/`: packet formatting/output runtime that consumes a prepared quaternion/status snapshot.
- `serial/`: CLI commands that enable/configure the backend only after it exists.

UDP and SlimeVR runtime are still not wired yet, so `output mode binary` and `output mode slimevr` must remain `NOT_IMPLEMENTED`.

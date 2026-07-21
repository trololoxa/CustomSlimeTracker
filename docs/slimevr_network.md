# SlimeVR Wi-Fi / UDP runtime

This document is the operational reference for the ESP32-C3 SlimeVR network backend. It covers the pieces that are easy to forget while debugging: Wi-Fi setup, autostart, SlimeVR UDP packets, status commands, and the diagnostics that should be used before thermal/power optimization.

## Runtime architecture

The network stack is deliberately split into layers:

```text
TrackerWifiManager          non-blocking station state machine
Esp32WifiStationAdapter     ESP32 Arduino WiFi wrapper
IUdpTransport               host-testable UDP abstraction
Esp32UdpTransport           WiFiUDP wrapper
SlimeVRPacketWriter         host-safe packet encoder/parser helpers
SlimeVROutputRuntime        discovery, server session, telemetry, RotationData
PreparedOutputRuntime       producer of coherent orientation/motion snapshot
```

`SlimeVROutputRuntime` never reads FIFO, AHRS or IMU objects directly. It only copies `TrackerPreparedOutputSnapshot`, which is produced by the tracking path from one accepted sample. The snapshot owns the quaternion and gravity-removed device-frame acceleration under one timestamp. World-frame acceleration is derived from that quaternion when needed rather than duplicated in RAM. This keeps Wi-Fi/UDP scheduling separate from sensor fusion and prevents packet 4 from recomputing motion against a different quaternion.

Local serial output is also separate from SlimeVR UDP. `stream ...` and `output ...` are developer serial outputs; `slime ...` controls the server transport.

## Wi-Fi setup

Typical first setup:

```text
net set ssid <2.4GHz SSID> save
net set pass <password> save
net set name TheFirst save
net enable save
reboot
```

After reboot:

```text
net status
slime status
```

Expected Wi-Fi fields:

```text
wifi_state=connected
connected=yes
ip=192.168.x.x
rssi_dbm=-30..-85
```

ESP32-C3 only uses 2.4 GHz Wi-Fi. If the SSID is not visible in `net scan`, check that the AP is not 5 GHz-only and that band steering is not hiding the 2.4 GHz radio.

## TX power workaround

Some ESP32-C3 board revisions become unstable at maximum Wi-Fi TX power because the antenna is too close to the crystal/board RF layout. This firmware applies a lower TX power immediately after `WiFi.begin()`:

```c
#define TRACKER_WIFI_APPLY_TX_POWER_AFTER_BEGIN 1
#define TRACKER_WIFI_TX_POWER WIFI_POWER_8_5dBm
```

These are in `src/defines.h` so another board profile can override them with build flags or a local edit. If a different board works at default power, set `TRACKER_WIFI_APPLY_TX_POWER_AFTER_BEGIN=0` or choose another `TRACKER_WIFI_TX_POWER` value.

## SlimeVR autostart

SlimeVR UDP autostarts when network config says Wi-Fi is enabled and credentials are valid:

```text
wifiEnabled=true
credentialsValid=true
```

This means a normal production boot path is:

```text
net enable save
reboot
```

No serial `output mode slimevr` exists anymore. `slime start` is a runtime command, and SlimeVR output is independent from local serial streams.

## SlimeVR protocol state

Current firmware metadata:

```text
protocol_version=19
board_type=10       LOLIN_C3_MINI
imu_type=13         LSM6DSV
mcu_type=6          ESP32_C3
```

Outgoing packets currently used:

```text
Handshake / discovery
SensorInfo
RotationData
HeartBeat
PingPong response
AcknowledgeConfigChange
BatteryLevel when enabled
Tap when enabled
Error when tracker health reports one
SignalStrength
Temperature
```

Incoming packets currently handled:

```text
Discovery response: raw 0x03 + "Hey OVR =D 5"
Heartbeat packet 0/1
PingPong packet 10
FeatureFlags packet 22, stored for diagnostics but not acted on
SetConfigFlag packet 25, used for runtime magnetometer/yaw toggle
ProtocolChange packet 200, counted but not acted on
```

Magnetometer capability is advertised via `SensorInfo.sensor_config`:

```text
0x0 = mag not supported
0x2 = mag supported, disabled
0x3 = mag supported, enabled
```

The firmware intentionally does not send periodic dummy `MagnetometerAccuracy` packets. Packet 18 should only be emitted if a real mag-calibration/accuracy workflow starts using it.

`SignalStrength` packet 19 carries one signed RSSI value in dBm. For example,
`-68 dBm` is encoded as the two's-complement byte `0xBC`; it is not normalized
to a user-facing 0..100 percentage. `slime status` exposes the value as
`last_signal_strength_dbm`.

## Known protocol gaps in the current baseline

The UDP runtime now provides coherent orientation plus linear acceleration, but
it does not yet implement the complete modern tracker/server contract:

- acceleration packet 4 is emitted immediately after a successful packet 17
  from the same snapshot. It carries gravity-removed device-frame acceleration
  in SI `m/s^2`; invalid acceleration suppresses packet 4 without suppressing rotation;
- the short SensorInfo acknowledgement packet 15 is not tracked as a confirmed
  state, and SensorInfo is refreshed periodically or on local changes;
- firmware FeatureFlags packet 22 is not sent, so optional packet bundling is
  not negotiated;
- received server FeatureFlags are stored only for diagnostics;
- ProtocolChange is counted but does not switch protocol;
- normal control packets are not yet hardened to the established server
  endpoint before they refresh session activity or change runtime flags.

These are planned protocol upgrades. Documentation must not describe them as
already active.

## CLI diagnostics

Compact status:

```text
slime status
```

Use it for normal checks. It shows server state, rotation counter, send failures, ping/pong, unknown packet count, mag flags, signed RSSI dBm and latest IMU temperature.

Full developer dump:

```text
slime debug
```

Use it when packet counters/timestamps matter. It includes handshake counts, SensorInfo counts, duplicate snapshot counters, last packet ids, reconnect-hardening counters, and last rotation metadata.

Counter reset:

```text
slime counters reset
```

Manual reconnect:

```text
slime reconnect
```

Manual rotation rate change:

```text
slime rate 100
```

`100 Hz` is the default target because it gives smooth SlimeVR preview/tracking
and the optimized ESP32-C3 runtime targets it with an 18-word FIFO watermark,
an 8 MHz SPI default and a RAM-backed work queue. It is not a protocol requirement. For thermal experiments or battery-focused
builds, `slime rate 50` is a valid lower-load setting, but verify the result
with `test runtime <seconds>` and the SlimeVR preview before changing defaults.

The FIFO watermark affects transaction overhead and latency. Production/Debug
default to 18 words; Slim remains at 9. Existing NVS values are preserved, so an
already-calibrated tracker using the previous 4 MHz / 12-word settings must opt in
with `config spi 8000000 save` and `fifo watermark 18 save`. If 8 MHz fails during
LSM initialization, startup retries at 4 MHz.

## Reconnect behavior

The runtime handles these cases:

```text
Wi-Fi disconnected       -> stop UDP, wait for Wi-Fi, rediscover server
server silence timeout   -> forget server endpoint, return to discovery
consecutive send errors  -> reopen UDP socket and rediscover
server restart           -> discovery response should restore server_found
```

Relevant debug counters:

```text
server_silence_resets
wifi_lost_resets
udp_reopen_requests
consecutive_send_failures
send_failures
udp_begin_failures
unknown_packets_received
```

A healthy steady state should have:

```text
server_found=yes
send_failures=0
unknown_packets_received=0
ping_received and pong_sent both increasing
rotation_sent increasing
```

## Runtime test for Wi-Fi + server load

`test static` is still useful for IMU/FIFO stationary analysis, but it is not a full network-load test. Use:

```text
test runtime 600
```

It measures the whole firmware loop while Wi-Fi/SlimeVR is active:

```text
loop timing
CLI timing
FIFO section timing
network section timing
boot heartbeat/status section timing
FIFO/perf deltas
quality deltas
Wi-Fi disconnect/connect-timeout deltas
SlimeVR rotation/send/ping/unknown packet deltas
temperature start/end/delta
```

Recommended optimization baseline:

```text
slime counters reset
test runtime 600
slime debug
health
fifo stats
quality stats
```

Acceptance targets before thermal optimization:

```text
wifi_connected_end=yes
slime_server_found_end=yes
slime_send_failures_delta=0
slime_unknown_packets_delta=0
wifi_disconnects_delta=0
tracking_recovery_delta=0
quality_estimated_dropped_delta=0
fifo_overrun_delta=0
fifo_full_delta=0
```

## Thermal optimization notes

Do not optimize blindly. First collect a runtime baseline with `test runtime`. Likely knobs for later testing:

```text
TRACKER_WIFI_APPLY_TX_POWER_AFTER_BEGIN / TRACKER_WIFI_TX_POWER
TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS
TRACKER_SLIMEVR_ENABLE_SIGNAL_TELEMETRY
TRACKER_SLIMEVR_ENABLE_TEMPERATURE_TELEMETRY
slime rate 50 vs 100
TRACKER_ENABLE_BOOT_HEARTBEAT
TRACKER_ENABLE_MACHINE_LOG
TRACKER_ENABLE_SERIAL_STREAM
```

`WiFi.setSleep(false)` is currently used for stable latency. Enabling modem sleep may reduce heat, but it must be tested against FIFO stability, UDP packet loss, ping, and SlimeVR preview smoothness.

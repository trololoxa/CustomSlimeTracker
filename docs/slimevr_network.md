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

`SlimeVROutputRuntime` never reads FIFO, AHRS or IMU objects directly. It only copies `TrackerPreparedOutputSnapshot`, which is produced by the tracking path from one accepted sample. The snapshot owns the quaternion and gravity-removed device-frame acceleration under one timestamp. World-frame acceleration is derived from that quaternion when needed rather than duplicated in RAM. This keeps Wi-Fi/UDP scheduling separate from sensor fusion and prevents the network encoder from recomputing motion against a different quaternion.

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
protocol_version=22
board_type=10       LOLIN_C3_MINI
imu_type=13         LSM6DSV
mcu_type=6          ESP32_C3
firmware_version    c3-6dsv-safe-calibration-autonomy-hardened
```

The feature version names the currently completed firmware capability and is
also used in the SlimeVR handshake. Git identity is reported separately by
`version`, `status`, test reports and machine-log metadata. Clean builds use
the short Git HEAD; uncommitted test builds use `HEAD+worktree-dirty`.

Outgoing packets currently used:

```text
Handshake / discovery
SensorInfo
FeatureFlags packet 22
Bundle packet 100 containing packet 17 then packet 4, when negotiated
RotationData packet 17
Acceleration packet 4 fallback
HeartBeat
PingPong response
AcknowledgeConfigChange
UserAction packet 21 on explicit command or configured physical tap
BatteryLevel when enabled
Tap when enabled
Error when tracker health reports one
SignalStrength
Temperature
```

Packet 23 remains compiled as an explicitly experimental encoder but is disabled
by default. It is not selected merely because the server reports bundle support.

Incoming packets currently handled:

```text
Discovery response: raw 0x03 + "Hey OVR =D 5"
SensorInfo acknowledgement: special six-byte packet 15 without packetNumber
Heartbeat packet 0/1
PingPong packet 10
FeatureFlags packet 22, used for capability negotiation
SetConfigFlag packet 25, transactionally persisted before acknowledgement
ProtocolChange packet 200, validated, counted and intentionally ignored
```

Magnetometer capability is advertised via `SensorInfo.sensor_config`:

```text
0x0 = mag not supported
0x2 = mag supported, disabled
0x3 = mag supported, enabled
```

The firmware intentionally does not send periodic dummy `MagnetometerAccuracy`
packets. Packet 18 should only be emitted if a real mag-calibration/accuracy
workflow starts using it.

### Negotiated motion transport

After discovery, firmware sends packet 22 with its firmware FeatureFlags. Server
FeatureFlags bit 0 means `PROTOCOL_BUNDLE_SUPPORT`. Only after receiving that bit
does runtime select:

```text
outer packet 100
  inner packet 17: float32 quaternion
  inner packet 4:  float32 SI linear acceleration
```

The inner order is rotation first and acceleration second. Both values come from
one prepared snapshot, packet 100 uses one outer packet number and one UDP
datagram, and no Q15/Q7 quantization is introduced. Battery, temperature, RSSI,
heartbeat, SensorInfo, ping/pong, tap, errors and config acknowledgements remain
separate service/control packets.

A server that does not answer FeatureFlags, or answers without bundle bit 0,
uses the compatibility path:

```text
packet 17 at configured pose rate
packet 4 at TRACKER_SLIMEVR_FALLBACK_ACCEL_RATE_HZ (default 50 Hz)
```

Step mounting receives real callback timestamps and does not require a fixed
100 Hz acceleration cadence. The fallback preserves coherent samples while
reducing pose datagram pressure from 200 to 150 datagrams per second at a 100 Hz
rotation rate.

`TRACKER_SLIMEVR_USE_COMPACT_MOTION_PACKET=1` is an explicit experimental
override for packet 23. The normal build keeps it at zero because packet 23 has
no legacy capability bit that proves a specific beta server parser actually
accepts it. Diagnostics therefore show both availability and use:

```text
packet23_available=yes
packet23_enabled=no
motion_packet_mode=bundle_100_rotation_17_accel_4
```

or, on an older server:

```text
motion_packet_mode=rotation_17_plus_accel_4_fallback
fallback_acceleration_rate_hz=50
```

`send_failures` counts failed physical datagrams. A failed packet-100 motion
bundle increments `bundled_motion_send_failures` and both logical
`rotation_send_failures` and `acceleration_send_failures`. A failed experimental
packet 23 uses `compact_motion_send_failures` instead.

UDP TX pressure is handled independently from inbound session liveness. An
ESP-IDF/lwIP `ENOMEM`, `ENOBUFS` or `EAGAIN` result enters a bounded 20–160 ms
backoff so stale pose packets are discarded instead of repeatedly blocking the
FIFO loop. Four consecutive physical failures, or eight failures in the exact
last 32 physical attempts, request recovery. When heartbeat/ping proves that the
server association is still alive, the runtime rebinds only the local UDP socket
on the same port and preserves the negotiated session. A second burst within two
seconds escalates to the normal full discovery lifecycle. Inspect
`tx_backoff_drops`, `tx_pressure_failures`, `tx_failure_window_trips`,
`udp_transport_rebind_successes`, `udp_full_reopen_escalations` and
`last_udp_send_error`.

Manual server mode is functional rather than storage-only. The runtime resolves
`serverHost` once per bounded discovery interval (dotted IPv4 locally, DNS on
ESP32), sends an ordinary unicast handshake, and repeats only at the configured
interval. When broadcast discovery is disabled, only the resolved manual
IP/port may establish the session. When discovery remains enabled, manual
unicast and broadcast handshakes coexist and the first valid response wins.
Server silence, Wi-Fi loss, socket recreation or config changes clear the
resolved endpoint and re-enter handshake/rebind flow.

After session establishment, only the selected server IP/port may send
FeatureFlags, ping, heartbeat, config or protocol-control packets. Foreign
endpoints cannot replace the live server, negotiate bundle mode or refresh the
silence timer. Empty FeatureFlags, truncated/oversized UDP datagrams and
headerless normal control packets are counted as malformed and never refresh
liveness. Inspect `foreign_endpoint_packets_dropped`,
`pre_session_packets_dropped`, `malformed_datagram_length`, the other
`malformed_*` counters, and `manual_server_*` fields in `slime debug`.

`SignalStrength` packet 19 carries one signed RSSI value in dBm. For example,
`-68 dBm` is encoded as the two's-complement byte `0xBC`; it is not normalized
to a user-facing 0..100 percentage. `slime status` exposes the value as
`last_signal_strength_dbm`.

## Session-completeness contract

The UDP runtime now implements the complete session contract used by this
firmware baseline:

- protocol version 22 advertises corrected tracker acceleration. Rotation and
  acceleration use the right-handed device basis `+X right, +Y forward, +Z
  top/outward`; the server therefore does not apply its historical extra -90
  degree local-Z acceleration correction;
- valid motion uses a negotiated float32 packet-100 bundle when available;
  otherwise packet 17 stays at pose rate and coherent packet 4 is limited to
  50 Hz. Hard-invalid acceleration never suppresses valid rotation;
- packet 23 remains an explicitly disabled experimental mode because no legacy
  server FeatureFlag proves parser compatibility;
- SensorInfo has explicit `dirty`, `waiting_for_ack` and `acknowledged` states.
  The special six-byte acknowledgement is parsed separately from normal
  headered packets, and resend stops only after the acknowledged state matches
  the latest local state;
- firmware/server FeatureFlags use explicit `not_started`, `waiting`,
  `negotiated` and `unavailable` states. Empty or malformed bitsets do not
  establish capabilities;
- SetConfigFlag type 1 applies the magnetometer/yaw state transactionally,
  persists it, verifies the result and only then sends packet 24. An idempotent
  retry is acknowledged without repeating the NVS write;
- `slime action yaw|full|mounting|pause` sends UserAction packet 21. Physical
  tap mapping is stored in network config and defaults to `off`;
- ProtocolChange is length-validated and recorded, but protocol switching is
  intentionally unsupported; the runtime remains on protocol 22.

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

Use it when packet counters/timestamps matter. It includes handshake counts, SensorInfo acknowledgement state, FeatureFlags attempts, malformed/foreign packet counters, SetConfigFlag apply/ACK results, UserAction counters, duplicate snapshot counters, reconnect-hardening counters and last rotation metadata.

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

Manual server actions:

```text
slime action yaw
slime action full
slime action mounting
slime action pause
```

Optional physical-tap mapping is persistent only with `save` and defaults to
`off`:

```text
slime tap-action off|yaw|full|mounting|pause [save]
```

The scheduler keeps an absolute phase. If a loop reaches a deadline late, it
advances directly to the nearest future deadline, sends at most one latest
snapshot and records any skipped periods. It does not set the next phase from
the late call and does not send stale catch-up bursts. Relevant diagnostics are:

```text
rotation_missed_deadlines
rotation_late_events
rotation_lateness_sum_ms
rotation_lateness_max_ms
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

## Magnetic reliability independence

Patch 0022 changes local magnetic trust, yaw re-entry and calibration-candidate
diagnostics only. It does not change SlimeVR protocol version, packet formats,
FeatureFlags, bundle negotiation, output rate or session behavior. During a
magnetic disturbance rotation remains available from the 6DoF AHRS; only local
yaw correction is withheld.

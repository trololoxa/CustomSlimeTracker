# 0023gl SlimeVR UDP TX recovery hardening

## Scope

This suffix hotfix is based on `0023gk_magnetometer_robust_fit_acceptance_hardening` and changes only the SlimeVR UDP transport/recovery path, related diagnostics, tests, and documentation. It does not change IMU ODR, FIFO configuration, AHRS, calibration math, tracking packet formats, protocol version, persistent config schema, or candidate format.

## Hardware evidence

Two similar ESP32-C3 trackers entered a long-lived low-TPS state that was cleared by a physical MCU restart.

Tracker 1 had remained connected for about 3,764 seconds with RSSI around -24 dBm and `server_found=yes`, yet accumulated roughly 103,000 UDP send failures. The same log showed 788 FIFO overruns, 870 FIFO-full events, a 24.2 ms average loop, a 21.7 ms average FIFO section, and network calls reaching about 20 ms. Rotation delivery fell to 32-63 Hz in firmware while the server UI had shown about 10 TPS.

Tracker 2 also had a strong link (-26 to -32 dBm), no Wi-Fi disconnect, and about 43,000 UDP send failures. Its old recovery logic incremented `udp_reopen_suppressed_recent_rx` 253 times because heartbeat/ping reception was treated as proof that the complete UDP socket was healthy. FIFO damage was smaller (18 overruns and 20 full events), and the tracker happened to recover around the time `perf on` was entered.

`perf on` only enables and resets the runtime profiler. It does not restart Wi-Fi, reopen UDP, clear the SlimeVR session, or alter FIFO/AHRS scheduling. The observed recovery therefore was not a defined command effect.

## Root cause

ESP-IDF documents that repeated UDP `send()`/`sendto()` calls can fail with `ENOMEM` when lower-layer Wi-Fi TX buffers are full, and recommends checking the error and retrying after a short delay. The existing firmware returned only a boolean from `WiFiUDP::endPacket()`, immediately retried pose output on the next network opportunity, and recovered only after a long consecutive-failure threshold.

Two defects made the failure self-sustaining:

1. Recent inbound heartbeat/ping suppressed socket recovery. RX liveness proves that the server association and receive path are alive, but it does not prove that the local TX path can allocate/send buffers.
2. Intermittent failure populations, such as the approximately 25-30% failure rate in tracker 1, continually reset the consecutive counter and could run indefinitely.

Repeated failed/slow UDP calls consumed the realtime loop budget. FIFO draining then fell behind, the bounded runtime raw queue approached capacity, hardware FIFO overrun/full events accumulated, and recovery work further increased loop pressure.

## Implementation

### errno-aware transport

`IUdpTransport` now exposes the errno-style reason from the most recent failed physical send. `Esp32UdpTransport` captures errors from both `beginPacket()` and `endPacket()` and reports invalid/partial sends explicitly.

### bounded TX backoff

`ENOMEM`, `ENOBUFS`, and `EAGAIN` are classified as TX-pressure failures. After a failed physical send, stale pose/acceleration/telemetry/tap traffic is suppressed for an exponentially bounded 20, 40, 80, then 160 ms interval. Suppressed attempts do not call the UDP stack and do not inflate physical send-failure counters.

Infrequent control, discovery, user-action, and error-report packets are not silently discarded solely because pose traffic is in backoff.

### dual recovery detector

Recovery is requested by either:

- four consecutive physical send failures; or
- at least eight failures in the exact last 32 physical send attempts.

The second detector uses a fixed 32-bit outcome window and catches persistent intermittent failure without heap allocation or unbounded history.

### session-preserving UDP rebind

When a recent valid server packet proves that the session association is alive, TX recovery closes and reopens only the local UDP socket on the same configured port. It preserves:

- selected server endpoint;
- `serverFound` state;
- SensorInfo acknowledgement state;
- server FeatureFlags and negotiated bundle mode;
- packet sequence and tracking configuration.

A 20 ms post-rebind send grace avoids immediately refilling a just-recreated TX path.

### bounded escalation

A second failure burst within two seconds of a successful local rebind escalates to the existing full SlimeVR discovery/session restart. A failed rebind also escalates immediately. If inbound server activity is already stale, the first burst goes directly to full discovery. This prevents an infinite local-rebind loop.

## Diagnostics

The following counters replace the misleading `udp_reopen_suppressed_recent_rx` field:

- `udp_transport_rebind_requests`
- `udp_transport_rebind_successes`
- `udp_transport_rebind_failures`
- `udp_full_reopen_escalations`
- `tx_backoff_drops`
- `tx_pressure_failures`
- `tx_other_failures`
- `tx_failure_window_trips`
- `last_udp_send_error`

They are exposed through SlimeVR status/debug, runtime tests, `perf tracking`, perf correlation, and motion diagnostics.

## Regression coverage

The native SlimeVR runtime test verifies:

- one `ENOMEM` failure enters backoff and the next stale pose does not call `send()`;
- four failures with recent RX perform a local rebind and preserve the negotiated packet-100 session;
- eight failures in the exact last-32-attempt window catch intermittent pressure;
- a second burst shortly after rebind performs a full session restart;
- stale server RX performs a full restart directly;
- a failed local rebind escalates fail-closed;
- existing SensorInfo refresh/control traffic is not accidentally discarded by pose backoff.

The policy also confirms that `perf on` has no Wi-Fi/UDP/SlimeVR side effect.

## Hardware acceptance

After applying the patch, run:

```text
python3 tools/check_all.py --clean --require-pio
```

Then on each tracker:

```text
perf on
perf tracking reset
```

Leave the trackers running with the normal full set for at least the previous failure duration. Inspect:

```text
perf tracking
perf status
slime status
fifo stats
```

Expected behavior during a transient pressure event:

- `tx_pressure_failures` may increment;
- `tx_backoff_drops` may increment because stale poses are intentionally discarded;
- `udp_transport_rebind_successes` may increment once;
- `udp_full_reopen_escalations` should normally remain zero;
- `fifo_overrun_delta`, `fifo_full_delta`, and tracking recovery deltas should remain zero or stop cascading;
- rotation delivery should recover automatically without a physical ESP restart.

Repeated full reopen escalation, persistent `last_udp_send_error`, or continued FIFO cascades remain hardware blockers and should be captured with the same four status commands.

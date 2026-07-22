# Wi-Fi remote console

The firmware can expose the normal CLI over a small TCP server for cable-free
setup and calibration. This is not a separate command set: the remote console
feeds the same `TrackerSerialCommandInterface` dispatcher that USB Serial uses.

Defaults:

- Debug: enabled.
- Production: enabled.
- Slim: disabled and excluded from the source filter.
- Port: `TRACKER_REMOTE_CONSOLE_PORT`, default `7777`.
- Per-loop input budget: `TRACKER_REMOTE_CONSOLE_BYTES_PER_LOOP`, default `32`
  in Debug and `16` in Production.
- Bounded output queue: `TRACKER_REMOTE_CONSOLE_OUTPUT_QUEUE_BYTES`, default
  `8192` bytes in Debug and Production/ProductionDiag. Telnet is the primary
  long-report channel, so this is deliberately larger than the USB queue.
- Atomic record staging: `TRACKER_REMOTE_CONSOLE_OUTPUT_RECORD_BYTES`, default
  `768` bytes. One overlong line is dropped as a whole.
- Per-drain output budget: `TRACKER_REMOTE_CONSOLE_OUTPUT_BYTES_PER_DRAIN`,
  default `128` bytes in Debug and `64` bytes in Production.
- Output drain cadence: `TRACKER_REMOTE_CONSOLE_OUTPUT_DRAIN_INTERVAL_MS`,
  default `4 ms` in Debug and `5 ms` in Production.

Connect from a PC on the same Wi-Fi network:

```bash
nc <tracker-ip> 7777
```

or:

```bash
telnet <tracker-ip> 7777
```

Typical wireless calibration flow:

```text
setup status
setup calibration
remote off
```

## Runtime control

The remote console is intentionally runtime-disableable. After calibration, run:

```text
remote off
```

This closes the current TCP client and stops the TCP server for the current
boot. After that the runtime no longer accepts/polls TCP console clients, so it
does not add normal-loop or Wi-Fi server work. It does not change NVS. USB Serial
can re-enable it for the current boot with:

```text
remote on
```

Status:

```text
remote status
```

prints whether the feature is compiled, enabled, listening, connected, the port,
client/input counters and bounded-output queue counters. The shared command:

```text
console status
```

prints both USB Serial and remote-console output queues together with the active
drain byte budgets and drain intervals. `console reset` discards stale queued
or partially staged console text and resets all output counters.

## Output backpressure

The TCP client is no longer written directly by command handlers or event logs.
Output is first copied into a fixed-size ring and drained only after both the
FIFO/AHRS path and the SlimeVR UDP update. The drain is limited by both a byte
budget and a time cadence, so a connected telnet client cannot consume every
high-rate app loop or take priority over RotationData. The TCP drain uses a
non-blocking socket send rather than `WiFiClient::write()`, because the pinned
Arduino-ESP32 2.0.x client does not expose a useful `availableForWrite()` value.
A slow or disconnected client therefore cannot block IMU processing or reduce
the planned UDP service rate.

The queue is deliberately bounded, but admission is line-atomic. Output is
staged until newline and then either committed as one complete record or dropped
as one complete record. Congestion can no longer produce glued fragments such as
`wifi_disconnmotion status`. After room becomes available the stream emits one
compact `# WARN console dropped N complete line(s)` marker.

High-rate machine logs remain best-effort. When the queue lacks reserve for a
record, the producer skips formatting and increments `LOGSTAT,BACKPRESSURE`. A
capture with queue drops or machine-log backpressure drops is incomplete and
should not be used as a replay fixture. Normal tracking keeps priority over
preserving every diagnostic line.

`remote_console_bytes_dropped` counts input/client-disconnect loss, while the
`remote_console_output_*` fields describe the bounded output queue itself.

## Notes

- Only one TCP client is accepted at a time. Extra clients receive a busy message
  and are closed.
- The console is intentionally unauthenticated. Use it only on a trusted private
  network or compile it out with `-DTRACKER_ENABLE_WIFI_REMOTE_CONSOLE=0`.
- Blocking setup/calibration commands read from the same active TCP stream. The
  app does not poll the remote-console parser from the blocking calibration
  service hook, so prompt input is not stolen recursively.
- `setup calibration` keeps an already connected Wi-Fi link during the
  temperature stage. It does not issue a forced `net reconnect` when Wi-Fi is
  already connected, because that would drop the remote TCP console session.


## RAM tuning

The console buffers are independent fixed arrays; they never share the LSM6DSV
FIFO or runtime IMU sample queues. Current defaults reserve approximately:

```text
USB Serial queue + record staging: 1536 + 512 bytes in Production
telnet queue + record staging:     8192 + 768 bytes
```

The larger telnet queue is intentional because `perf status`, `motion status`
and `slime debug` are multi-kilobyte bursts. If a future subsystem needs RAM,
reduce `TRACKER_REMOTE_CONSOLE_OUTPUT_QUEUE_BYTES` and/or the USB queue first; do
not raise the sustained drain rate, because that can compete with SlimeVR UDP.

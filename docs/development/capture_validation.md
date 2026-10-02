# Capture Validation

## Strict cable-free static capture

Build and flash a clean `BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG` image, power the
tracker from battery and disconnect USB. Then run:

```bash
python3 tools/capture_telnet_log.py \
  --host <tracker-ip> \
  --seconds 600 \
  --rate 20 \
  --mode full \
  --output logver3_static_clean_001.log
```

The tracker remains stationary for the complete golden candidate. The host tool
requires a known full base commit plus a valid worktree fingerprint; clean and
dirty ProductionDiag builds are accepted and recorded in the manifest. It validates magnetometer runtime,
calibration/alignment and current trust, resets log/console counters, owns one
TCP session, drains the deferred pipeline and writes both the capture and a
SHA-256 manifest using atomic file replacement. The requested capture path is
promoted only after strict validation succeeds. Any disconnect, malformed frame, sequence/timestamp
fault, FIFO recovery, drop or out-of-range Q rate fails the candidate. Failed
runs are retained only as uniquely named partial diagnostics.

Validate an already captured candidate again with:

```bash
python3 tools/replay/strict_logver3_gate.py \
  --log logver3_static_clean_001.log \
  --output logver3_static_clean_001.validation.json
```

Do not create the release golden JSON until the real capture has passed and its
thresholds have been reviewed independently. A synthetic positive log would
only test the parser and is not hardware evidence.

## Magnetometer replay capture smoke sequence

Use this when creating a fixture for magnetometer calibration, axis mapping, or
magnetic disturbance work:

```text
stream off
output stop
mag enable save
mag heading auto off
mag yaw disable save
log reset
log full
log rate 20
log header
mag cal reset
mag cal start
# rotate slowly through all orientations for 60-120 seconds
mag cal stop
mag cal status
log summary
log off
```

Validate the captured file on the host:

```bash
python tools/replay/replay_machine_log.py logs/mag_sweep_001.log --require-magr --min-magr-rows 500 --pretty
```

Do not mix `stream raw/scaled/quat/debug` with replay capture; the replay parser
ignores human output, but cleaner serial captures are easier to inspect and
archive.

## Full runtime / Wi-Fi / SlimeVR load test

`test static` is an IMU/FIFO stationary test. It is not enough for Wi-Fi/server optimization because it was designed around sensor stability, not around the complete firmware loop. For full network load use:

```text
test runtime <seconds>
```

Recommended baseline before optimizing heat or Wi-Fi power:

```text
net status
slime status
slime counters reset
test runtime 600
slime debug
health
fifo stats
quality stats
```

After the compact `RUNTIME TEST DONE` marker, request the retained detailed
report over USB with `test report runtime`. The report contains:

- loop/CLI/FIFO/network/heartbeat section timing;
- max and average loop costs;
- slow loop/network/FIFO counters;
- IMU runtime sample rate;
- FIFO/perf/quality deltas;
- Wi-Fi disconnect/connect-timeout deltas;
- SlimeVR rotation, ping/pong, unknown packet and send failure deltas;
- start/end temperature.

Healthy Wi-Fi + SlimeVR run targets:

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

For SlimeVR output, also check the effective rotation rate:

```text
slime_rotation_rate_hz_observed ~= slime rate
```

New/default Production/Debug configs use an 18-word FIFO watermark and 8 MHz SPI, with a
4 MHz startup fallback. Hardware drains feed a 512-sample raw RAM ring in
Production/ProductionDiag (256 in Slim) and a 64-sample mag ring. The consumer
is work-conserving: each slice guarantees raw progress, favors raw IMU samples
over mag callbacks, services network between slices, and receives a larger app
budget when the raw queue reaches its urgent high-water threshold. Hardware SPI
time is excluded from the callback budget.
Long runtime/SlimeVR diagnostic reports cooperatively service FIFO and network
between output sections. Healthy hardware tests must keep both hardware FIFO and
RAM-ring overflow counters at zero.

For a 100 Hz hardware cadence check on ProductionDiag, prefer the compact,
non-destructive window:

```text
perf tracking reset
# move the tracker continuously for 5-10 minutes
perf tracking
```

Healthy results normally have zero `fifo_overrun_delta`, `fifo_full_delta`,
`quality_dropped_delta`, `quality_recovery_delta`, and RAM-ring overflow deltas.
If a rare bounded FIFO full/overrun does occur, `tracking_soft_recovery_enter_delta`
should increase together with `tracking_soft_recovery_complete_delta`, while
`tracking_recovery_enter_delta` and a long `rotation_no_snapshot_delta` remain
zero. Timestamp-corruption and explicit-reset cases must still use strict recovery.
`rotation_delivery_pct` should be near 100%, `rotation_sent_rate_hz` should be near
the configured rate during movement, and `acceleration_sent_delta` should remain
close to `rotation_sent_delta`. The baseline command does not reset quality,
timestamp reconstruction or recovery state, so the measurement cannot hide an
existing fault. Use full `health`/`slime debug` only after the window when deeper
context is needed. A server GUI may visually report a low idle TPS for nearly
identical quaternions; firmware counter deltas are the source of truth for packet
cadence.

For an existing calibrated tracker, test 8 MHz and an 18-word watermark with:

```text
config spi 8000000 save
fifo watermark 18 save
```

Apply the watermark while stationary. Live hardware reconfiguration clears old
software-queued samples and intentionally requests controlled recovery when a valid
orientation already exists; wait for that short recovery before starting the timed
window. During boot, reconfiguration before the first quaternion must instead report
a startup-bootstrap bypass and allow normal AHRS gravity initialization. Failed
hardware apply or NVS save must restore the previous config. `perf tracking` reports
classified recovery causes so reconfiguration can be distinguished from
FIFO/timestamp faults.

Use `test stop` to finish early. `test status` prints both static and runtime
status. Completion never formats the large report in the loop/sample hot path;
`test report static|runtime` prints the immutable retained snapshot later over USB or TCP.

### RC1 serial provisioning compatibility


Battery telemetry ownership, filtering and failure behavior are specified in the [battery contract](../architecture/battery.md). Use `battery status` to inspect the local reading during validation.

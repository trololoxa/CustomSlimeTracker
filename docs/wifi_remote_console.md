# Wi-Fi remote CLI console

The TCP console is a cable-free transport for the same profile-specific CLI
that is available over USB. It feeds the normal fixed-buffer command parser and
the same dispatcher without an origin-specific allowlist or privilege split.

Profile contract:

- Debug: listener enabled for development;
- Production: listener and source unit compiled out;
- Production Diagnostic: listener enabled for capture/diagnostics;
- Slim: listener and CLI compiled out;
- port: `TRACKER_REMOTE_CONSOLE_PORT`, default `7777`;
- one active client; additional clients receive a bounded busy response;
- fixed output queue `8192` bytes and atomic record staging `768` bytes in
  Debug/ProductionDiag;
- non-blocking socket drain, `128` bytes per admitted drain at a `4 ms` cadence.

Connect from a trusted private network:

```bash
nc <tracker-ip> 7777
```

or:

```bash
telnet <tracker-ip> 7777
```

Telnet IAC negotiation, subnegotiation and CR-NUL bytes are filtered before the
ASCII command buffer. Raw TCP clients continue to use ordinary LF or CRLF.

## Security and command policy

The listener is unauthenticated. Anyone who can reach the port can run every
command compiled into that image, including setup, NVS/config/calibration and
network mutation, reset, reboot, stream/output control and detailed reports.
Use Debug/ProductionDiag only on a trusted isolated network; Production and Slim
still compile the listener out.

USB and TCP share the same command bounds:

```text
log rate 1..200 Hz
test static|runtime 1..21600 seconds
```

`help` prints the same profile-specific command surface on both transports and
includes the current origin in the banner. `remote off`/`remote on` are also
available over either transport; issuing `remote off` through TCP intentionally
closes the connection that carried the command.

## Session ownership

Every accepted client receives a monotonically changing non-zero session ID.
Machine log and static/runtime tests bind to the initiating `Stream`, origin and
session for output routing and disconnect cleanup. Any connected CLI transport
may stop, reconfigure or rebind an active diagnostic; there is no USB-only
administrative override.

On disconnect, the close hook runs before the stream is detached. It aborts
owned log/tests, releases their stream pointers and records disconnect/shutdown
drops. There is no fallback to USB and no stale pointer to a detached TCP
stream.

An active session also blocks motion light sleep, including capture preflight.
The capture tool sends a Telnet IAC NOP every five seconds; any consumed input
renews the firmware's 30-second application lease. If the host disappears
without FIN/RST, lease expiry invokes the same close hook, releases log/test
ownership and removes the sleep blocker instead of waiting indefinitely for
the TCP stack. After closure, the normal 60-second SlimeVR-server-absence and
motion policy resumes. Neither the lease nor capture changes NVS.

## Deferred LOGVER3 output

IMU and magnetometer callbacks enqueue immutable fixed-size records only. E1
also queues a one-hertz `NET` snapshot from background service after its due
gate; it never copies Wi-Fi/SlimeVR status from a sensor callback. CSV
float formatting and `Stream::print` run later, after FIFO/AHRS and SlimeVR work,
under the optional-service slack gate. At most one complete CSV line is
serialized per service call. Multi-line Q/FIFO/BIAS/CAL and MAG/MAGR/YAW bundles
stay at the queue head until complete.

The fixed pipeline exposes:

- producer queue drops and high-water;
- serialized/enqueued counts;
- service deferrals;
- shutdown/disconnect drops;
- maximum record age;
- bounded-console complete-line drops.

`log finish` stops producers while allowing queued records to drain. Only after
`LOGSTAT,PIPELINE` reports `queued=0` and `enqueued=serialized` should the client
request the final summary and issue `log off`.

After a measured test closes, `test summary static|runtime` emits one compact
immutable `TESTSUM` CSV row with exact full-rate sensor/FIFO/network deltas.
The retained multi-page `test report` is available over USB or TCP and remains outside the measured completion path.

## Unattended cable-free capture

Use the host tool rather than manually copying terminal output:

```bash
python3 tools/capture_telnet_log.py \
  --host <tracker-ip> \
  --capture static \
  --seconds 600 \
  --rate 20 \
  --mode full \
  --output logver3_static_clean_001.log
```

It requires ProductionDiag, a known full 40-hex base commit, a valid source
fingerprint and a live remote session. Clean and dirty builds are accepted; a
dirty build must report the exact `<head>+<worktree>-dirty` identity, which is
preserved in the manifest. Static capture additionally requires a
ready trusted calibrated MAG/yaw path, a calibrated base gyro bias and live
SlimeVR UDP. It resets counters, starts a full 20 Hz log and test (1..21600 seconds),
drains the logger, checks lifecycle/console counters, validates strict LOGVER3
E1 and promotes the log/manifest pair only after both candidates are ready.
Failures preserve a unique partial capture.

To retain an already-occurring SlimeVR problem, use `--capture runtime` with
UDP and the normal server/tracker load left active. Structural corruption still
fails; network/sensor health problems are saved and reported as
`health_passed=false`. TCP is not an independent channel: it shares the same
radio, Wi-Fi association and lwIP buffers with UDP, so a general radio or shared
memory failure can disconnect both and yield only a partial capture.

Power the tracker from its battery and physically disconnect USB before a
magnetic static-golden run. Moving log traffic from USB to TCP does not remove
the magnetic influence of a cable that remains attached for power.

## Backpressure acceptance

The bounded console commits or drops complete lines; it never glues fragments
together. A later warning reports dropped records. Tracking always has priority
over diagnostics.

A fixture candidate is invalid if any of these is non-zero:

- `LOGSTAT,BACKPRESSURE`;
- producer, shutdown or disconnect drops in `LOGSTAT,DROPS`;
- remote console bytes/records/oversized records/pending warnings;
- remote capture aborts;
- remote session lease expirations;
- FIFO full/overrun/unknown/fallback or unexpected recovery.

Service deferrals may be non-zero; they are expected when tracking slack is
temporarily unavailable. The strict gate instead bounds maximum record age and
requires the pipeline to drain completely.

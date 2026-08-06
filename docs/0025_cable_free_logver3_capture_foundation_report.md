# 0025 cable-free LOGVER3 capture foundation

Date: 2026-08-02

Prerequisite: `0024_trusted_host_baseline_and_release_identity.patch`, applied
to `631fcc4c31892d5f404033b20fce6c03a4fabc3e`.

## Result

This patch stops at the first honest hardware checkpoint. It provides the
firmware transport, low-overhead diagnostics, unattended host capture and
fail-closed LOGVER3 integrity gate needed to obtain a candidate static log. It
does not invent a positive fixture or golden JSON without a real tracker.

Normal Production is now the committed default and does not link the TCP
listener. `BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG` has a distinct
`ProductionDiag` identity and is the explicit battery-powered capture image.

## Predecessor audit

Before starting this wave, patch 0024 was reapplied and reviewed for failure
and cleanup behavior. The corrected 0024 artifact closes the findings from
that review:

- release builds reject stale/incremental `.pio` artifacts;
- manifests reject missing, zero-size or unknown-toolchain firmware;
- concurrent native runs use private build directories;
- timeout cleanup terminates the complete process group on POSIX and Windows;
- Git-status failures, environment restoration and temporary-process cleanup
  fail closed.

The corrected 0024 host gate passes `44/44`. Its ESP32/LSan/HIL limitations
remain explicit rather than being reported as passes.

## Firmware changes

### Profile and remote policy

- committed default: Production;
- USB monitor baud: 921600;
- ProductionDiag: distinct profile identity, complete bounded diagnostic
  surface and Production-family scheduling;
- Production/Slim: TCP listener compiled out;
- command origin is explicit (`UsbSerial` or `RemoteTcp`);
- remote commands pass an exact fail-closed, non-persistent allowlist before
  every dispatcher;
- setup, config/NVS, calibration, network mutation, reset, reboot and other
  destructive commands remain USB-only;
- Telnet IAC/subnegotiation and CR-NUL bytes cannot enter the ASCII parser.

### Session ownership

Machine log and runtime/static tests bind to the initiating transport. A TCP
disconnect invokes the close hook before stream detach, aborts all owned
producers and records the abort. There is no fallback to USB and a later TCP
client cannot inherit an old log session.

Remote log rate is capped at 20 Hz and remote tests at 900 seconds. Static and
runtime tests are mutually exclusive. Detailed retained reports are USB-only;
TCP receives compact progress and completion markers.

### Deferred machine log

IMU/magnetometer callbacks now enqueue immutable fixed-size records. Float/CSV
formatting and `Stream::print` execute only from the slack-admitted background
service after FIFO/AHRS/SlimeVR work.

The bounded contracts are:

- 12 queued records;
- each record at most 224 bytes;
- one complete CSV line per service call;
- line-atomic staging of at least 384 bytes;
- ProductionDiag TCP queue/staging: 8192/768 bytes;
- ProductionDiag USB queue/staging: 1536/512 bytes;
- explicit producer, service-deferral, shutdown, disconnect, high-water and
  maximum-record-age counters.

`log finish` closes producers while allowing the queue to drain. A candidate
requires `queued=0`, equal enqueued/serialized totals and zero loss/abort
counters before `log off`.

### Dormant diagnostic hot path

Debug keeps its diagnostic surface, but disabled diagnostics no longer charge
full timing work to every loop/sample:

- loop/FIFO/sample timing is active only while profiler or a test is active;
- runtime-test section timing is sampled at 1/16 cadence;
- static-test expensive moments are merged in 64-sample blocks;
- Euler conversion and large report formatting occur after the measured
  window;
- CLI polling is an explicit no-inline bounded phase;
- the ordinary `TrackerApp::loop` retains its 256-byte host stack ceiling;
- console, machine-log admission and diagnostic activation have separate
  160/96/64-byte ceilings.

Static aggregation preserves the original numerical statistics. Centered
per-block sums avoid float precision loss for `dt_us^2`; native regression
tests compare blocked and direct statistics.

### Strict LOGVER3 contract

The new parser requires:

- one `LOGVER,3,E0` header with full clean 40-hex Git identity;
- exact, ordered, single `LOGFMT` declarations;
- exact field counts, finite numeric values and unquoted CSV;
- known frames/stats only and no firmware `# ERR` lines;
- contiguous sequence bundles and globally non-rewinding timestamps;
- Q/FIFO/BIAS/CAL and MAG/MAGR/YAW bundle integrity;
- 19–21 Hz observed Q cadence for a requested 20 Hz capture;
- normalized quaternions, hardware timestamps and zero FIFO/recovery/drop
  faults;
- completely drained deferred pipeline with bounded record age;
- exactly one final set of zero-valued console loss/abort counters;
- `STATIC TEST DONE`.

LOGVER2 remains compatibility-only. Release preflight still fails until a real
strict fixture and independently reviewed golden JSON exist.

## Unattended capture

`tools/capture_telnet_log.py` performs:

1. ProductionDiag/full-SHA/clean-worktree/session-origin preflight;
2. live magnetometer initialization/sample/trust/calibration/axis preflight;
3. console/log counter reset;
4. session-bound `log full 20` plus `test static`;
5. producer finish and complete pipeline drain;
6. final log/console status collection;
7. atomic capture and SHA-256 manifest writes;
8. strict LOGVER3 validation.

Failed runs do not overwrite the requested output; a unique partial capture and
failure manifest are retained for diagnosis.

## Verification

- `python3 tools/check_all.py --clean --host-only` — PASS,
  `47/47`, all policies/replay/tool checks;
- `python3 tools/run_standalone_tests.py --clean --sanitizer undefined
  --extra-cxxflag=-Werror` — PASS, `47/47`;
- strict LOGVER3 unit regressions — PASS, `13/13`;
- fail-closed capture promotion regressions — PASS, `4/4`;
- Production, ProductionDiag and Slim `tracker_app.cpp` host profile compiles
  with `-Wall -Wextra -Wshadow -Werror` — PASS (motion-sleep target code
  disabled for this host-only compile);
- profile/source-filter/document validators — PASS;
- `git diff --check` — PASS.

Not run in this environment:

- five authoritative PlatformIO ESP32 builds (PlatformIO is unavailable);
- ESP32 link/map/static-RAM review;
- LSan outside ptrace;
- physical Wi-Fi, battery and no-USB capture;
- HIL magnetometer/FIFO/SlimeVR acceptance.

## Hardware checkpoint

Use one clean ProductionDiag build on the same tracker and location:

1. diagnostics off baseline;
2. `test runtime 600`, log off;
3. `log full 20`, no static test;
4. `log full 20` plus `test static 600`.

The fourth run must be battery-powered with the USB cable physically removed:

```bash
python3 tools/capture_telnet_log.py \
  --host <tracker-ip> \
  --seconds 600 \
  --rate 20 \
  --mode full \
  --output logver3_static_clean_001.log
```

Do not create or commit the positive fixture/golden if any drop, recovery,
fallback timestamp, schema/chronology error, console abort, rate miss or stack/
performance budget violation is observed. Full logs should remain outside Git;
only a minimal reviewed fixture, its independent golden and hashes belong in
the following acceptance patch.

## Compatibility and rollback

Persistent config/NVS layouts, calibration bytes, IMU ODR, FIFO chronology,
AHRS equations and SlimeVR packet format are unchanged. The observable CLI
changes are the safe default profile, 921600 monitor baud, remote allowlist,
session ownership, compact test completion and deferred `test report` output.

Rollback is the reverse application of this patch. Reverting only the profile
or transport pieces is unsupported because source filters, feature flags,
session ownership and capture tooling form one contract.

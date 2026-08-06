# 0025a cable-free diagnostic capture hardening

Date: 2026-08-03

Prerequisites, applied in order to
`631fcc4c31892d5f404033b20fce6c03a4fabc3e`:

1. `0024_trusted_host_baseline_and_release_identity.patch`;
2. `0025_cable_free_logver3_capture_foundation.patch`.

## Result

This incremental patch fixes the concrete defects found while independently
reviewing 0025. It makes the cable-free logger useful for both a clean static
golden candidate and an already-failing SlimeVR UDP session without claiming
that TCP is physically independent of Wi-Fi/lwIP.

No persistent configuration, calibration bytes, IMU/FIFO/AHRS equations,
SlimeVR packets or output cadence change. Normal Production still compiles the
TCP console and machine logger out. ProductionDiag remains a temporary
diagnostic image.

## Audit findings closed

### UDP was invisible to the strict log

E0 could pass while SlimeVR UDP was losing sends or reopening its socket. E1
adds one-hertz `NET` records containing Wi-Fi state, RSSI, SlimeVR/UDP readiness,
rotation sent/due/deadline counts, classified send failures, rebind/full-reopen
counts and current recovery state. Status copies occur only after the one-hertz
due gate and are queued in the existing deferred background serializer; no
Wi-Fi/SlimeVR status copy runs from IMU or magnetic callbacks.

`last_udp_error` is retained by SlimeVR as historical diagnostic context even
after successful sends resume. E1 therefore uses measured counter deltas,
consecutive failures and the current TX recovery state for health decisions;
a stale old errno alone cannot reject a later clean window.

Static and runtime captures intentionally have different acceptance semantics:

- `static` is fail-closed: sensor, MAG/BIAS, UDP, deadline or lifecycle health
  faults reject the candidate;
- `runtime` preserves a structurally complete problem capture and returns
  `health_passed=false` with the exact observed faults. Logger/console loss,
  malformed chronology or an incomplete drain still fail structurally.

SlimeVR UDP remains enabled in both modes. Turning it off would remove the
load/failure being investigated and is only useful as a separate A/B control.

### Completion evidence was too weak

The measured path emitted only `STATIC/RUNTIME TEST DONE`, while the exact
full-rate result lived in a retained USB-only report. A new immutable compact
`TESTSUM` row is explicitly requested after the measured window closes. It
contains exact duration, sample/timestamp/FIFO counters, network deltas and,
for static tests, gyro/accel/temperature and MAG trusted/rejected results. The
multi-page report remains out of the IMU/loop completion path.

### Sleep cleanup was not bounded for half-open TCP

An accepted diagnostic session now blocks motion light sleep, including the
preflight interval before `log start`. The host sends a Telnet IAC NOP every
five seconds. If the firmware consumes no input for 30 seconds, its
application-level lease expires and the ordinary close hook aborts owned
log/tests, clears queued output, releases their stream pointers and removes the
sleep blocker. This does not depend on a much longer lwIP half-open timeout.

On normal completion the tool performs `log finish`, exact summary/drain,
`log off`, verifies both tests inactive and closes TCP. If the tool fails after
starting work, it best-effort sends `test stop`, `log finish` and `log off`;
disconnect/lease cleanup is the final firmware-side fallback. After release,
the existing 60-second no-server/no-motion sleep policy starts normally again.
Nothing is written to NVS.

### Diagnostic callback repeated hot-path work

0025 passed already-computed sensor values through a callback that then
re-evaluated current gyro bias and temperature-compensation flags before the
logger's `off` check. The pipeline now checks `logState.accepting()` before the
callback and passes the single per-sample `GyroTempCompRuntimeEval` and current
bias already used by calibration/quality/runtime bias. The policy rejects a
return of the duplicate evaluations. Production remains compile-time free of
the logger; dormant ProductionDiag also avoids this cost.

### MAG/BIAS presence did not prove usable state

`mag processed` now ends with one compact, internally consistent preflight
snapshot spanning processed validity, calibration/alignment, heading, field
trust/reference and yaw gate/apply state. `bias status` exposes source and base,
temperature-model/range validity. Static capture requires a calibrated base
bias and ready trusted MAG/yaw state, then the E1 validator checks every logged
MAG/BIAS row and every yaw gate/apply row semantically. Runtime capture records
unhealthy state instead of silently changing it.

The tool never enables MAG, runtime bias or yaw correction and never persists
settings merely to make a log pass. The log describes the configuration that
is actually running.

### Capture/manifest promotion was not pair-consistent

0025 could replace the requested log and then fail while building/writing its
manifest. 0025a validates the candidate and fully serializes/fsyncs both
candidate files before replacing either destination. An ordinary replacement
error rolls both destinations back to their previous generation. The manifest
hash is the authoritative pair check after an unclean host power loss between
the two filesystem renames. Rejected/error candidates never replace a known
good pair.

Rollback restores the two paths independently. If storage failure also blocks
one restoration, the other path is still restored and the unrecovered
`.rollback-*.tmp` remains on disk as the only recoverable old generation; the
tool reports the exact backup path instead of deleting it in cleanup.

### New E1 declarations exceeded USB staging

The first E1 field names made `LOGFMT,TESTSUM` longer than the existing
512-byte USB record staging, although the 768-byte TCP staging accepted it.
E1 now uses compact field names; every unquoted `LOGFMT` line, including CRLF,
is below 512 bytes. RAM queues and their existing ceilings are unchanged.

## Collection workflow

Build and flash the temporary diagnostic image from a clean committed tree:

```bash
python3 tools/check_all.py --clean --require-pio
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG -t clean
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG -t upload
```

For a clean static candidate, keep SlimeVR Server running, leave UDP enabled,
place the tracker motionless in a stable magnetic environment, power it from
the battery and physically disconnect USB:

```bash
python3 tools/capture_telnet_log.py \
  --host <tracker-ip> \
  --capture static \
  --seconds 600 \
  --rate 20 \
  --mode full \
  --output logver3_static_clean_001.log
```

For the reported in-game UDP failure, reproduce the problem with SlimeVR and
the normal tracker count/load still active:

```bash
python3 tools/capture_telnet_log.py \
  --host <tracker-ip> \
  --capture runtime \
  --seconds 600 \
  --rate 20 \
  --mode full \
  --output logver3_runtime_udp_issue_001.log
```

A runtime capture can be successfully saved with `health_passed=false`; that
is the desired result when it contains the UDP fault. Send the `.log` and its
`.manifest.json` together. On any failed/partial run, preserve the unique
`.partial-*` log and JSON before retrying.

After diagnosis, flash ordinary Production to remove the listener and all
diagnostic RAM/code footprint:

```bash
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION -t clean
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION -t upload
```

## Honest transport boundary

TCP/Telnet and SlimeVR UDP use the same ESP32 radio, Wi-Fi association and
lwIP memory. An endpoint/rebind-specific UDP defect may leave TCP healthy, but
radio loss, association loss, shared buffer exhaustion or severe event-loop
starvation can slow or disconnect both. In that case the tool saves a partial
capture and firmware lease cleanup still restores normal ownership/sleep; no
patch can make same-radio TCP an independent out-of-band channel.

## Verification and remaining checkpoint

The final incremental tree passed:

- aggregate host gate, including all policy/replay checks: `PASS`;
- native matrix: `48/48`;
- independent `-Werror` plus UBSan native matrix: `48/48`;
- strict E1 contract regressions: `21/21`;
- capture/pair-promotion regressions: `10/10`;
- profile matrix, documentation validation and `git diff --check`: `PASS`.

These checks cover exact E1 schema/semantics, runtime-health retention,
transaction rollback, lease wraparound, deferred completion, command ownership
and line-size/RAM/stack policies. PlatformIO is unavailable in the verification
environment. Target ESP32 build/map, real Wi-Fi behavior, battery sleep current
and the first 10-minute no-USB fixture therefore remain hardware checkpoints
and must not be represented by a synthetic positive golden.

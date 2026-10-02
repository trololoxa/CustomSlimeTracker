# Device Smoke

## Bounded device workflow

`tools/device_workflow.py` owns USB selection, cooperative locking, app-only flash,
serial smoke and offline crash symbolization. It does not build, install packages,
change configuration, erase NVS, enable streams, issue reboot/factory reset commands,
or start OpenOCD. Flash explicitly resets the chip. Even a serial open can pulse
control lines on some drivers/boards; use a bench device, not an active game session.

Use the explicit Python/PlatformIO paths from [session setup](session.md). `pyserial`
is already in the project's PlatformIO dependency environment. Close VS Code serial
monitors and other clients before connecting. Run hardware commands from Windows;
WSL USB forwarding, sudo and cross-OS locks are not configured by this tool.

First enumerate without opening any port:

```powershell
& $TrackerPython tools/device_workflow.py list
```

Copy the exact `port`, `serial`, `vid`, `pid` from that output. For example, edit
these assignments for YOUR device; VID/PID accept decimal or hexadecimal:

```powershell
$TrackerPort = 'COM13'
$TrackerUsbSerial = 'REPLACE_WITH_LIST_SERIAL'
$TrackerVid = '0x303a'
$TrackerPid = '0x1001'
$TrackerDevice = @('--port', $TrackerPort, '--usb-serial', $TrackerUsbSerial, '--vid', $TrackerVid, '--pid', $TrackerPid)
$TrackerEnvironment = 'BOARD_LOLIN_C3_MINI_USB_DIAG'
```

Missing/duplicate USB serial identities fail before opening a port. There is no
"first device" fallback. Identification refers to the USB endpoint, not proof of
which board is connected behind a generic USB/UART adapter. Lock files are shared
by checkouts for the current OS user and automatically unlocked on process exit;
do not delete lock files to bypass ownership. Other users/OSes/external monitors
must be coordinated manually. Re-enumeration causes a stop, not an automatic rebind.

For full `health` over USB, use the [USB diagnostic bench variant](../reference/build_profiles.md#usb-diagnostic-bench-variant).
Normal ProductionDiag's 1536-byte serial queue can discard report lines; a longer
host timeout cannot recover them. Keep the actual running environment in
`$TrackerEnvironment`; changing a host variable does not change flashed firmware.
The bench variant preserves the same drop safeguards and scheduling budgets.

### Smoke an already running build

Use the exact `build_git` from that firmware's `version` output or its saved build
manifest, not today's repository HEAD. Check six health observations (about ten
seconds on a responsive board), without changing runtime configuration:

```powershell
$TrackerDeviceBuild = 'REPLACE_WITH_EXACT_BUILD_GIT'
& $TrackerPython tools/device_workflow.py smoke @TrackerDevice --expect-build $TrackerDeviceBuild --environment $TrackerEnvironment
```

The initial version handshake can resynchronize at its exact header after an
unterminated boot-banner prefix; raw bytes and `initial_version_header_resyncs` are
retained. Its 3 s deadline and single command remain unchanged. Panic/safe-mode
still fail. Boot text during initial attachment is outside the observation window;
after version validation, reboot detection and all response framing stay strict.

The report requires matching version/profile, complete framed responses, monotonic
uptime/counters, gyro sample and pose sequence progress, fresh prepared output and
finite near-unit AHRS quaternion. Budgets are recorded in JSON: pose age <=100 ms,
blackout <=4 s, at most two increments per recovery/drain/fault counter over the
observation window. These are host smoke criteria, not changed firmware gates.
Queue overflow/batch-capacity violations, exhausted recovery, sensor-fault state,
observed reboot/panic and invalid quaternions fail. At least three observations and
two final healthy frames are mandatory. Historical counters before the window do
not cause a failure by themselves. Missing fields/timeouts mean insufficient
smoke evidence; they do not authorize firmware shutdown or repeated flashing.

`DEGRADED_MAG`/`DEGRADED_ACCEL` can pass the 6D liveness check and are reported.
Add `--require-mag` only when testing a calibrated magnetic stream: both magnetic
sample and trusted-sample counters must advance. This is not 9D accuracy proof.
Sampled status cannot prove every intervening quaternion, absence of every short
blackout, gyro timing/WCET, stationarity, network delivery or physical accuracy.

### Review and explicitly flash a saved build

Use a manifest made by `check_all.py` for the same build artifacts. It must contain
`firmware.bin`, `firmware.elf`, `partitions.bin`, known embedded source identity and
an uploadable profile. Hashes, ESP32-C3 app/header, RISC-V ELF identity and the
reviewed no-OTA partition layout are validated before device access. Old manifests
with overwritten artifacts fail; do not regenerate a manifest to bless unknown
stale binaries. For separately built artifacts see `release_manifest.py --help`.

```powershell
$TrackerManifest = "build\check_all\platformio\manifests\$TrackerEnvironment.json"
& $TrackerPython tools/device_workflow.py flash-plan --manifest $TrackerManifest
```

`flash-plan` does not enumerate/open/reset hardware. Save the previous known-good
manifest and binaries before updating a board; this no-OTA layout has no automatic
rollback. For native USB use the verified standalone esptool 4.9.0 environment. The pinned
PlatformIO uploader remains 4.5.1; changing reset flags alone did not fix its stub
communication failure. Do not replace global PlatformIO packages. These are individual
PowerShell commands, not a script file. One-time setup (installation is explicit):

```powershell
& $TrackerPython -m venv 'H:\TrackerTools\esptool-usb-probe-4.9.0'
$TrackerUsbProbePython = 'H:\TrackerTools\esptool-usb-probe-4.9.0\Scripts\python.exe'
& $TrackerUsbProbePython -m pip install 'esptool==4.9.0'
& $TrackerUsbProbePython -m pip check
& $TrackerUsbProbePython -m esptool version
```

Stop after any nonzero exit; use `$LASTEXITCODE` immediately after each command.
If this environment already works, only restore `$TrackerUsbProbePython` after a
shell restart; do not reinstall. After reviewing the manifest and explicitly
authorizing flash of the selected tracker:

```powershell
& $TrackerUsbProbePython tools/device_workflow.py flash @TrackerDevice --manifest $TrackerManifest --execute
```

The default runs `-m esptool` from the SAME Python environment. Optional `--esptool`
accepts an existing Python entry script, also version-checked before reset/read/write.
Windows installs need not contain `Scripts\esptool.py`; never assume that path.

The tool selects `usb_reset` before operations for verified 303A:1001 endpoints,
otherwise `default_reset`. Native USB keeps the initial 115200 baud to skip
esptool v4 baud switching; other endpoints retain 460800. The selected baud is
recorded in the report. This does not change the runtime console baud. Intermediate steps use `no_reset`; the last app read-back
uses `hard_reset` to start the application. Reset policy is recorded in the report.
These host flags are explicit because this uploader does not execute PlatformIO's
upload target. Port re-enumeration still requires explicit selection after `list`.

The tool snapshots validated artifacts, requires esptool 4.9.0, reads the device partition
table, and compares it byte-for-byte before writing only the app at `0x10000`.
It then reads the app back and compares SHA-256. App read-back has a 600 s
deadline (the 1.86 MB bench read took 162.8 s); other commands retain 180 s. Full
read-back remains mandatory on every controlled flash, adding time but no erase/write
cycles. Failures never trigger automatic retry, erase, rebind or rollback.
Bootloader, partitions and NVS are not written by this uploader. The firmware's own
startup/migration behavior is outside that guarantee. A blank/differently partitioned
board is deliberately unsupported; initial provisioning needs a separate decision.
A failure can leave ROM download mode or a partial app: retain evidence, inspect the
cause and explicitly reflash the saved known-good image if appropriate.

A successful flash report certifies bytes, not boot/sensor health. Re-run `list`,
then `smoke` with the manifest's source identity. Do not flash merely to test tooling.
The command syntax is from [esptool v4](https://docs.espressif.com/projects/esptool/en/release-v4/esp32c3/esptool/basic-commands.html).

### Boot/capture and crash evidence

Passive capture sends no CLI commands and does not request reboot:

```powershell
& $TrackerPython tools/device_workflow.py listen @TrackerDevice --seconds 15
```

It records boot/panic text if emitted while attached; no boot occurrence or boot
success is invented from silence. Review `boot_health` reset/safe-mode lines when
present. This is a bounded capture, not a substitute for sensor smoke. Raw serial
capture is limited to 2 MiB. For long cable-free LOGVER3 recording use the existing
`capture_telnet_log.py` workflow below; it owns logger/session changes and validation.

Keep the ELF/manifest of the CRASHED build. Offline symbolization needs no device:

```powershell
$TrackerAddr2line = Join-Path $env:PLATFORMIO_CORE_DIR 'packages\toolchain-riscv32-esp\bin\riscv32-esp-elf-addr2line.exe'
$TrackerCrashLog = 'REPLACE_WITH_SAVED_CRASH_LOG'
& $TrackerPython tools/device_workflow.py symbolize --manifest $TrackerManifest --crash-build $TrackerDeviceBuild --crash-log $TrackerCrashLog --addr2line $TrackerAddr2line
```

Caller-supplied crash identity must match the manifest; addresses without a known
build are not trustworthy symbol evidence. Output can contain unresolved addresses.
Do not use MSYS host GDB for ESP32-C3. Live JTAG/OpenOCD needs the board's actual USB
JTAG/wiring/driver setup and an explicit attach request; attaching/halting disrupts
tracking. No verified live-debugger setup or flash core-dump partition is claimed.

Each operation writes bounded raw logs and a scoped summary under `build/gate_runs`.
Keep important evidence with its artifacts outside the ephemeral build directory.
Host fake-transport tests do not certify serial drivers, flash, or physical recovery.
Target acceptance remains: enumerate/lock; smoke existing firmware; one explicitly
authorized app update/read-back and subsequent smoke; capture one boot and inspect
reset/safe-mode; symbolization of a known crash when available. No deliberate panic,
NVS erase or unsafe fault injection is required merely to accept host tooling.

## Hardware/runtime test budget

Do not request hardware tests for behavior that is already fully covered by host
logic and profile builds. Use this default matrix:

| Change | Native/project-contract checks | PlatformIO builds | Tracker runtime test |
|---|---:|---:|---:|
| Documentation | Documentation validation | None | None |
| Host tools | Affected tooling tests | None unless build integration changed | Fake transport first; real device only for hardware-facing claims |
| Profile policy or source filters | Relevant policy checks | Affected profiles | None |
| Pure math, packet encoding or host-safe state machine | Required | Required | None unless hardware integration changed |
| FIFO/IMU driver or timestamp integration | Required where possible | Required | One focused serial/telnet smoke test |
| Calibration capture using real sensors | Required for fit/state logic | Required | One focused capture only |
| Wi-Fi/UDP reconnect or server protocol integration | Required with fake transports | Required | One focused server smoke test |
| Sleep/wake or power policy | Required for controller logic | Required | One dedicated A/B test stage |

Long `test static` or `test runtime` captures are release/acceptance tools, not a
mandatory response to every patch. Prefer one short test that crosses the exact
hardware boundary changed by the patch. Do not ask for several ideal-condition
captures when the same regression can be proven by native tests or a fake
transport.

## Real-time output resilience acceptance

The host gate covers phase-locked rotation deadlines, jitter/late-loop catch-up,
`millis()` wraparound, complete-record admission/drop behavior, oversized-line
rejection, ring wrap, partial drains, drop-warning insertion, reset semantics,
stalled sinks and empty-drain no-op behavior. Native executable coverage now
includes session ownership, Telnet filtering, deferred diagnostic completion,
blocked-static numerical equivalence and machine-log producer/backpressure
behavior.

The hardware checkpoint for the cable-free capture foundation is:

1. Debug, diagnostics off.
2. Debug, `test runtime 600`, log off.
3. ProductionDiag, `log full 20 Hz`, no static test.
4. ProductionDiag, `log full 20 Hz` plus `test static 600`.

Run the final candidate from battery with USB physically disconnected. Use
`tools/capture_telnet_log.py`; it performs known clean-or-dirty identity and magnetometer preflight,
session-bound capture, pipeline drain, strict validation and manifest writing.
Acceptance requires 19..21 Hz Q, zero FIFO faults/recovery and every producer,
pipeline, disconnect and console drop counter at zero. Service deferrals may be
non-zero, but maximum record age must remain within the strict bound.

## Running firmware diagnostics

Firmware diagnostics still need PlatformIO and the ESP32-C3. You can run them directly or through the quality gate:

```bash
python tools/check_all.py --require-pio
```

Direct commands:

```bash
pio run -e BOARD_LOLIN_C3_MINI_DEBUG
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG
pio run -e BOARD_LOLIN_C3_MINI_SLIM
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG -t upload
pio device monitor
```

Recommended smoke-test commands after an architectural change:

```text
help
status
health
config nvs
fifo stats
quality stats
ahrs status
bias status
mag status
stream
net status
slime status
stream quat
stream off
test status
test static 120
```

After a CLI-domain refactor, also touch each command domain once. The goal is not
to validate sensor quality, but to catch missing `.cpp` includes, broken hook
wiring, and command router regressions on the real firmware build:

```text
version
config print
imu status
fifo status
quality stats
ahrs status
bias status
mag status
stream
net status
slime status
test status
cal temp print
```

If IMU/FIFO live reconfiguration was touched, verify that the magnetometer path
is re-armed correctly after the change:

```text
imu rate 240 save
fifo stats
mag status
```

For long-run stability after a risky runtime change:

```text
test static 3600
```

Important acceptance metrics:

```text
estimated_dropped_samples=0
fifo_overrun_delta=0
fifo_full_delta=0
fallback_timestamp_samples=0
tracking_recovery_active=no
bad_timestamp_samples=0
```

`yaw_drift_rate_deg_min` is useful, but it is not expected to be zero in 6DoF
mode without magnetometer correction.

## Motion light sleep

USB_DIAG disables manual and automatic motion light sleep to keep USB available
without a Server. It cannot certify sleep/wake; use normal ProductionDiag for that.
An open USB console alone does not block sleep in normal ProductionDiag.

See [motion_light_sleep.md](../architecture/sleep.md) for the opt-in GPIO10/INT1 light-sleep bench procedure and its host-test coverage.

## Tap detector diagnostic capture

For a physical tap investigation, keep the tracker connected to SlimeVR and open either USB Serial or the Wi-Fi remote console. Run:

```text
tap reset
tap log on
tap status
```

Tap the enclosure several times, then run `tap status` and `slime status`. `tap log` emits only nonzero `TAP_SRC`, decoded single/double/axis bits, accumulator queue or suppression decisions, and the final SlimeVR send result; it deliberately does not print the idle 5 ms polls. The same `# TAP_LOG ...` lines are mirrored to Serial and the active remote-console/telnet client. Use `tap log off` after the capture.

Interpretation: no `event=tap_src` means the LSM6DSV hardware engine did not report an event; `tap_src` without `physical_tap` indicates an unexpected source-bit pattern; `suppressed_below_min`, `suppressed_duplicate`, or `suppressed_lockout` identifies firmware gesture filtering; `slimevr_no_server` / `slimevr_send_failed` identifies the output path.

## FIFO coherency acceptance

After flashing `c3-6dsv-fifo-coherency` in ProductionDiag, use `fifo status`,
`motion status`, and `perf tracking`. `motion status` already contains the full
FIFO and quality correlation blocks. The complete ProductionDiag image also
links `fifo stats` and `quality stats`; TCP and USB expose the same status, reset and reconfiguration commands compiled into that profile.
Normal operation should keep gyro-only and pair-mismatch counters at zero or
extremely rare values. Isolated component
loss may increment them, but must not request FIFO recovery, stop gyro
integration, or make linear acceleration valid for that degraded sample.

A dynamic `ACCEL_NORM_OUTLIER` is different: it must disable use of accel as an
AHRS gravity observation without suppressing motion output. During movement,
`acceleration_sent_delta` should therefore track `rotation_sent_delta` unless a
separate `acceleration_skipped_*_delta` counter identifies a hard invalidity.
`slime_last_rotation_snapshot_age_us` is MCU publish-to-send age and should stay
near the output period; it no longer compares the LSM6DSV timestamp epoch with
ESP32 `micros()`.


## Protocol 22 motion-frame acceptance

`test_slimevr_motion_frame` locks the rotation/acceleration local-frame
contract. It verifies that the protocol adapter preserves device axes while
converting acceleration from `g` to `m/s^2`, keeps Hamilton
`q_world_from_device`, and emulates current server processing. The protocol-22 branch must reproduce one
coherent world-space motion vector; the legacy pre-22 extra -90 degree local-Z
acceleration correction must demonstrably disagree.

After flashing `c3-6dsv-calibration-epoch-field-safe` in ProductionDiag:

```text
slime status
motion on
# move the tracker strongly along its marked +X, +Y and +Z directions
motion status
```

Expected status includes:

```text
protocol_version=22
motion_frame_contract=device_x_right_y_forward_z_up
rotation_convention=world_from_device
acceleration_frame=device
acceleration_units=mps2
legacy_acceleration_correction=no
motion_frame_config_ready=yes
step_mounting_ready=yes
motion_packet_mode=bundle_100_rotation_17_accel_4
packet23_available=yes
packet23_enabled=no
sensor_info_sync_state=acknowledged
feature_negotiation_state=negotiated
server_feature_flags_available=yes
server_bundle_supported=yes
```

During movement on a bundle-capable server, `bundled_motion_sent`,
`acceleration_sent`, and `rotation_sent` should advance together and all hard
`acceleration_skipped_*` reasons should remain zero.
`bundled_motion_send_failures_delta` and `udp_send_failures_delta` should remain
zero. A server that does not answer FeatureFlags must report
`rotation_17_plus_accel_4_fallback`; rotation remains at the configured rate,
while `acceleration_rate_limited_delta` confirms the 50 Hz packet-4 fallback.
Under intentional UDP TX pressure, `tx_pressure_failures_delta` and
`tx_backoff_drops_delta` may rise. Recent valid ping/heartbeat reception should
select `udp_transport_rebind_successes_delta`; a failed rebind, stale RX or a
second burst may instead increment `udp_full_reopen_escalations_delta` and
`udp_reopen_requests`. `foreign_endpoint_packets_dropped`,
`pre_session_packets_dropped` and all `malformed_*` counters should remain zero on
a normal single-server LAN. `tap_user_action=off` is the default unless a mapping
was explicitly saved. The definitive directional acceptance is a successful
server step-mounting run; ordinary quaternion FBT can look correct even when
acceleration alone has the wrong local axes.

The build-profile validator also verifies that every ESP32-C3 environment inherits
`partitions/tracker_4mb_no_ota.csv`, that the factory app is exactly 3 MiB at
`0x10000`, that no OTA slot exists and that the complete layout ends at the 4 MiB
flash boundary.

##  session acceptance

After the server is found, run:

```text
slime debug
```

A normal bundle-capable session should converge to:

```text
sensor_info_sync_state=acknowledged
feature_negotiation_state=negotiated
server_bundle_supported=yes
foreign_endpoint_packets_dropped=0
pre_session_packets_dropped=0
malformed_packets=0
set_config_flag_apply_failures=0
ack_config_send_failures=0
protocol_change_ignored=0
```

`slime action yaw|full|mounting|pause` must emit packet 21 and advance the
corresponding UserAction counters. `slime tap-action ... save` must survive reboot;
the default is `off`. A repeated server magnetometer command after a deliberately
lost ACK must be acknowledged without another config write or magnetic-runtime
restart. Boot with magnetometer enabled must not report a redundant
`tracking_recovery_reconfigure_delta` solely from starting QMC6309 after FIFO
bootstrap.

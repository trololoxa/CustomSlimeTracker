# Serial CLI reference

The serial CLI is a developer/configuration interface. It is text based, fixed-buffer, no-heap, and non-blocking while idle. Command implementations live in `src/serial/*_commands.cpp`; headers expose only cross-module APIs.

Legend:

- **Runtime**: changes current boot/session state.
- **Persisted**: changes NVS/config when `save` is used or when the command explicitly saves.
- **Blocking**: command may occupy the firmware while it gathers calibration samples.

## System

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `help`, `?` | Print command list | No | Human-readable reference. |
| `status` | Print runtime status | No | Uses runtime status hook when available. |
| `health` | Print status + quality/FIFO health | No | Uses runtime health hook when available. |
| `version` | Print firmware/build identity and CLI protocol | No | Shows profile, PlatformIO environment, Git HEAD, dirty worktree fingerprint and handshake firmware string. |
| `reboot` | Restart ESP32 | No | Flushes output before restart. |
| `factory_reset` | Reset runtime config defaults and erase config store | Yes | Reboot recommended after success. |
| `sleep` | Queue motion-triggered ESP32 light sleep | No | Compiled only with `TRACKER_ENABLE_MOTION_LIGHT_SLEEP=1`; serial/FIFO/Wi-Fi/mag are stopped after the current CLI poll returns, and a qualifying LSM6DSV motion event wakes the tracker. |
| `remote status` | Print Wi-Fi TCP console state | No | Available when `TRACKER_ENABLE_WIFI_REMOTE_CONSOLE=1`. |
| `remote off` / `remote on` | Stop/start the Wi-Fi TCP console for the current boot | No | `remote off` closes the TCP client/server so it stops adding normal-loop work. |
| `console status` | Print bounded USB Serial and telnet output state | No | Shows queue/record capacity, staged and queued bytes, complete-record drops, warning notices, drain stalls and the current USB stall backoff. |
| `console reset` | Reset bounded-output queues and counters | Runtime | Discards stale queued/staged console text, then starts fresh counters. It does not touch IMU/FIFO queues. |

## Config

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `config print` | Print active config | No | Inspection only. |
| `config crc` | Print active config CRC | No | Inspection only. |
| `config nvs` | Inspect config store | No | Reads store metadata/status. |
| `config defaults` | Reset active config to defaults | No | Does not erase NVS until save/erase. |
| `config load` | Load config from NVS | Runtime | Applies loaded values to runtime where supported. |
| `config save` | Save the current config plus supported non-calibration runtime policy | Yes | Does not re-snapshot or re-date calibration evidence. Calibration commands update `TrackerConfig` when a real calibration event occurs; use `cal save` to explicitly capture all current calibration runtimes. |
| `config erase` | Erase config store | Yes | Reboot recommended. |
| `config spi [<hz> [save]]` | Inspect or live-reconfigure SPI clock | Optional | Transactional: runtime apply and NVS save roll back to the previous clock on failure. |

## IMU/FIFO/quality

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `imu status` | Print IMU config/status | No | Inspection only. |
| `imu whoami` | Print last WHOAMI | No | Hardware sanity check. |
| `imu read` | Read one direct sample | No | Debug path, not FIFO runtime. |
| `imu rate <120|240|480|960> [save]` | Live IMU+FIFO ODR reconfigure | Optional | Stops stream/log during reconfigure and re-arms mag if needed. |
| `config fifo [status]` / `fifo status` | Print persisted FIFO tuning and live counters | No | The short `fifo` alias is available in Production/ProductionDiag without enabling the full developer FIFO CLI. |
| `config fifo watermark <1..255> [save]` / `fifo watermark ...` | Live FIFO watermark reconfigure | Optional | Transactional hardware apply; clears pre-reconfigure software queues, requests controlled orientation recovery, and rolls back on apply/save failure. |
| `config fifo drain <16..4096> <1..32> [save]` / `fifo drain ...` | Change bounded drain limits | Optional | Takes effect on the next app-loop drain; `save` persists before the in-RAM config is committed. |
| `fifo stats` | Print detailed FIFO counters | No | Full developer CLI only. Production/ProductionDiag use `health` or `perf tracking`. |
| `fifo reset` | Reset hardware FIFO/runtime path | No | Full developer CLI only; intentionally requests orientation recovery. |
| `quality stats` / `quality reset` | Inspect/reset detailed quality counters | No | Full developer CLI only. ProductionDiag uses non-destructive `perf tracking reset` baselines. |

## Calibration

| Command | Effect | Persisted | Blocking |
|---|---|---:|---:|
| `cal gyro` | Capture stationary gyro bias from FIFO | Runtime | Yes |
| `cal gyro save` | Save current gyro calibration | Yes | No |
| `cal gyro clear` | Clear gyro bias calibration | Runtime | No |
| `cal accel face XP|XN|YP|YN|ZP|ZN` | Capture one 6-position accel face | Runtime | Yes |
| `cal accel compute` | Compute accel 6-position full 3x3 affine calibration | Runtime | No |
| `cal accel dump` | Print captured accel faces | No | No |
| `cal accel save` | Save accel calibration | Yes | No |
| `cal accel clear` | Clear accel calibration/captures | Runtime | No |
| `cal temp print` | Print gyro temp-comp model | No | No |
| `cal temp enable|disable [save]` | Enable/disable temp compensation | Optional | No |
| `cal temp set_slope X Y Z [save]` | Set temp slope manually | Optional | No |
| `cal temp fit_static [save]` | Fit temp model from last static test | Optional | No |
| `cal temp clear [save]` | Clear temp compensation | Optional | No |
| `cal save` | Save calibration/config | Yes | No |
| `cal clear_all` | Clear all local calibration | Runtime | No |

## AHRS

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `ahrs status`, `ahrs config` | Print AHRS config/status | No | Inspection only. `ahrs status` includes timestamp recovery diagnostics: `large_dt_rebase_count`, `fifo_rebase_count`, `last_rebase_t_us`, and `post_fifo_recovery_samples`. |
| `ahrs reset` | Reset AHRS runtime state | No | Also uses app hook. |
| `ahrs defaults [save]` | Reset AHRS tunables to defaults | Optional | Config-layer defaults. |
| `ahrs accel on|off [save]` | Enable/disable accel correction | Optional | Runtime + optional config. |
| `ahrs adaptive on|off [save]` | Enable/disable adaptive accel gain | Optional | Runtime + optional config. |
| `ahrs accel_kp <gain> [save]` | Set accel correction gain | Optional | Runtime + optional config. |
| `ahrs max_step <deg> [save]` | Set max accel correction step | Optional | Runtime + optional config. |
| `ahrs accel_norm`, `accel_innovation`, `accel_var`, `gyro_gate`, `dt` | Tune gates | Optional | Runtime + optional config. |

## Magnetometer

| Command group | Effect | Persisted | Notes |
|---|---|---:|---|
| `mag status`, `id`, `qmcstatus`, `regs`, `hub`, `fifo` | Inspect mag/sensor-hub path | No | Debug/status only. |
| `mag enable|disable [save]` | Enable/disable mag runtime | Optional | Runtime + optional config. |
| `mag processed`, `mag trust` | Print processed mag/trust state | No | Inspection only. |
| `mag heading ...` | Manage heading reference/auto-reference | Runtime | Reference is runtime state. |
| `mag yaw ... [save]` | Tune yaw correction gates/rates/apply flag | Optional | Applies only when mag path is trusted. |
| `mag axis ... [save]` | Configure mag axis mapping | Optional | Persist after validating orientation. |
| `mag cal start|stop|reset|status|print|apply [save]` | Manage full ellipsoid mag calibration collector | Optional | `apply save` persists hard-iron plus full 3x3 soft-iron matrix when box coverage, directional coverage, robust inlier ratio and residual gates pass. |


## Accumulative tap input

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `tap status` | Print LSM6DSV tap runtime, accumulator and register-check counters | No | Shows hardware config, physical single/double detections, pending window, suppressions, SlimeVR send counters and masked register verification. |
| `tap on` / `tap off` | Enable/disable tap runtime for the current boot | No | Reconfigures the LSM6DSV embedded tap engine and keeps using the existing INT1 line. |
| `tap log on\|off\|status` | Enable/disable/status event-only tap diagnostics | No | When enabled, prints nonzero `TAP_SRC`, decoded physical tap direction/type, accumulator decisions and SlimeVR send outcomes to USB Serial and to the active Wi-Fi remote-console/telnet client. No idle poll spam. Compiled only with `TRACKER_ENABLE_TAP_DIAGNOSTICS=1`. |
| `tap test [2..10]` | Send one manual SlimeVR Tap packet directly | No | Verifies the server path without using the LSM6DSV detector or accumulator. Defaults to `2`. |
| `tap inject <1..10>` | Emulate physical tap events through the accumulator | No | Useful for checking sliding-window aggregation without physically tapping the tracker. A single injected tap is intentionally suppressed by the default min count. |
| `tap reset` | Reset tap counters | No | Does not reset SlimeVR counters. |

LSM6DSV tap recognition is routed through the same physical INT1 line as FIFO events; INT2 is not required. The ISR remains lightweight, and the runtime polls/clears `TAP_SRC` from the normal loop. Hardware double-tap is disabled by default so the sensor reports physical taps one-by-one; firmware then aggregates 2..10 taps in a sliding `TRACKER_TAP_AGGREGATION_WINDOW_MS` window and sends one SlimeVR Tap packet. The driver enables `FUNCTIONS_ENABLE.INTERRUPTS_ENABLE` with a masked write and verifies it together with the tap registers; this preserves the FIFO timestamp bit in the same register. Register verification is performed after configuration and then only every `TRACKER_TAP_REGISTER_VERIFY_INTERVAL_MS` while no tap window is pending, so FIFO timing is not burdened by frequent config reads. The default tap threshold is intentionally moderate (`TRACKER_LSM6DSV_TAP_THRESHOLD=2`) so hand taps can be detected during RC1 tuning; raise it if the enclosure produces false positives. For physical debugging, use `tap log on`; the log prints event-only evidence rather than every 5 ms poll.

## Status LED

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `led status` | Print configured pin, polarity, current mode, override state and write counter | No | Use this first when validating the ESP32-C3 SuperMini onboard LED. |
| `led auto` | Clear manual override and return to runtime-derived status | No | The runtime derives status from Wi-Fi and SlimeVR state. |
| `led on` / `led off` | Force the GPIO LED on/off for the current boot | No | Hardware smoke-test for pin/polarity; does not change network state. |
| `led identify [ms]` | Fast blink for locating this tracker | No | Defaults to `TRACKER_STATUS_LED_IDENTIFY_DEFAULT_MS`. |
| `led test <mode>` | Force a status pattern | No | Modes include `normal`, `wifi`, `server`, `connection_error`, `sensor_error`, `hardware_error`. |
| `led reset` | Reset LED write counter | No | Diagnostic only. |

The default board mapping is `TRACKER_STATUS_LED_PIN=8` and `TRACKER_STATUS_LED_ACTIVE_LOW=1`, matching common ESP32-C3 SuperMini blue-LED boards. Override those macros for clones or SuperMini Plus RGB/WS2812 variants. LED updates are non-blocking and rate-limited by `TRACKER_STATUS_LED_UPDATE_INTERVAL_MS`, so the FIFO path never waits for visible blink timing. Runtime patterns are SlimeVR-style: normal/server-found is a very short heartbeat blink, Wi-Fi connecting is one short blink per second, server discovery/connection error is three long blinks every five seconds, sensor error is two long blinks every five seconds, and hardware error is four long blinks every five seconds.

## Stream/log/output

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `stream off|heartbeat|raw|scaled|quat|debug` | Set serial stream mode | Runtime | Also updates config runtime fields where supported. |
| `stream rate <hz>` | Set stream rate | Runtime | May be saved through config save. |
| `log off|basic|full|start|stop` | Control session-bound machine log | Runtime | Used by host replay/metrics tooling. `log full` also emits `MAGR` raw/calibrated/body magnetometer vectors. TCP is capped at 20 Hz. |
| `log finish` | Stop producers but retain/drain the deferred queue | Runtime | Wait for `LOGSTAT,PIPELINE` queued `0` and equal enqueued/serialized before summary/off. |
| `log rate <hz>` | Set machine-log rate | Runtime | USB accepts 1..200 Hz; TCP accepts 1..20 Hz. A quality fixture uses full 20 Hz. |
| `log header` | Emit LOGVER/LOGFMT header | No | Use before captures intended for replay. |
| `log summary` | Emit compact runtime summary | No | Human/agent diagnostic helper. |
| `log reset` | Reset log counters and deferred pipeline | Runtime | Does not reset firmware runtime. Only the owning session, or USB, may control an active log. |
| `output mode debug` | Select local serial debug output | Runtime/config | Does not affect SlimeVR UDP. |
| `output mode binary` | Return `NOT_IMPLEMENTED` | No | Custom binary backend is still reserved. |
| `output rate <hz>` | Set local serial output rate | Runtime/config | SlimeVR has its own `slime rate <hz>` command. |
| `output start|stop` | Start/stop local serial quaternion output | Runtime/config | Does not start/stop SlimeVR UDP. |


## Tests

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `test status` | Print static-test and runtime-test status | No | Runtime test status is shown when the hook is available. |
| `test static <seconds>` | Low-overhead IMU/FIFO stationary observer | No | Exact fault/timestamp counters, block-aggregated statistics and deferred completion. TCP duration is capped at 900 s. |
| `test runtime <seconds>` | Sampled full firmware runtime/load observer | No | Existing counters remain exact; section timers sample 1/16 loops. TCP duration is capped at 900 s. |
| `test stop` | Stop active static/runtime test | No | Only the owner can stop over TCP; USB may force-stop. Completion prints a compact marker. |
| `test summary static|runtime` | Print one immutable compact `TESTSUM` CSV row | No | Available after completion over USB or the owning TCP session; contains exact measured-window counters without the multi-page report. |
| `test report static|runtime` | Print the retained detailed report | No | Available only after completion and intentionally USB-only. It never runs from the measured sample/loop hot path. |

## Network / SlimeVR

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `remote status` | Print Wi-Fi TCP console state | No | Shows enabled/listening/client/counter state. |
| `remote off` / `remote on` | Disable/enable the TCP diagnostic listener for the current boot | No | Privileged USB-only control; use `remote off` after cable-free diagnostics. |
| `net status` | Print Wi-Fi config/runtime status | No | Shows NVS load state, IP, RSSI, MAC, reconnect counters. |
| `net print` | Print network config | No | Password is not revealed. |
| `net set ssid <ssid> [save]` | Set Wi-Fi SSID | Optional | Use 2.4 GHz SSID for ESP32-C3. |
| `net set pass <password> [save]` | Set Wi-Fi password | Optional | Do not wrap the password in quotes unless quotes are part of the password. |
| `net clear pass [save]` | Clear Wi-Fi password | Optional | For open networks/testing. |
| `net set name <name> [save]` | Set tracker/device name | Optional | Hostname is sanitized for Wi-Fi/DHCP. |
| `net set server <host> [port] [save]` | Configure manual SlimeVR server endpoint | Optional | Accepts dotted IPv4 or DNS hostname. With discovery off, only the resolved endpoint may establish the session; with discovery on, unicast and broadcast handshakes coexist. |
| `net discovery on|off [save]` | Enable/disable UDP discovery | Optional | Default is on. |
| `net enable|disable [save]` | Enable/disable Wi-Fi manager | Optional | Runtime change; `save` persists. |
| `net reconnect` | Restart Wi-Fi connection attempt | No | Non-blocking reconnect. |
| `net scan [visible|hidden] [limit N]` | Blocking Wi-Fi environment scan | No | Developer diagnostic; pauses sensor processing while scan runs. |
| `net save|load|defaults|erase` | Manage network NVS config | Yes/Runtime | Network config is stored separately from main tracker config. |
| `slime status` | Print compact SlimeVR UDP runtime status | No | Shows server state, rotation, tap/battery packet counters, failures, ping, mag flags, signed `last_signal_strength_dbm`, latest temperature and latest battery telemetry. |
| `slime debug` | Print full SlimeVR counters/timestamps | No | Developer view with packet counters, last packet values, battery telemetry state and reconnect-hardening counters. |
| `slime start` | Start SlimeVR UDP runtime | Runtime | Uses prepared quaternion snapshots directly and leaves local serial output off. |
| `slime stop` | Stop SlimeVR output runtime | Runtime | Does not erase saved Wi-Fi/config. |
| `slime reconnect` | Restart SlimeVR discovery/session | Runtime | Useful after server restart or network changes. |
| `slime rate <hz>` | Set SlimeVR `RotationData` rate | Runtime/config | Stored in the existing outputRateHz field for compatibility, but not tied to local serial output. |
| `slime counters reset` | Reset SlimeVR counters | Runtime | Does not restart Wi-Fi. |
| `slime action yaw|full|mounting|pause` | Send SlimeVR UserAction packet 21 | Runtime | Server-side reset/pause action; does not alter local AHRS calibration. |
| `slime tap-action off|yaw|full|mounting|pause [save]` | Map physical tap aggregation to UserAction | Runtime/network config | Defaults to `off`; `save` persists through the transactional network-config store. |
| `battery status` / `bat status` | Print ADC battery monitor state | No | Shows GPIO, raw ADC mV, computed battery voltage/percentage, present/not-present state and read-failure counters. |
| `battery reset` / `bat reset` | Reset battery runtime counters/filter | Runtime | Does not change saved config. Next update resamples GPIO. |


### Wi-Fi remote console

Debug and the explicit ProductionDiag image expose a restricted diagnostic CLI
over TCP. Production and Slim compile the listener out. The port is
`TRACKER_REMOTE_CONSOLE_PORT` (`7777` by default):

```bash
nc <tracker-ip> 7777
```

Remote commands pass an exact fail-closed allowlist before dispatch. Persistent
config/setup/calibration/network changes, reset/reboot and other mutations are
USB-only. TCP supports bounded status/performance inspection plus session-bound
`log` and `test` control; a disconnect aborts those producers without falling
back to USB. Both transports use fixed-size, byte-budgeted output queues. See
`docs/wifi_remote_console.md` for the complete policy and unattended capture.

SlimeVR UDP is independent from the local `output`/`stream` commands. `slime start` leaves serial `Q,...` output off and reads prepared quaternion snapshots directly.

Autostart only depends on network config: if Wi-Fi is enabled and credentials are valid, SlimeVR discovery starts after boot. User-facing first-run setup should normally use:

```text
setup wifi
```

That command scans visible networks, asks for a numbered selection and password, connects, saves successful credentials to NVS, starts SlimeVR discovery, and leaves Wi-Fi/SlimeVR autostart enabled. The lower-level `net set ...`, `net scan`, and `slime ...` commands remain available for diagnostics and scripting.

`SensorInfo.sensor_config` advertises magnetometer support/enabled state. The firmware intentionally does not send periodic dummy `MagnetometerAccuracy` packets; packet 18 is only sent if a real mag-calibration/accuracy workflow starts using it.

## Live performance and motion diagnostics

These commands are compiled only when `TRACKER_ENABLE_RUNTIME_PROFILER=1`
(Debug and ProductionDiag). Production and Slim exclude the profiler/motion
source files. Profiler/motion activation is USB-only; the restricted TCP console
may inspect their bounded status without mutating them.

FIFO tuning behavior: apply `fifo watermark` while the tracker is stationary. A successful live hardware reconfigure intentionally enters controlled recovery, so output resumes after the short stationary tilt capture. A failed apply or NVS save restores the previous config and hardware settings.

Safety behavior: `perf` and `motion` are runtime-only diagnostics. `motion` does not touch the per-sample hot path until `motion on` is issued, and missing diagnostic pointers are treated as unavailable commands/status rather than as a boot failure.

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `perf on` / `perf off` | Enable/disable rolling loop-section profiler | Runtime | Off by default; enabling resets the timing window. |
| `perf status` | Print loop-section timing plus temperature/system, Wi-Fi, quality and SlimeVR counters | No | Shows all measured sections (`loop`, `cli`, `remote`, `fifo`, `battery`, `network_outer`, `network_nested`, `calibration_0022`, `calibration_0023`, `runtime_bias_deferred`, `tap`, `led`, `heartbeat`, `idle_yield`), fixed-memory p50/p95/p99, 10 ms frame headroom, profiler overhead, software-only queue/prepared/rotation age, optional-service admission skips, and 1/64 sampled IMU stage timings. Hardware-FIFO residence is not folded into the software-age fields. |
| `perf top` | Print highest average/max section summary plus correlation blocks | No | Shortcut for TPS-drop triage. |
| `perf tracking reset` | Capture a compact, non-destructive FIFO/tracking baseline | Runtime | Does not reset timestamp reconstruction, quality state, recovery state or transport counters. |
| `perf tracking` / `perf brief` | Print compact deltas and effective rotation rate | No | Before the first reset it reports counters since boot; after reset it reports only the selected test window. Intended for low-intrusion hardware validation. |
| `perf reset` | Reset profiler window and tracking baseline | Runtime | Does not reset firmware quality/timestamp state. |
| `motion on` / `motion off` | Enable/disable per-sample motion diagnostics | Runtime | Intended for ankle/fast-motion tests. Exact quality/event counters still observe every sample; expensive aggregate metrics are sampled and `motion_metric_samples`/`motion_sample_divisor` expose that cadence. |
| `motion status` | Print motion window plus FIFO/quality/bias/SlimeVR correlation | No | Reports dt, sample rate, gyro/accel norms, saturation, accel outliers, AHRS skips and runtime-bias rejects. SlimeVR packet counters are printed both as absolute totals and as deltas/rates since `motion on`/`motion reset`, so `slime_rotation_sent_rate_hz` is a window rate rather than a boot-total rate. |
| `motion reset` | Reset motion diagnostic window | Runtime | Keeps current enabled/disabled state. |

Recommended ankle test sequence:

```text
perf on
motion on
# capture once while TPS is normal
perf status
motion status
# capture again when TPS drops
perf status
motion status
slime debug
```


## Calibration storage commands

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `config slots` / `config nvs` | Print both active slots, selector, generations, signatures, quality, boot load/degraded state, legacy/candidate state and transaction counters | No | Does not change the selected config. |
| `config verify` | Read-only verification of the authoritative slot/selector/commit marker | No | Does not migrate, repair, apply runtime state or clear a degraded-storage write latch. Use `config load` for authoritative recovery. |
| `config migrate` | Explicitly migrate a valid legacy `cfg` blob when no committed dual-slot generation exists | Yes | Never overwrites an existing authoritative dual-slot config; otherwise only performs stale legacy cleanup. |
| `cal status` | Print saved calibration validity plus the full autonomy status block | No | Read-only convenience summary. |
| `cal autonomy status` | Print 0022/0023 enable state, lifecycle, evidence, probation, rollback, storage error, motion-sleep blocker/reason and workload counters | No | Read-only. |
| `cal autonomy 0022 on\|off [save]` | Enable/disable background `magToImu` learning from wave 0022 | Optional | Without `save`, applies only until reboot. Disabling removes only an owned 0022 background candidate. |
| `cal autonomy 0023 on\|off [save]` | Enable/disable safe background calibration lifecycle | Optional | Disabling an active provisional promotion performs rollback before turning autonomy off. |
| `cal autonomy rollback` | Roll back the current provisional autonomy promotion | Yes | Available only when a valid write-ahead rollback journal exists. |
| `cal autonomy reset` | Clear RAM-only evidence and independent sessions | No | Active calibration and NVS are unchanged. |
| `cal autonomy clear_rejections` | Clear bounded rejected-candidate memory | Yes | Does not change active calibration. |
| `cal erase_all confirm` | Completely erase saved calibration state | Yes | Works even from `suspended_storage` or with an obsolete/corrupt autonomy journal. A verified erase-recovery marker is written first; boot completes the operation after a power loss. The command installs a calibrationless authoritative config preserving ordinary Wi-Fi/output policy, then removes journal/rejection metadata and the marker last. Explicit confirmation is required. |
| `cal candidate status` | Print candidate metadata, signature, quality and comparison reasons | No | Reads RAM candidate first, then NVS. |
| `cal candidate stage [manual|setup|background] [flush|force]` | Capture current runtime config/calibration as an inactive candidate | RAM; optional NVS | Does not change active tracking. |
| `cal candidate flush [force]` | Persist the staged candidate | Yes | Normal flush obeys quality and minimum-write-interval gates. |
| `cal candidate compare` | Compare candidate with the selected active calibration revision | RAM metadata | New v3 candidates use calibration-model-only freshness, so policy/evidence/timestamp saves do not stale them. Stored v2 candidates retain the old metadata-inclusive revision contract; legacy v1 candidates retain generation-based freshness. |
| `cal candidate discard` | Remove candidate from RAM and NVS | Yes | Active slots are untouched; persistent discard is blocked while storage is degraded or an authoritative apply is pending. |
| `cal candidate promote [force]` | Compose calibration fields onto current active config, apply them without IMU/FIFO restart, then select and mark the prepared slot committed | Yes | `force` bypasses quality only; signature, stale-calibration, structural and `already_promoted` checks remain mandatory. |

See [calibration_storage.md](calibration_storage.md) for the power-loss and
promotion contracts.

## Bias and tests

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `bias status` | Print runtime gyro-bias estimator state | No | Inspection only. |
| `bias on|off` | Enable/disable runtime bias estimator | Runtime | No NVS write by itself. |
| `bias reset` | Clear runtime trim/estimator state | Runtime | Does not clear saved calibration. |
| `test static <seconds>` | Start non-blocking static test | Runtime | FIFO/AHRS/CLI continue; statistics merge in bounded blocks. |
| `test runtime <seconds>` | Start sampled full loop/network runtime test | Runtime | Measures Wi-Fi/SlimeVR/FIFO/quality deltas without permanent loop timers. |
| `test status` | Print static/runtime test status | No | Inspection only. |
| `test stop` | Stop current static/runtime test | Runtime | Leaves an immutable completed snapshot when usable samples exist. |
| `test summary static|runtime` | Print compact immutable E1 completion evidence | No | Remote-safe after completion; exact measured-window counters. |
| `test report static|runtime` | Print retained detailed result | No | Run after the measured window; USB-only. |

## Guided setup commands

`setup` is the user-facing first-run layer. It intentionally exposes only the compact production path; low-level `net`, `cal`, `mag`, and `test` commands remain available for service diagnostics, but the old manual `setup rest/accel/mag/axis/temp` wrappers are not part of the public setup CLI.

| Command | Effect | Notes |
|---|---|---|
| `setup guide` | Print the first-run sequence | Does not modify state. |
| `setup status` | Print readiness checklist and next step | Includes 6DoF, mag-yaw, temp model, Wi-Fi and SlimeVR readiness. |
| `setup wifi` | Interactive Wi-Fi provisioning | Scans visible networks, asks for a network number and password, tries to connect, saves successful credentials to NVS, starts SlimeVR discovery and enables Wi-Fi/SlimeVR autostart. If Wi-Fi connects but the server is not found within the setup timeout, Wi-Fi remains saved and discovery continues in normal runtime. |
| `setup calibration [resume|full] [nomag|6dof] [axis <bodyX> <bodyY> <bodyZ>]` | Run the guided production calibration | Performs continuous-window rest/gyro with held-out validation, Wi-Fi heat warm-up, leave-one-bin-out gyro-temperature fit, the same six ordinary accel faces with automatic held-out validation, sensor-to-device frame alignment, optional mag hard/soft collection and automatic mag axis inference, then verifies final coherent quaternion/linear acceleration output before saving. No diagonal or precise-angle pose is required. Resume mode skips valid stages. |
| `setup verify` | Verify final runtime quaternion/linear acceleration | No | With the tracker still on any stable ordinary face, checks coherent snapshot availability, quaternion norm/continuity, gravity-removed acceleration residual and FIFO/timestamp/recovery health. Does not alter calibration. |
| `setup frame status` | Print the physical sensor-to-device frame | Shows validity, determinant and all three rotation rows. |
| `setup frame calibrate` | Repeat only physical case-frame alignment | Requires an existing accel calibration, asks for top/+Z and forward/+Y up, validates the proper rotation and saves transactionally. |

`setup calibration` is transaction-safe. Full mode snapshots the current RAM calibration/config, performs all stages in RAM, and writes NVS once after every quality gate passes. Resume mode uses the same rollback semantics per missing stage and commits each completed checkpoint so a later interruption does not discard earlier successful work. If a stage fails or is aborted, setup stops active mag/temp captures, restores the stage snapshot, resets AHRS/mag/runtime-bias transient state, and does not overwrite the previous authoritative calibration. The temperature stage no longer depends on `test static`; it uses a dedicated setup capture that commits only short contiguous stationary windows from the normal FIFO pipeline. Brief touches, constant slow rotation, excessive vibration, timestamp/FIFO faults or accel instability reject only the current candidate window; accepted temperature-bin progress is retained and capture resumes automatically. High-rate sensor noise is judged by a hard vibration ceiling plus the statistical precision and cross-window consistency of the mean, so a quiet surface is required but unrealistically noiseless individual 960 Hz samples are not. Type `q` then Enter to abort any long guided collection safely.

During the magnetometer motion stage, setup records gyro/mag motion intervals and uses them to validate the signed axis permutation before enabling mag yaw. The hard/soft-iron collector uses a deterministic Algorithm-R reservoir over the complete accepted capture, so a long late sweep cannot replace the whole fit set with only the newest orientation. Coverage gates are computed from that exact bounded fit set; full-capture extrema remain diagnostics only. Guided axis intervals use a second fixed-memory stratified reservoir keyed by dominant gyro axis and train/validation window parity. It continues admitting and replacing intervals after 160 candidates instead of freezing on the first 160, while a 75 ms admission cadence prevents near-duplicate samples from dominating. Diagnostics expose candidate count, replacements/skips, stored/seen windows, excited axes, axes confirmed independently in both train/validation partitions, and all six bucket counts. Failure output separates full-capture and fit-set spans/norms and reports `mag_cal_failure_reason`; finite rejected fits also report `last_fit_quality` as algebraic RMS, normalized geometric RMS, directional coverage, inlier ratio, axis ratio and box coverage, plus `last_fit_quality_limits` in the same gate order. A large skip count is expected after reservoir activation and is not itself a quality failure.

`setup status` reports:

- `production_ready`
- `tracking_6dof_ready`
- `mag_yaw_ready`
- `temp_model_ready`
- `slimevr_ready`
- per-block statuses for `config`, `wifi`, `rest_gyro`, `accel_6pos`, `mag_driver`, `mag_hard_soft`, `mag_axis`, `temperature_model`, and `slimevr_runtime`
- `rest_calibration_sent_to_slimevr`
- `slimevr_server_found`
- `mag_yaw_apply_enabled`

When a block is missing, `setup status` points back to the simple production path (`setup wifi` or `setup calibration`). `tracking_6dof_ready` now requires a valid sensor-to-device frame in addition to gyro and accel calibration. The readiness report does not modify config, NVS, AHRS, FIFO, or calibration state.

SlimeVR `SensorInfo.hasCompletedRestCalibration` is driven by the local rest/gyro calibration state. Before a valid gyro bias exists, the firmware reports `false`; after `setup calibration` or another valid gyro calibration save, it reports `true` and requests a SensorInfo refresh.

### SlimeVR telemetry notes

- Signal-strength telemetry is packet type 19 and uses one signed dBm byte. `-68 dBm` is sent as two's-complement `0xBC`, not as a normalized 0..100 quality percentage. `slime status` prints `last_signal_strength_dbm`.
- Temperature telemetry is sent with SlimeVR UDP packet type 20 (`sensorId + f32 temperatureC`). The server parser accepts it, but not every GUI view exposes it. Use `slime status` fields `temperature_sent`, `last_temperature_valid`, and `last_temperature_c` to verify firmware-side emission.
- Battery telemetry is sent with SlimeVR UDP packet type 12 (`f32 voltage + f32 percentage`). The RC1 ADC backend expects `BAT+ -> R_TOP -> GPIO -> R_BOTTOM -> GND`, defaults to GPIO4 and 180 kΩ / 180 kΩ, and maps 3.30 V to 0% and 4.20 V to 100%. GPIO4 is ESP32-C3 ADC1_CH4, so no ADC2 force-use path is needed. The firmware reads it sparsely (`TRACKER_BATTERY_ADC_SAMPLE_INTERVAL_MS`, default 10000 ms), uses a small median-filtered burst (`TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT`, default 3), rejects implausible ADC millivolts, applies EMA filtering, and rejects impossible voltage steps. If the divider reads below `TRACKER_BATTERY_PRESENT_MIN_VOLTAGE`, the runtime treats the battery as absent and still reports safe 0.000 V / 0.0% telemetry.
- Magnetometer support is advertised through `SensorInfo.sensorConfig`: bit 1 = supported, bit 0 = enabled. When mag support is enabled from firmware config, `sensor_config` should be `0x3`; `0x1` is interpreted by the current server as `Mag not supported`.

### SlimeVR incoming UDP packet handling

The firmware handles normal server-to-tracker UDP packets with the SlimeVR 12-byte header (`type:u32be + packetNumber:u64be + payload`). The raw discovery response remains a special case: `0x03 + "Hey OVR =D 5"`. SensorInfo acknowledgement is a second special case: exactly six bytes (`type:u32be + sensorId:u8 + sensorStatus:u8`) with no packet number.

The firmware currently handles the server-to-tracker packets needed for a normal UDP SlimeVR session:

- special packet `15` SensorInfo acknowledgement: advances `dirty`/`waiting_for_ack`/`acknowledged` state only when sensor ID and status match the latest sent state; matching acknowledgement stops resend.
- packet `0`/`1` HeartBeat: counted and answered with tracker heartbeat packet `0`.
- packet `10` PingPong: reads `pingId:u32be` and echoes it with tracker packet `10`, so the server can compute ping instead of showing a timeout placeholder.
- packet `22` FeatureFlags: non-empty bitsets advance explicit negotiation state. Server bit 0 enables packet-100 bundles; packet 23 is not inferred from that bit and remains disabled by default.
- packet `25` SetConfigFlag: reads `sensorId:u8`, `configType:u16be`, `state:u8`. For config type `0x0001` the firmware transactionally applies and persists the magnetometer/yaw state, verifies success, marks SensorInfo dirty and only then sends packet `24`. Repeated identical commands are acknowledged without another NVS write.
- packet `200` ProtocolChange: length-validated and stored for diagnostics, then intentionally ignored. The firmware stays on UDP protocol v22.

Use `slime status` or `slime debug` to inspect `sensor_info_sync_state`, `feature_negotiation_state`, `manual_server_*`, `ping_received`, `pong_sent`, `feature_flags_received`, `set_config_flag_*`, `ack_config_*`, `malformed_datagram_length`, the remaining `malformed_*` counters, `user_action_*`, `protocol_change_ignored`, and `unknown_packets_received`. Only complete validated packets from the selected server endpoint refresh session liveness.

UDP TX-pressure diagnosis is exposed as `tx_backoff_drops`, `tx_pressure_failures`, `tx_other_failures`, `tx_failure_window_trips`, `udp_transport_rebind_requests`, `udp_transport_rebind_successes`, `udp_transport_rebind_failures`, `udp_full_reopen_escalations`, and `last_udp_send_error`. A local rebind preserves the selected server and negotiated bundle mode; full escalation restarts discovery. `perf tracking` and `motion status` report window deltas for the same recovery path.

The firmware sends its FeatureFlags after server discovery. When the server
returns bit 0, valid timestamp-coherent rotation and linear acceleration are
serialized as packet-100 inner packet 17 followed by packet 4, preserving
float32 values and one-datagram coherence. Without that capability, rotation
stays at the configured pose rate while packet-4 acceleration is bounded to
50 Hz. Packet 23 remains available only through an explicit compile-time
experimental override and `packet23_enabled=no` is the normal status. Hard-invalid
acceleration falls back to rotation-only packet 17. Dynamic accel-norm outliers
disable AHRS gravity correction but remain valid motion output. Protocol 22
states that acceleration uses the corrected device frame shared with quaternion
(`+X right, +Y forward, +Z top/outward`), so the server does not apply its legacy
acceleration-only -90 degree local-Z correction. `slime status`, `slime debug`,
`perf tracking`, and `motion status` expose mode, negotiation and skip counters.
SensorInfo acknowledgement, endpoint/liveness validation, FeatureFlags negotiation, persistent SetConfigFlag acknowledgement and UserAction sending are active. ProtocolChange application is intentionally unsupported rather than an unfinished state transition.

## SlimeVR Server serial compatibility

These aliases are for first-run provisioning from SlimeVR Server's Serial Console / Setup Wizard. They are intentionally kept separate from the lower-level `net`, `slime`, and `setup` commands so the firmware can later slim developer diagnostics without breaking server provisioning.

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `SET WIFI <ssid> <password>` | Set Wi-Fi credentials, enable Wi-Fi/discovery, save to network NVS, restart Wi-Fi and SlimeVR discovery | Yes | The tokenizer accepts quoted SSID/password values, e.g. `SET WIFI "My WiFi" "pass with spaces"`. |
| `SET BWIFI <base64_ssid> <base64_password>` | Same as `SET WIFI`, but base64 decoded first | Yes | Compatible with safer provisioning flows that avoid quoting/encoding problems. |
| `GET INFO` | Print SlimeVR-firmware-style tracker, vendor, sensor and battery status lines | No | Battery line uses the ADC runtime; absent/unreadable battery is reported as 0.000 V / 0.0%. |
| `GET CONFIG` | Print SlimeVR-firmware-style build/pin config lines | No | Reports ESP32-C3/LSM6DSV-compatible metadata plus LED, INT and battery ADC/divider pins/values. |
| `GET TEST` | Print a compact sensor smoke-test response | No | Uses current LSM/AHRS/sample counters. |
| `GET WIFISCAN` | Blocking Wi-Fi scan with `[WSCAN]` SlimeVR-style lines | No | Suppresses expected FIFO-recovery console noise for a short grace period after the scan reply. |
| `REBOOT` | Reboot the tracker | No | Mirrors official firmware command name. |
| `FRST` | Factory reset config/network NVS and reboot | Yes | Clears tracker and network config. |
| `DELCAL` | Clear saved IMU/mag calibration state | Yes | Keeps Wi-Fi credentials. |
| `TCAL PRINT|DEBUG|RESET|SAVE` | Compatibility wrappers for temperature-calibration inspection/reset/save | Optional | Temperature-only compatibility path. `SAVE` persists gyro temperature compensation without capturing unrelated runtime output/accel state. `RESET` changes only RAM temperature-comp slope/quality metadata, not the config object saved in NVS. |

SlimeVR provisioning reports the upstream `WiFiReconnectionStatus` values, not Arduino `WL_*`: `SavedAttempt=1` during an ordinary saved-credential boot attempt, `ServerCredAttempt=3` while trying credentials supplied by the server, `Failed=4` after a failed/backoff attempt, and `Success=5` after connection. `SET WIFI` and `SET BWIFI` acknowledge a successful persistent credential update with the exact upstream-compatible `New wifi credentials set, reconnecting` text; the ACK remains immediate and does not block serial handling while waiting for DHCP.

For Connect Trackers compatibility, healthy `GET INFO`/`GET TEST` output keeps `status: 0`; Wi-Fi and server-discovery progress are not encoded in that tracker-health field. Every UDP discovery handshake uses packet number zero, matching the official tracker contract, while established-session packets use the normal monotonic sequence. The `firmware:` field and UDP handshake firmware string use `<feature-version>+build.YYYYMMDD`; `GET INFO` additionally prints `Build date: YYYY-MM-DD`, and the native `version` command prints `slimevr_firmware_version` plus `build_date_utc`. PlatformIO derives the date in UTC and honors `SOURCE_DATE_EPOCH` for reproducible builds.

When `SET WIFI` or `SET BWIFI` succeeds, the tracker now forces a fresh SlimeVR discovery and `SensorInfo` registration even if it was already connected and the same credentials were submitted. The persistent candidate is committed before live Wi-Fi/session state changes; a failed save leaves the working connection unchanged. This intentional session restart lets the server's Connect trackers flow observe an already connected device without requiring a reboot or a physical link drop.

Blocking server/diagnostic commands such as Wi-Fi scans can intentionally pause sensor processing long enough to cause FIFO recovery on the next loop. During the configured grace window (`TRACKER_SERIAL_COMMAND_RECOVERY_SUPPRESS_MS`, default 3000 ms), human-facing `# WARN FIFO recovery requested` and `# TRACKING ... recovery` lines are muted so the command reply remains parseable. Machine-log events, quality counters, FIFO recovery and orientation resets still happen; only console noise is suppressed. Background magnetometer auto-heading reference events are also silent so they cannot interleave with server provisioning replies; manual `mag heading ref` still prints an explicit result. Treat `net scan`/`GET WIFISCAN` as a tracking interruption: movement during the blocking scan is lost, while movement after recovery should resume normally. Verify recovery with `ahrs status`: after a scan, `last_integrated_t_us` should advance again, `bad_dt_rejects` should not grow continuously, and `post_fifo_recovery_samples` should increase as new samples are integrated.


## Live diagnostics profile note

The `perf` and `motion` commands are compiled in Debug and in the service
`BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG` environment. They are intentionally absent
from normal Production and Slim so release builds do not carry live diagnostic
code, strings or per-loop instrumentation hooks. Slim also disables both serial
CLI and Wi-Fi remote console; its remaining SlimeVR packet set is RotationData,
PingPong responses, SignalStrength/RSSI, Temperature and BatteryLevel.

Recommended wearable diagnostic build:

```bash
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG -t upload
```

Typical session over serial or telnet:

```text
perf on
motion on
perf status
motion status
```

## Patch 0022 magnetic runtime fields

`mag heading` now prints field-reliability state/flags, trusted-for-yaw, norm/dip
reference errors, heading step/rate, transition/rejection counters, and continuous
axis-candidate coverage/solve/storage state. `mag yaw status` additionally prints
normal versus reacquisition mode, pending/active state, stable-field duration and
bounded correction statistics. Background alignment never needs a new command:
when coverage produces a staged result, inspect and manage it with the existing
`cal candidate status|compare|flush|promote|discard` commands. `flush` and
`promote` remain explicit user operations.

### Hotfix 0022a fields and axis input

`mag heading` additionally reports instantaneous versus filtered heading rate,
changed-environment count, coarse/final/active solver scores, refinement angle,
measured quality, deferred action, service deferrals and solve/storage last/max
microseconds. The filtered rate is the value used by large-error reacquisition.

`mag axis set <x> <y> <z> [save]` still accepts signed axis tokens, but new manual
mappings must form a right-handed proper rotation (`det=+1`). Continuous matrices
produced by setup/background refinement are printed as three matrix rows and are
stored through the normal calibration path. Existing legacy persisted mappings
are not silently invalidated by this hotfix.

### Hotfix 0022b fields

`mag heading` and `mag yaw status` additionally expose stationary heading-window
delta/rate, discontinuity-latch state, training/validation interval and window
counts, validation winner consistency, normalized separation, observable angular
motion, and deferred-service reject reasons. Hardware FIFO unread-word and
rotation-deadline-slack diagnostics explain why a pending solve/stage action was
deferred. No new mutation command is added; candidate lifecycle remains explicit.

## 0023gd magnetic calibration diagnostics

Magnetic status/failure output now distinguishes numerical solver progress from physical quality rejection. Relevant fields include:

```text
last_fit_solver_stage
last_fit_normalization_center
last_fit_normalization_scale
last_fit_solver_pivot_ratio
last_fit_solver_samples
last_fit_quality
last_fit_robust_refit_passes
last_fit_robust_threshold_factor
last_fit_quality_limits
gyro_endpoint_valid
gyro_endpoint_t_us
gyro_endpoint_skew_us
```

Guided alignment additionally reports:

```text
dynamic_axis_gyro_skew_rejected
dynamic_axis_timing_rejected
dynamic_axis_motion_rejected
dynamic_axis_excited_axes
dynamic_axis_partition_confirmed_axes
dynamic_axis_bucket_counts
```

A numerical failure should be diagnosed from `mag_cal_failure_reason`, solver stage, normalization and pivot ratio. A finite fit rejected by a physical gate should be diagnosed from `last_fit_quality` against `last_fit_quality_limits`; the first limit is the effective algebraic numerical backstop (normally 2.2 times the geometric limit), not a second stricter physical residual gate. `last_fit_robust_refit_passes` reports bounded inlier refits and `last_fit_robust_threshold_factor` reports the actual final normalized radial threshold selected after applying the configured floor, sigma estimate, and quality cap. Do not collect indefinitely when the final `inlier_ratio` or geometric residual indicates a disturbed/non-ellipsoidal environment rather than missing coverage.

## 0023ge FIFO magnetic callback diagnostics

`status` adds:

```text
fifo_runtime_mag_count_deferrals
fifo_runtime_mag_budget_deferrals
```

`perf tracking` adds window deltas:

```text
runtime_mag_count_deferrals_delta
runtime_mag_budget_deferrals_delta
```

A count deferral means the bounded per-pass mag callback allowance was reached.
A budget deferral means the cooperative callback-time budget was reached. In
both cases the raw timeline remains parked at the last coherent gyro endpoint
until the due magnetic backlog is serviced.

## 0023gg magnetic timestamp and setup verification diagnostics

FIFO, magnetic status, and runtime status reports include `mag_timestamp_imu_anchors`, `mag_timestamp_nominal_fallbacks`, `mag_timestamp_monotonic_adjustments`, `mag_timestamp_last_anchor_correction_us`, and `mag_timestamp_max_anchor_correction_us`. Guided mag-axis diagnostics distinguish `gyro_mag_axis_coarse_winner_matches_training`, `gyro_mag_axis_continuous_refinement_agreement`, and `gyro_mag_axis_coarse_consensus_fallback_used`. Final setup verification prints `capture_duration_ms` plus the individual `stationary_input_sample_count_passed`, `stationary_gyro_mean_passed`, `stationary_gyro_precision_passed`, `stationary_accel_mean_passed`, and `stationary_accel_std_passed` gates.

## pre-0024a deadline and deferred magnetic evidence diagnostics

`perf tracking` adds:

```text
runtime_fifo_urgent_by_depth
runtime_fifo_urgent_by_age
runtime_fifo_raw_callback_sample_divisor
runtime_fifo_raw_callback_avg_us
runtime_fifo_raw_callback_max_us
runtime_fifo_mag_callback_avg_us
runtime_fifo_mag_callback_max_us
runtime_fifo_slice_budget_overshoot_delta
runtime_fifo_slice_budget_overshoot_max_us
runtime_mag_axis_evidence_queued_delta
runtime_mag_axis_evidence_processed_delta
runtime_mag_axis_evidence_dropped_delta
runtime_mag_axis_evidence_stale_dropped_delta
runtime_mag_axis_evidence_service_deferrals_delta
runtime_mag_axis_evidence_queue_high_water
```

Depth urgency means the legacy software-queue depth threshold was reached. Age urgency means the retained sensor-timestamp span reached 40 ms even though depth may still be below that threshold. Overshoot is measured after the mandatory four-sample coherent micro-batch and should remain near one raw/magnetic callback cost, not a twelve-callback burst. Axis evidence overflow or stale drops affect only uncommitted background alignment learning; they must remain zero in an ordinary single-tracker run. `runtime_mag_axis_evidence_service_deferrals_delta` may increase while FIFO/output deadlines are busy; this proves evidence work was postponed instead of competing with tracking.

## pre-0024ab overload-age and transform-cache diagnostics

The existing `perf status` and `perf tracking` software-age percentile fields now remain finite and readable when queue age exceeds 100 ms. Histogram bounds extend through 2 seconds; samples beyond the final finite bound report the actual observed maximum rather than `4294967295`. This changes diagnostics only. Sensor timestamps, queue contents, output cadence and packet payloads are unchanged.

### pre-0024ac hotpath/slack fields

`perf status` additionally reports `perf_optional_service_admission_skips_{battery,led,mag_deferred,calibration_autonomy,remote_console}` and five `perf_imu_stage=` rows (`scale_calibration`, `quality`, `ahrs_recovery`, `prepared_output`, `per_sample_outputs`). Stage timings sample one of every `perf_imu_stage_sample_divisor` IMU samples and are diagnostic only. Admission-skip counters mean the ordinary-loop call was withheld because FIFO work was pending/urgent or insufficient rotation-deadline slack; they do not indicate dropped IMU samples or changed AHRS cadence.


## pre-0024ad network-pressure diagnostics

`slime status`, `slime debug`, `perf tracking`, `perf status`, `motion status`, and the runtime test report expose the pressure episode and physical datagram split:

```text
tx_pressure_state
tx_recovery_reason
tx_pressure_episode_active
tx_pressure_episode_count
tx_pressure_episode_duration_ms
tx_pressure_episode_max_ms
tx_pressure_stable_resets
last_successful_motion_tx_age_ms
successful_motion_tx_streak
udp_rebind_suppressed_cooldown
udp_full_reopen_suppressed_cooldown
physical_datagrams_sent
motion_datagrams_sent
separate_rotation_datagrams_sent
separate_acceleration_datagrams_sent
background_control_datagrams_sent
critical_control_datagrams_sent
motion_packet_mode_transitions
bundle_to_separate_transitions
separate_to_bundle_transitions
acceleration_suppressed_during_negotiation
rotation_phase_offset_ms
```

`tx_pressure_state=transient_pressure` with continuing successful motion is not a broken socket. Rebind should remain rare; `full_reopen` should be near zero in a healthy multi-tracker session. `last_successful_motion_tx_age_ms` is `4294967295` only before the first successful motion datagram. A rising `acceleration_suppressed_during_negotiation` is expected only during bounded reconnect capability negotiation and indicates packet 4 was withheld to avoid a temporary 150-datagram/s fallback.

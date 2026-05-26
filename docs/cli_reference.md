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
| `version` | Print CLI protocol marker | No | Lightweight sanity check. |
| `reboot` | Restart ESP32 | No | Flushes output before restart. |
| `factory_reset` | Reset runtime config defaults and erase config store | Yes | Reboot recommended after success. |
| `remote status` | Print Wi-Fi TCP console state | No | Available when `TRACKER_ENABLE_WIFI_REMOTE_CONSOLE=1`. |
| `remote off` / `remote on` | Stop/start the Wi-Fi TCP console for the current boot | No | `remote off` closes the TCP client/server so it stops adding normal-loop work. |

## Config

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `config print` | Print active config | No | Inspection only. |
| `config crc` | Print active config CRC | No | Inspection only. |
| `config nvs` | Inspect config store | No | Reads store metadata/status. |
| `config defaults` | Reset active config to defaults | No | Does not erase NVS until save/erase. |
| `config load` | Load config from NVS | Runtime | Applies loaded values to runtime where supported. |
| `config save` | Capture supported runtime values and save | Yes | Saves calibration/output/runtime settings captured by config layer. |
| `config erase` | Erase config store | Yes | Reboot recommended. |
| `config spi <hz> [save]` | Live SPI clock reconfigure | Optional | Uses app hook; `save` persists configured frequency. |

## IMU/FIFO/quality

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `imu status` | Print IMU config/status | No | Inspection only. |
| `imu whoami` | Print last WHOAMI | No | Hardware sanity check. |
| `imu read` | Read one direct sample | No | Debug path, not FIFO runtime. |
| `imu rate <120|240|480|960> [save]` | Live IMU+FIFO ODR reconfigure | Optional | Stops stream/log during reconfigure and re-arms mag if needed. |
| `fifo status` | Print FIFO status | No | Inspection only. |
| `fifo stats` | Print FIFO counters | No | Inspection only. |
| `fifo watermark <words> [save]` | Live FIFO watermark reconfigure | Optional | Re-arms runtime/FIFO path. |
| `fifo drain <max_words> <rounds> [save]` | Live drain limits reconfigure | Optional | Affects bounded drain behavior. |
| `fifo reset` | Reset FIFO/runtime counters/path | No | Uses app reset hook when available. |
| `quality stats` | Print quality counters | No | Inspection only. |
| `quality reset` | Reset quality counters | No | Runtime only. |

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
| `tap test [2..10]` | Send one manual SlimeVR Tap packet directly | No | Verifies the server path without using the LSM6DSV detector or accumulator. Defaults to `2`. |
| `tap inject <1..10>` | Emulate physical tap events through the accumulator | No | Useful for checking sliding-window aggregation without physically tapping the tracker. A single injected tap is intentionally suppressed by the default min count. |
| `tap reset` | Reset tap counters | No | Does not reset SlimeVR counters. |

LSM6DSV tap recognition is routed through the same physical INT1 line as FIFO events; INT2 is not required. The ISR remains lightweight, and the runtime polls/clears `TAP_SRC` from the normal loop. Hardware double-tap is disabled by default so the sensor reports physical taps one-by-one; firmware then aggregates 2..10 taps in a sliding `TRACKER_TAP_AGGREGATION_WINDOW_MS` window and sends one SlimeVR Tap packet. The driver enables `FUNCTIONS_ENABLE.INTERRUPTS_ENABLE` with a masked write and verifies it together with the tap registers; this preserves the FIFO timestamp bit in the same register. Register verification is performed after configuration and then only every `TRACKER_TAP_REGISTER_VERIFY_INTERVAL_MS` while no tap window is pending, so FIFO timing is not burdened by frequent config reads. The default tap threshold is intentionally moderate (`TRACKER_LSM6DSV_TAP_THRESHOLD=4`) so hand taps can be detected during RC1 tuning; raise it if the enclosure produces false positives.

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
| `log off|basic|full|start|stop` | Control machine-readable log | Runtime | Used by host replay/metrics tooling. `log full` also emits `MAGR` raw/calibrated/body magnetometer vectors. |
| `log rate <hz>` | Set machine-log rate | Runtime | Runtime only. |
| `log header` | Emit LOGVER/LOGFMT header | No | Use before captures intended for replay. |
| `log summary` | Emit compact runtime summary | No | Human/agent diagnostic helper. |
| `log reset` | Reset log counters | Runtime | Does not reset firmware runtime. |
| `output mode debug` | Select local serial debug output | Runtime/config | Does not affect SlimeVR UDP. |
| `output mode binary` | Return `NOT_IMPLEMENTED` | No | Custom binary backend is still reserved. |
| `output rate <hz>` | Set local serial output rate | Runtime/config | SlimeVR has its own `slime rate <hz>` command. |
| `output start|stop` | Start/stop local serial quaternion output | Runtime/config | Does not start/stop SlimeVR UDP. |


## Tests

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `test status` | Print static-test and runtime-test status | No | Runtime test status is shown when the hook is available. |
| `test static <seconds>` | Existing IMU/FIFO stationary test | No | Best for calibration/stability of IMU path, not full Wi-Fi load. |
| `test runtime <seconds>` | Full firmware runtime/load test | No | Measures loop/CLI/FIFO/network/heartbeat timing plus Wi-Fi/SlimeVR/FIFO/quality deltas. |
| `test stop` | Stop active static/runtime test | No | Requests stop; report is printed by the runner. |

## Network / SlimeVR

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `remote status` | Print Wi-Fi TCP console state | No | Shows enabled/listening/client/counter state. |
| `remote off` / `remote on` | Disable/enable the TCP CLI console for the current boot | No | Use `remote off` after cable-free setup/calibration. |
| `net status` | Print Wi-Fi config/runtime status | No | Shows NVS load state, IP, RSSI, MAC, reconnect counters. |
| `net print` | Print network config | No | Password is not revealed. |
| `net set ssid <ssid> [save]` | Set Wi-Fi SSID | Optional | Use 2.4 GHz SSID for ESP32-C3. |
| `net set pass <password> [save]` | Set Wi-Fi password | Optional | Do not wrap the password in quotes unless quotes are part of the password. |
| `net clear pass [save]` | Clear Wi-Fi password | Optional | For open networks/testing. |
| `net set name <name> [save]` | Set tracker/device name | Optional | Hostname is sanitized for Wi-Fi/DHCP. |
| `net set server <host> [port] [save]` | Configure manual SlimeVR server endpoint | Optional | Discovery remains available when manual server is disabled. |
| `net discovery on|off [save]` | Enable/disable UDP discovery | Optional | Default is on. |
| `net enable|disable [save]` | Enable/disable Wi-Fi manager | Optional | Runtime change; `save` persists. |
| `net reconnect` | Restart Wi-Fi connection attempt | No | Non-blocking reconnect. |
| `net scan [visible|hidden] [limit N]` | Blocking Wi-Fi environment scan | No | Developer diagnostic; pauses sensor processing while scan runs. |
| `net save|load|defaults|erase` | Manage network NVS config | Yes/Runtime | Network config is stored separately from main tracker config. |
| `slime status` | Print compact SlimeVR UDP runtime status | No | Shows server state, rotation, tap/battery packet counters, failures, ping, mag flags, RSSI, latest temperature and latest battery telemetry. |
| `slime debug` | Print full SlimeVR counters/timestamps | No | Developer view with packet counters, last packet values, battery telemetry state and reconnect-hardening counters. |
| `slime start` | Start SlimeVR UDP runtime | Runtime | Uses prepared quaternion snapshots directly and leaves local serial output off. |
| `slime stop` | Stop SlimeVR output runtime | Runtime | Does not erase saved Wi-Fi/config. |
| `slime reconnect` | Restart SlimeVR discovery/session | Runtime | Useful after server restart or network changes. |
| `slime rate <hz>` | Set SlimeVR `RotationData` rate | Runtime/config | Stored in the existing outputRateHz field for compatibility, but not tied to local serial output. |
| `slime counters reset` | Reset SlimeVR counters | Runtime | Does not restart Wi-Fi. |
| `battery status` / `bat status` | Print ADC battery monitor state | No | Shows GPIO, raw ADC mV, computed battery voltage/percentage, present/not-present state and read-failure counters. |
| `battery reset` / `bat reset` | Reset battery runtime counters/filter | Runtime | Does not change saved config. Next update resamples GPIO. |


### Wi-Fi remote console

Debug and Production expose the same CLI over TCP by default. The port is
`TRACKER_REMOTE_CONSOLE_PORT` (`7777` by default):

```bash
nc <tracker-ip> 7777
```

Use it for cable-free `setup calibration`, then run `remote off` to close the
TCP client/server for the current boot. During `setup calibration`, the
firmware keeps an already connected Wi-Fi link instead of forcing a reconnect
at the temperature stage, so the TCP console should stay attached. Slim
compiles this feature out. See `docs/wifi_remote_console.md` for details.

SlimeVR UDP is independent from the local `output`/`stream` commands. `slime start` leaves serial `Q,...` output off and reads prepared quaternion snapshots directly.

Autostart only depends on network config: if Wi-Fi is enabled and credentials are valid, SlimeVR discovery starts after boot. User-facing first-run setup should normally use:

```text
setup wifi
```

That command scans visible networks, asks for a numbered selection and password, connects, saves successful credentials to NVS, starts SlimeVR discovery, and leaves Wi-Fi/SlimeVR autostart enabled. The lower-level `net set ...`, `net scan`, and `slime ...` commands remain available for diagnostics and scripting.

`SensorInfo.sensor_config` advertises magnetometer support/enabled state. The firmware intentionally does not send periodic dummy `MagnetometerAccuracy` packets; packet 18 is only sent if a real mag-calibration/accuracy workflow starts using it.

## Bias and tests

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `bias status` | Print runtime gyro-bias estimator state | No | Inspection only. |
| `bias on|off` | Enable/disable runtime bias estimator | Runtime | No NVS write by itself. |
| `bias reset` | Clear runtime trim/estimator state | Runtime | Does not clear saved calibration. |
| `test static <seconds>` | Start non-blocking static test | Runtime | FIFO/AHRS/CLI continue running. |
| `test runtime <seconds>` | Start full loop/network runtime test | Runtime | Measures Wi-Fi/SlimeVR/FIFO/quality deltas. |
| `test status` | Print static/runtime test status | No | Inspection only. |
| `test stop` | Stop current static/runtime test | Runtime | Leaves last completed report when available. |

## Guided setup commands

`setup` is the user-facing first-run layer. It intentionally exposes only the compact production path; low-level `net`, `cal`, `mag`, and `test` commands remain available for service diagnostics, but the old manual `setup rest/accel/mag/axis/temp` wrappers are not part of the public setup CLI.

| Command | Effect | Notes |
|---|---|---|
| `setup guide` | Print the first-run sequence | Does not modify state. |
| `setup status` | Print readiness checklist and next step | Includes 6DoF, mag-yaw, temp model, Wi-Fi and SlimeVR readiness. |
| `setup wifi` | Interactive Wi-Fi provisioning | Scans visible networks, asks for a network number and password, tries to connect, saves successful credentials to NVS, starts SlimeVR discovery and enables Wi-Fi/SlimeVR autostart. If Wi-Fi connects but the server is not found within the setup timeout, Wi-Fi remains saved and discovery continues in normal runtime. |
| `setup calibration [axis <bodyX> <bodyY> <bodyZ>]` | Run the full guided production calibration | Performs rest/gyro, Wi-Fi heat warm-up, dedicated gyro temperature capture/fit, auto-detected accel 6-position full 3x3 affine calibration, mag hard/soft collection, automatic mag axis inference using gyro-assisted mag motion plus static accel-face/mag inclination checks, enables accel/adaptive accel, runtime gyro bias and mag-yaw apply, then saves. Axis tokens are now an override/fallback path. |

`setup calibration` is transactional. It snapshots the current RAM calibration/config at start, performs every stage in RAM, and writes to NVS only once after all quality gates pass. If any stage fails or is aborted, the command stops mag/temp captures, restores the previous RAM calibration/config, resets AHRS/mag/runtime-bias transient state, and leaves the previous NVS calibration untouched. The temperature stage no longer depends on `test static`; it uses a dedicated setup capture that records only the temperature-fit bins from the normal FIFO pipeline. During the magnetometer motion stage, setup also records gyro/mag motion intervals and uses them to validate the signed axis permutation before enabling mag yaw.

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

When a block is missing, `setup status` points back to the simple production path (`setup wifi` or `setup calibration`). The readiness report does not modify config, NVS, AHRS, FIFO, or calibration state.

SlimeVR `SensorInfo.hasCompletedRestCalibration` is driven by the local rest/gyro calibration state. Before a valid gyro bias exists, the firmware reports `false`; after `setup calibration` or another valid gyro calibration save, it reports `true` and requests a SensorInfo refresh.

### SlimeVR telemetry notes

- Temperature telemetry is sent with SlimeVR UDP packet type 20 (`sensorId + f32 temperatureC`). The server parser accepts it, but not every GUI view exposes it. Use `slime status` fields `temperature_sent`, `last_temperature_valid`, and `last_temperature_c` to verify firmware-side emission.
- Battery telemetry is sent with SlimeVR UDP packet type 12 (`f32 voltage + f32 percentage`). The RC1 ADC backend expects `BAT+ -> R_TOP -> GPIO -> R_BOTTOM -> GND`, defaults to GPIO4 and 180 kΩ / 180 kΩ, and maps 3.30 V to 0% and 4.20 V to 100%. GPIO4 is ESP32-C3 ADC1_CH4, so no ADC2 force-use path is needed. The firmware reads it sparsely (`TRACKER_BATTERY_ADC_SAMPLE_INTERVAL_MS`, default 10000 ms), uses a small median-filtered burst (`TRACKER_BATTERY_ADC_OVERSAMPLE_COUNT`, default 3), rejects implausible ADC millivolts, applies EMA filtering, and rejects impossible voltage steps. If the divider reads below `TRACKER_BATTERY_PRESENT_MIN_VOLTAGE`, the runtime treats the battery as absent and still reports safe 0.000 V / 0.0% telemetry.
- Magnetometer support is advertised through `SensorInfo.sensorConfig`: bit 1 = supported, bit 0 = enabled. When mag support is enabled from firmware config, `sensor_config` should be `0x3`; `0x1` is interpreted by the current server as `Mag not supported`.

### SlimeVR incoming UDP packet handling

The firmware handles normal server-to-tracker UDP packets with the SlimeVR 12-byte header (`type:u32be + packetNumber:u64be + payload`). It also keeps a legacy raw one-byte fallback for diagnostics. The raw discovery response remains a special case: `0x03 + "Hey OVR =D 5"`.

The firmware currently handles the server-to-tracker packets needed for a normal UDP SlimeVR session:

- packet `0`/`1` HeartBeat: counted and answered with tracker heartbeat packet `0`.
- packet `10` PingPong: reads `pingId:u32be` and echoes it with tracker packet `10`, so the server can compute ping instead of showing a timeout placeholder.
- packet `22` FeatureFlags: stored for diagnostics. The current firmware does not enable optional behavior from these flags yet.
- packet `25` SetConfigFlag: reads `sensorId:u8`, `configType:u16be`, `state:u8`. For config type `0x0001` the firmware treats it as the runtime magnetometer/yaw enable toggle, applies it without writing NVS, refreshes `SensorInfo`, and sends packet `24` AckConfigChange.
- packet `200` ProtocolChange: stored for diagnostics only. The firmware stays on UDP protocol v19.

Use `slime status` to inspect `ping_received`, `pong_sent`, `feature_flags_received`, `set_config_flag_*`, `ack_config_sent`, `protocol_change_received`, and `unknown_packets_received`.

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

Blocking server/diagnostic commands such as Wi-Fi scans can intentionally pause sensor processing long enough to cause FIFO recovery on the next loop. During the configured grace window (`TRACKER_SERIAL_COMMAND_RECOVERY_SUPPRESS_MS`, default 3000 ms), human-facing `# WARN FIFO recovery requested` and `# TRACKING ... recovery` lines are muted so the command reply remains parseable. Machine-log events, quality counters, FIFO recovery and orientation resets still happen; only console noise is suppressed. Background magnetometer auto-heading reference events are also silent so they cannot interleave with server provisioning replies; manual `mag heading ref` still prints an explicit result. Treat `net scan`/`GET WIFISCAN` as a tracking interruption: movement during the blocking scan is lost, while movement after recovery should resume normally. Verify recovery with `ahrs status`: after a scan, `last_integrated_t_us` should advance again, `bad_dt_rejects` should not grow continuously, and `post_fifo_recovery_samples` should increase as new samples are integrated.

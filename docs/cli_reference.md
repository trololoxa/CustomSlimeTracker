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
| `cal accel compute` | Compute accel 6-position calibration | Runtime | No |
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
| `ahrs status`, `ahrs config` | Print AHRS config/status | No | Inspection only. |
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
| `mag cal start|stop|reset|status|print|apply [save]` | Manage mag calibration collector | Optional | `apply save` persists. |

## Stream/log/output

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `stream off|heartbeat|raw|scaled|quat|debug` | Set serial stream mode | Runtime | Also updates config runtime fields where supported. |
| `stream rate <hz>` | Set stream rate | Runtime | May be saved through config save. |
| `log off|basic|full|start|stop` | Control machine-readable log | Runtime | Used by host replay/metrics tooling. |
| `log rate <hz>` | Set machine-log rate | Runtime | Runtime only. |
| `log header` | Emit LOGVER/LOGFMT header | No | Use before captures intended for replay. |
| `log summary` | Emit compact runtime summary | No | Human/agent diagnostic helper. |
| `log reset` | Reset log counters | Runtime | Does not reset firmware runtime. |
| `output mode debug` | Select supported debug output | Runtime | Current safe mode. |
| `output mode binary|slimevr` | Return `NOT_IMPLEMENTED` | No | Must stay disabled until real backend exists. |
| `output rate <hz>` | Set output rate | Runtime/config | Used by current output policy. |
| `output start|stop` | Start/stop quaternion output flag | Runtime/config | Serial/debug-oriented until backend exists. |

## Bias and tests

| Command | Effect | Persisted | Notes |
|---|---|---:|---|
| `bias status` | Print runtime gyro-bias estimator state | No | Inspection only. |
| `bias on|off` | Enable/disable runtime bias estimator | Runtime | No NVS write by itself. |
| `bias reset` | Clear runtime trim/estimator state | Runtime | Does not clear saved calibration. |
| `test static <seconds>` | Start non-blocking static test | Runtime | FIFO/AHRS/CLI continue running. |
| `test status` | Print static test status | No | Inspection only. |
| `test stop` | Stop current static test | Runtime | Leaves last completed result if available. |

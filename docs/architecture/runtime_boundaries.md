# Runtime boundaries

## Runtime data flow

### Boot flow

```text
main.cpp
  -> trackerAppContextSetup()
    -> TrackerApp::begin(makeTrackerAppDeps())
    -> TrackerApp::setup()
      -> Serial startup
      -> config load/defaults/sanitize/apply
      -> LSM init
      -> FIFO init
      -> calibration IO setup
      -> mag startup from config
      -> command context wiring
      -> runtime reset/ready
```

### Main loop flow

```text
main.cpp
  -> trackerAppContextLoop()
    -> TrackerApp::loop()
      -> CLI poll
      -> FIFO runtime process
          -> raw IMU samples -> imu_sample_pipeline
          -> raw mag samples -> mag_runtime_controller
      -> CLI poll
      -> heartbeat/output maintenance
```

### IMU sample flow

```text
Lsm6dsvFifoReader::RawSample
  -> ImuSamplePipeline
    -> scaled accel/gyro conversion
    -> gyro bias and temp compensation
    -> accel calibration
    -> quality monitor
    -> AHRS update
    -> runtime bias estimator
    -> static test runner
    -> prepared output / serial stream / machine log
```

### Mag sample flow

```text
LSM sensorhub FIFO mag sample
  -> MagRuntimeController
    -> calibration / axis alignment
    -> trust/reject gates
    -> heading calculation
    -> heading reference / auto-reference
    -> yaw correction gate
    -> optional AHRS yaw correction
    -> status/static-test/machine-log hooks
```

## Persistent vs runtime state

Persisted in NVS:

- hardware/IMU/FIFO defaults;
- AHRS config;
- gyro bias;
- gyro temp compensation model and metadata;
- accel calibration and quality metadata;
- mag calibration and quality metadata;
- mag yaw correction config;
- output policy;
- active sensor-to-device alignment plus neutral compatibility-reserved bytes;
- network/SlimeVR config in separate storage.

Runtime only:

- live quaternion;
- tracking state transitions;
- FIFO/quality/performance counters;
- static test in-progress stats;
- mag heading auto-reference runtime state;
- prepared output snapshot;
- runtime gyro-bias trim;
- recovery state.

Rule of thumb: if a value is derived from the current boot/session, do not persist it unless there is a deliberate calibration/save command.

## Calibration ownership

Calibration is split intentionally:

- math/model structs live in `sensor/`;
- FIFO-compatible capture helpers live in `sensor/fifo_calibrations.hpp`;
- persistent storage lives in `config/`;
- command entrypoints live in `serial/tracker_calibration_commands.hpp`;
- long static-test temp fitting lives in `runtime/gyro_temp_static_fit.hpp`;
- app wiring lives in `app/tracker_app_hooks.hpp`.

Do not put calibration algorithms into CLI files. CLI files should only parse arguments, call the right module/hook, and print results.

## Output and SlimeVR boundary

There are two output paths with separate ownership:

- local serial developer output: `stream ...`, `output ...`, and machine logs;
- SlimeVR UDP output: `net ...` / `slime ...`, using one prepared coherent motion snapshot. A negotiated packet-100 datagram contains float32 packet 17 followed by float32 packet 4; unsupported servers use packet 17 at pose rate plus a bounded 50 Hz packet-4 fallback. Packet 23 is experimental and disabled by default.

The SlimeVR backend is real Wi-Fi/UDP transport, not a fake packet mode. Custom binary output is still not implemented and should remain disabled until it has a real backend.

Current SlimeVR-related modules:

```text
network/wifi_manager.hpp
network/udp_transport.hpp
output/slimevr_packet_writer.hpp
runtime/slimevr_output_runtime.hpp
runtime/status_led_runtime.hpp
```

`src/network/` owns Wi-Fi/UDP transport primitives only. `src/output/` owns packet encoding, and `runtime/slimevr_output_runtime.*` owns discovery/session/output scheduling. `runtime/status_led_runtime.*` owns only GPIO LED pattern timing and status display; it must consume high-level runtime states, not sensor samples. Do not place AHRS/FIFO logic in transport modules, and do not place transport state in AHRS or mag-yaw code.

The firmware should send local sensor/device orientation and health. It should not bake in server/body/mounting calibration semantics unless there is a clear protocol-level reason.

## Sensor calibration implementation split status

The sensor calibration layer now follows the public-header/private-implementation rule:

- `sensor/calibration.hpp` declares IMU calibration, stationary detection, startup gyro calibration, and online gyro bias APIs.
- `sensor/calibration.cpp` implements those algorithms.
- `sensor/fifo_calibrations.hpp` keeps the FIFO drain template in the header but moves the non-template calibration runners to `sensor/fifo_calibrations.cpp`.

This keeps command/config/app code from recompiling the calibration implementation in every translation unit while preserving the templated FIFO drain helper where it belongs.

### RC1 serial provisioning compatibility


Battery telemetry ownership, filtering and failure behavior are specified in the [battery contract](battery.md). Use `battery status` to inspect the local reading during validation.

The firmware now exposes SlimeVR Server serial-compatibility commands for initial provisioning: `SET WIFI`, `SET BWIFI`, `GET INFO`, `GET CONFIG`, `GET TEST`, `GET WIFISCAN`, `REBOOT`, `FRST`, `DELCAL`, and temperature-only `TCAL`. `SET WIFI`/`SET BWIFI` save credentials into the separate network NVS config, enable Wi-Fi/discovery, and restart SlimeVR discovery. Blocking Wi-Fi scans suppress expected FIFO-recovery console noise for a short grace window without disabling recovery, counters, or machine-log events. `TCAL SAVE` is intentionally scoped to gyro temperature compensation so host-side temperature-calibration commands cannot accidentally capture unrelated runtime output/accel state. `TCAL RESET` changes only the in-RAM temperature model, resets the volatile runtime residual gyro trim learned against the previous model, and does not mutate the persistent config object unless a later explicit `TCAL SAVE` is requested.
## FIFO gyro/accel coherency and continuity

The FIFO parser never pairs a stale gyro word with a later accelerometer word.
AHRS gravity-observation validity and linear-acceleration output validity are
separate decisions: dynamic acceleration can be rejected as a gravity reference
while remaining valid packet-4 motion data.
A repeated gyro tag publishes the older gyro as a gyro-only sample so AHRS
prediction and heading continuity continue. Gyro-only samples never use accel
correction, never produce linear acceleration, and never update persistent or
runtime calibration learners. Repeated accel words keep only the newest accel
observation because accel alone cannot advance orientation. Waiting-for-timestamp
queue overflow and completed-sample queue overflow are distinct failures: the first
loses timestamp association before assignment, while the second drops an already
timestamped sample waiting for the consumer. They have separate counters, quality
flags and recovery reasons.

The relative modulo-4 gyro/accel tag-counter offset is learned from normal
pairs. An isolated offset mismatch marks only that pair as degraded: gyro is
still integrated, while accel correction is disabled. A persistent new offset
is re-locked after several identical pairs, which handles a clean FIFO epoch
change without forcing recovery. A genuinely superseded gyro is marked as an
orphan for diagnostics, but orphan/component-loss flags do not become timing
recovery triggers. Normal pending state across FIFO drain boundaries is not
marked as orphaned.

## magnetic reliability ownership

Pure sensor-domain logic is split into:

```text
sensor/mag_horizontal_trust.hpp shared pure local-field heading observability policy
sensor/mag_field_reliability.*  temporal field reference, disturbance and recovery
sensor/mag_axis_alignment.*     proper-rotation gyro/mag solver and bounded collector
sensor/mag_yaw_correction.*     normal and large-error yaw correction gates
```

`runtime/mag_runtime_controller.*` owns orchestration, timestamps, bounded solve
scheduling and calibration-candidate staging. It may inspect/stage candidate
storage through explicit dependencies, but it cannot flush or promote. App hooks
only wire current gyro timestamp/state and storage callbacks. This keeps magnetic
math host-testable and prevents storage or CLI ownership from entering the
per-sample algorithm modules.

### deferred ownership and continuous rotation

The magnetic sample path may only collect bounded observations and set a pending
action. `TrackerApp::loop()` services at most one solve/slot-check/stage action
after FIFO and network work; the app callback rejects service while FIFO is
pending or urgent. Storage presence probing is lightweight and fail-closed.

`mag_axis_alignment` owns the complete numerical solve. It first enumerates the
24 proper signed permutations, then refines the best hypotheses on `SO(3)` within
a bounded angular radius. The result is therefore a pure rotation, not a general
3x3 calibration matrix. `runtime/mag_runtime_controller` may stage a measured RAM
candidate only when the result beats active alignment on the same intervals. The
storage layer owns comparison/promotion and recognizes measured alignment quality
without changing the persistent record layout.

### independent evidence and realtime admission

`mag_field_reliability` owns the stationary-discontinuity latch. It is active only
under the existing gyro/accel stationary condition, is deliberately above the
normal yaw-correction rate, and can only clear through stable return to the
established field or explicit reference restart.

`mag_axis_alignment` assigns every retained interval to a temporal window. Even
and odd window IDs form deterministic training and validation partitions. Coarse
search and `SO(3)` refinement use training data only; validation independently
selects the winner, evaluates active alignment and supplies candidate quality.
Confidence is normalized by observable angular motion instead of raw score
separation.

The app composition owns deferred admission because only it can inspect both the
real LSM FIFO and output scheduler. The controller receives a structured gate
result and records software-FIFO, hardware-status, hardware-busy and output-
deadline deferrals. Numerical/storage modules remain unaware of hardware globals.

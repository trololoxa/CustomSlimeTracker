# Calibration Validation

## Guided temperature-capture validation

The dedicated setup temperature capture is covered by a native system test that
feeds complete sample/quality sequences. It verifies stable-window acceptance,
brief-touch recovery without restarting the full capture, slow-rotation and
vibration rejection, timestamp-fault pausing, and final partial-window commit.
This patch does not require a tracker runtime test because it changes only the
deterministic capture state machine; the existing physical setup procedure is
unchanged.

The firmware now exposes SlimeVR Server serial-compatibility commands for initial provisioning: `SET WIFI`, `SET BWIFI`, `GET INFO`, `GET CONFIG`, `GET TEST`, `GET WIFISCAN`, `REBOOT`, `FRST`, `DELCAL`, and temperature-only `TCAL`. `SET WIFI`/`SET BWIFI` save credentials into the separate network NVS config, enable Wi-Fi/discovery, and restart SlimeVR discovery. Blocking Wi-Fi scans suppress expected FIFO-recovery console noise for a short grace window without disabling recovery, counters, or machine-log events. `TCAL SAVE` is intentionally scoped to gyro temperature compensation so host-side temperature-calibration commands cannot accidentally capture unrelated runtime output/accel state. `TCAL RESET` changes only the in-RAM temperature model, resets the volatile runtime residual gyro trim learned against the previous model, and does not mutate the persistent config object unless a later explicit `TCAL SAVE` is requested.

## Sensor-to-device alignment validation

`test_sensor_to_device_alignment` is the host system test for the physical case-frame stage. It covers all 24 right-handed signed axis mappings, a non-discrete proper rotation, separation of rotation from a combined full 3x3 accel fit, contiguous stationary capture with an interrupted window, rejection of duplicate/parallel positions, runtime forward/inverse application and config validation. The normal hardware acceptance for this patch is only successful compilation; the next real `setup calibration` run will perform the two short physical observations through the same guided flow.

##  calibration-storage hardware acceptance

Only one real-device cycle is required after all PlatformIO profiles build:

```text
config slots
config save
config slots
reboot
config verify
config slots
```

Acceptance requires two valid generations after the second save, a valid
selector, the expected active generation after reboot, and unchanged tracking
calibration. The destructive fallback portion is optional unless a development
build exposes safe NVS corruption tooling; native tests already cover torn
writes, corrupt selected slots, selector loss and interrupted promotion.

Candidate smoke test:

```text
cal candidate stage manual
cal candidate status
cal candidate compare
cal candidate discard
```

Staging/status/compare/discard must not reset AHRS, reconfigure FIFO or change the
selected active generation. Do not use `promote force` as a routine hardware
smoke test.

### first-boot migration regression

When upgrading a supported legacy single-blob installation, boot must reach the
console and report the expected build identity. Run `config slots` and
`config verify`; legacy `cfg` migration must complete without a reboot loop.
The host gate `tests/contracts/test_calibration_storage_stack.py` must also pass.


### Storage lifecycle regression

For the intended test build, run:

```text
version
config slots
config verify
cal candidate status
```

A normal boot must report `load_status=loaded` or `migrated`,
`storage_degraded=no`, `authoritative_apply_pending=no`,
`commit_uncertain_count=0`, and no unexpected legacy cleanup pending state.
`defaults_storage_error` is a degraded boot and must not be treated as an empty
first-run device. In that state `config save`, candidate flush/discard/promotion
and calibration persistence must remain blocked. `config verify` is read-only and
must not clear the block; only a successful `config load` plus hardware/runtime
apply may confirm recovery. `config erase` is the deliberate destructive escape.

Supported legacy v1, transitional v2 and deployed v3 slots must migrate to a
validated committed v4 slot without losing valid calibration or user policy.
A second boot must load v4 without generation churn; an identical save must
increment `noop_save_count` while leaving generation unchanged. Compatibility
normalization is limited to the [named storage rules](../architecture/calibration_storage.md).

Candidate promotion is calibration-only: it must not increment FIFO
reconfiguration/recovery counters, change output rate/FIFO/IMU settings, or stop
network output. A v3 candidate remains fresh after unrelated policy/evidence/timestamp saves but
must become stale after any active gyro/accel/mag/alignment calibration change.
After promotion, `last_comparison=promoted` must survive reboot and a repeat
promotion, including `force`, must return `already_promoted` without changing the
active generation.

### hardware no-op and model-freshness acceptance

After `config load`, run two immediate `config save` commands. With no runtime
policy change, both must leave `active_generation`, selected slot, config CRC and
`successful_active_writes` unchanged while incrementing `noop_save_count` twice.
Active quality/provenance must remain unchanged.

Stage a v3 candidate, toggle only `cal temp enable|disable save` or save another
non-calibration policy, and compare again. It must not report stale. Changing an
actual gyro/accel/mag/alignment model must report stale. A promoted candidate must
preserve current `temp_comp_enabled`, output/AHRS/FIFO/SPI policy, transfer mag
trust bounds, clear magnetic heading/yaw state and leave FIFO reconfigure/recovery
counters unchanged.

Integration acceptance must also verify:

- `cal gyro save` cannot persist an unsaved accel change and `cal accel save` cannot
  persist an unsaved gyro/temperature change;
- `config load/defaults`, gyro-validity changes, candidate promotion and local
  magnetometer enable/disable cause SensorInfo to become dirty and re-ACK;
- enabling magnetic yaw without accel, magnetic-field and axis-alignment calibration fails
  explicitly instead of reporting success after sanitize disables it;
- promotion returns `runtime_calibration_diverged` when RAM contains a third
  unsaved calibration model;
- a persisted temperature slope without a valid gyro bias sanitizes to no
  temperature model and clears its evidence; `cal temp set_slope` must reject
  the same missing-bias state without writing NVS.

##  magnetic acceptance

The native suite includes temporal field disturbance/recovery, repeated
independent disturbance cycles, proper-rotation enumeration/reflection rejection,
gyro/mag timestamp-skew rejection, synthetic axis solve, and large-yaw
reacquisition. Hardware acceptance must also cover a same-norm directional
magnetic disturbance, removal/re-entry, a yaw error beyond the normal innovation
limit, multi-axis motion coverage/candidate staging, and concurrent FIFO/network
counters. See [magnetic_heading_reliability.md](../architecture/magnetic_heading.md)
for the exact sequence and expected states.

### regression matrix

The hotfix adds integrated native coverage for:

- actual 60 Hz noisy heading samples through the field monitor into yaw
  reacquisition;
- stable changed-environment fail-closed behavior and explicit reference restart;
- two-stage coarse-plus-`SO(3)` recovery of an approximately two-degree mechanical
  misalignment;
- orthonormal/determinant constraints and reflection/shear rejection for new
  solver results;
- active and candidate scoring on one dataset;
- measured axis candidate comparison and normal promotion preparation without
  `force`;
- policy enforcement that sample callbacks contain no full storage inspection,
  candidate flush or promotion.

Hardware acceptance must additionally measure solve/storage max duration while
960 Hz FIFO and 100 Hz network output are active, and require zero FIFO
full/overrun, no recovery entry and normal delivery during deferred service.

### regression matrix

Native and policy coverage additionally requires:

- abrupt stationary heading shifts of 5, 9, 12 and 19 degrees remain disturbed;
- slow drift and the maximum normal 2 deg/s yaw correction do not self-latch;
- real physical rotation with nonzero gyro does not trigger the stationary latch;
- realistic 60 Hz datasets solve at 30, 60, 90 and 120 deg/s using normalized
  confidence;
- a hypothesis that fits training windows but not held-out windows is rejected;
- training and validation each contain multiple independent temporal windows;
- rotation-deadline slack reports armed, future and due states correctly;
- policy gates require real FIFO status, output-deadline admission and
  axis-independent collection for repairing bad active alignment.

Hardware acceptance must still establish actual ESP32-C3 solve/storage duration
and require zero FIFO overrun/full, no recovery entry and normal packet delivery.

### cross-ABI promotion stack regression

`tests/contracts/test_calibration_storage_stack.py` now rejects by-value
`TrackerConfig` composition inside `prepareCandidatePromotion` and enforces a
promotion-specific 768-byte host stack ceiling. This provides margin below the
1 KiB project limit across GCC ABIs; host frames do not certify target stack use.


##  setup/output acceptance

The guided calibration still requires only the six ordinary case faces. Each
capture automatically includes a short held-out tail; do not move the tracker
until the command asks for the next face. The final verifier asks for one stable
ordinary face and checks the already-prepared coherent quaternion/linear
acceleration pair. No 45-degree or diagonal placement is required.

After setup, run:

```text
setup verify
cal autonomy status
config slots
perf on
perf status
```

`setup verify` must report `setup_output_verified=yes`, zero FIFO/timestamp/
recovery deltas, a valid linear-acceleration ratio of at least 0.8 and bounded
stationary residuals. `cal erase_all confirm` is the destructive recovery path
for completely removing calibration, candidate and autonomy transaction state.

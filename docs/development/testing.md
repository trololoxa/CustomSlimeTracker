# Testing

Start with the changed owner in [test map](test_map.md). The
[maintenance rules](maintenance.md) define naming, ownership, duplication and size
limits. A focused PASS is partial; missing tools/data and timeouts are not PASS.

## Entrypoints

Use the explicit Python/compiler environment in [session](session.md).

```powershell
& $TrackerPython tools/check_all.py --list-checks
& $TrackerPython tools/run_standalone_tests.py --list-tests
& $TrackerPython tools/check_all.py --check test_gate_reporting --check validate_documentation
& $TrackerPython tools/run_standalone_tests.py --cxx $TrackerCxx --test test_core_math_ahrs
```

Normal full host validation (native, contracts/tooling, documentation, replay):

```powershell
& $TrackerPython tools/check_all.py --clean --host-only
```

Full local Windows/WSL/firmware verification uses
[cross-platform verification](cross_platform_verification.md). It requires clean
committed checkouts and preserves failed stages; it does not edit documentation.
For a local firmware build matrix without WSL:

```powershell
& $TrackerPython tools/check_all.py --clean --require-pio --pio-bin $TrackerPio
```

Release is distinct: `tools/check_all.py --release` requires a clean Git identity,
complete normal/ASan-UBSan/LSan native matrices, all five clean target builds,
nonempty ELF/BIN and manifests, and a real LOGVER3 fixture with independent golden
JSON. Current release data gaps are in [status](../status.md). Do not use skip flags
or synthetic smoke data to claim release readiness.

## Scope and diagnostics

- Native logic tests execute real host-compiled modules using controlled adapters;
  they do not prove physical SPI/GPIO/FIFO timing, NVS or ESP32 performance.
- Contract checks enforce build/source/stack constraints. Host ABI budgets are
  different evidence from target timing and target stack measurements.
- ASan/UBSan and separate LSan have executable capability probes. Requested modes
  require both clean execution and intentional-fault detection; no silent fallback.
  Some historical optional contract variants explicitly report unavailable modes.
- Raw logs and JSON summaries live under `build/gate_runs`. Read the summary first,
  then the relevant failed log. Preserve earlier failures when rerunning.
- Expected injected `ERR` output is judged by assertions and exit status.
- Recoverable FIFO/network events have bounded budgets and recovery requirements;
  they do not automatically invalidate good calibration. Invalid/stale data,
  partial persistence, wrong-way correction and unbounded waits remain hard failures.

The runners own compile/test deadlines and process cleanup; see
[runner details](runners.md) and [test map](test_map.md). Do not create another
shell/Python full-test runner or add unbounded retries.

## Additional evidence

- [Fusion accuracy and comparison](fusion_validation.md): independent oracle,
  actual algorithm scenarios, compatible input/config/harness fingerprints.
- [Mathematical coverage plan](math_coverage.md): implemented versus future scenarios.
- [Replay](replay.md) and [capture validation](capture_validation.md): recorded data.
- [Device smoke](device_smoke.md), [sensor recovery](sensor_smoke.md),
  [calibration acceptance](calibration_validation.md): explicit hardware work.

For migrations, map removed checks to retained behavior/variants before deletion.
Use selected negative mutations where needed to show a replacement still detects
its defect. A documentation-only edit does not require all native/firmware builds.
A completed runner/registration migration does require a full host check once the
focused checks are stable; final Windows/target checks remain environment-specific.

## Development branch acceptance

DEV-07 closes CI/artifact delivery, a real reviewed LOGVER3 fixture, current
Server interoperability and a demonstrated compatible rollback. A patch prepares
these operations; it cannot record hardware PASS on behalf of its operator.
The [status page](../status.md) owns the actual completion state. Keep the external
local-agent implementation outside this firmware/tooling change.

Use one candidate commit and one evidence directory per acceptance attempt.
Record full Git SHA, environment, board USB identity, tool versions, Server
version/build/commit, UTC time, report paths and conclusions. Keep failed attempts
and raw logs; a later PASS does not overwrite them. Do not publish Wi-Fi secrets,
private network details or a whole raw NVS dump in a public GitHub artifact.

1. Apply/review the patch, run its focused checks, commit selected paths and open
   a draft PR. Ordinary PR CI is the economical scope in [runners](runners.md).
2. Request one full CI run for that commit. Confirm host, both sanitizers and all
   six profiles, plus `CI result`, succeeded. Download reports and the selected
   ProductionDiag bundle. A historical green run certifies only its own SHA.
3. Extract the nested firmware ZIP preserving paths into a checkout of the
   artifact SHA. Run `device_workflow.py flash-plan` there. Check embedded
   identity/profile, hashes, partitions and `opens_device=false`. Preserve the
   whole bundle; copying only manifest.json leaves its source files unprotected.
4. Prepare a known-good rollback bundle and record configuration/calibration
   evidence before any update. Use the app-only procedure in
   [device smoke](device_smoke.md#review-and-explicitly-flash-a-saved-build).
   A/B rollback acceptance is described below. No full erase or automatic rollback.
5. On normal ProductionDiag, verify Server compatibility below, then obtain the
   stationary cable-free strict capture and reviewed golden in
   [capture validation](capture_validation.md#strict-cable-free-static-capture).
   ProductionDiag USB health may truncate: use the TCP commands/reports for this
   profile, not a larger USB timeout. USB_DIAG is a bench variant; its successful
   serial smoke does not certify ProductionDiag capture or sleep behavior.
6. Complete the outstanding boot/reset/safe-mode observation when a planned
   reset exposes the serial boot text. `listen` is passive; an empty log or lost
   USB endpoint is missing evidence. Do not force a crash/safe-mode just to create
   a log. Record reset reason and observed safe-mode state, not an assumed PASS.
7. Commit only reviewed real fixture/golden inputs and relevant documentation.
   On the final substantive source run the Windows→WSL workflow once, following
   [cross-platform verification](cross_platform_verification.md). Resolve mounted
   checkout differences instead of bypassing its source check. Known deferred
   Windows stack failures remain visible; they do not become a green full gate.
8. For a release-readiness claim, run `check_all.py --release` in a suitable Linux
   environment with PIO and both sanitizer capabilities, after committing the
   fixture. It additionally owns clean builds/manifests and golden checking.
   Hosted `full` CI is not this release gate. An unavailable tool/data or FAIL
   blocks that claim; no skip flags or relaxed thresholds.
9. Review evidence and update existing status/roadmap. DEV tooling may be accepted
   with the owner's already-deferred firmware stack issues explicitly retained;
   this is scoped DEV acceptance, never full firmware/release certification.
   Any additional failure needs diagnosis and a concrete disposition.
10. Complete the separately agreed Codex evaluation before declaring the whole
    dev-preparation programme complete. Then merge the reviewed PR, verify main
    CI and ancestry, preserve evidence, and only then remove the finished branch.
    Firmware work resumes at remaining 0027/0028, then 0029 and 0030.

### Current Server interoperability

Use the actual Server used for tracking. Record its version and, for a combined
custom build, exact commit/branch composition. "Latest" is not an identity.
A second official build is needed only if claiming compatibility with it too.
Do not change packet mode or add protocol features during this DEV acceptance.

- Record `version`, `net status`, `slime status`, `slime debug`, and the Server
  log before testing. Confirm the expected tracker identity and live orientation;
  exercise visible motion and verify supported acceleration/battery telemetry
  where the selected configuration provides it. Mark unsupported fields N/A
  with the configuration reason; a GUI TPS estimate alone is insufficient.
- Prefer `capture_telnet_log.py --capture runtime --seconds 600 --rate 20
  --mode full --host <tracker-ip> --output <new-log>` for the stable window: it
  owns the TCP lease, capture and retained TESTSUM. Require both `passed=true`
  and `health_passed=true`; runtime mode retains health faults for diagnosis.
  Alternatively keep an interactive TCP session alive (input every <30 s), run
  `test runtime 600`, then request `test report runtime`. Use the [runtime criteria](capture_validation.md#full-runtime--wi-fi--slimevr-load-test)
  for the stable window. Save Server and tracker observations together.
- After the clean window, stop/restart only the selected Server instance. Measure
  loss detection and reconnection; verify the same tracker resumes, no duplicate
  identity is created, orientation progresses and no reboot is needed. Keep this
  intentional outage separate from the zero-disconnect baseline.
- Test a tracker-only Wi-Fi interruption if the acceptance claim includes it;
  avoid restarting a shared router. Record actual recovery duration and compare
  with the configured retry/discovery policy, without inventing a new SLA.
- This checks the network boundary and supported packet behavior; it does not
  certify absolute pose accuracy, every Server version, or all nine trackers.

### Compatible update and rollback

This no-OTA layout rolls back by explicitly writing a saved app image. A manifest
proves matching bytes, not older firmware's ability to read newer persisted data.

1. Select accepted old bundle A and candidate B. Compare partition tables and
   persistent formats/migration code between their source commits. If either is
   incompatible or unknown, stop and design the migration/recovery separately.
   Never resolve incompatibility by erasing calibration or writing old raw NVS.
2. Keep both complete bundles in durable storage. Validate both with `flash-plan`
   in their corresponding checkouts. Record existing `config print`, `config crc`,
   `config nvs`, `net print`, `cal status`, `bias status`, `mag cal status`,
   `mag cal print`, `mag axis print`, `cal temp print` and `setup status` as available.
   These are inspection commands, not a restorable backup or authorization to save.
3. Install B using the selected USB identity, esptool 4.9.0 and app-only write plus
   mandatory read-back. Confirm B's exact `version`, profile and sensor/network
   health. Re-read saved configuration/calibration evidence after startup.
4. Explicitly install saved A by the same controlled operation. Confirm A's
   version and healthy operation, stored config validity and calibration values,
   and restored Server connection. No erase, recalibration or credential entry
   may be silently required to make rollback look successful.
5. Dynamic counters/temperature/RAM bias need not equal the baseline. If persistent
   values/CRC change, explain them from actual migration/autonomy evidence; an
   unexplained difference leaves preservation unaccepted. The uploader itself
   does not write NVS, but firmware startup/autonomy can.
6. If B is the desired final installed candidate, explicitly return to B and
   verify it. A successful A→B→A cycle proves only this version pair and this
   configuration. Keep the final installed identity with the acceptance report.

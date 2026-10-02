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

# 0027c Windows stack and failure summary report

## Scope and ordering

0027c is additive after 0027, 0027a and corrected 0027b. It fixes the remaining
MSYS2 yaw stack failure and makes aggregate failures repeat the captured child
stderr that explains the failure.

## Changes

- `MagYawCorrectionController::update(InputView, ...)` is split into bounded,
  no-inline semantic phases. Gates, cooldown, reacquisition and correction math
  retain their original order and thresholds.
- The 96-byte public frame ceiling is unchanged. An x86-64 Microsoft-ABI probe
  checks `-O2` and `-Os`; every helper is capped at 96 bytes and the nested peak
  at the 192-byte predecessor maximum.
- `check_all` keeps stdout live, captures child stderr, prints it immediately,
  and repeats the complete captured stderr beneath the final failure entry.
- The deliberately injected FIFO drain failure still exercises production
  recovery, but its expected `# ERR` line is consumed by a test-only sink.

## Verification

- Python syntax and aggregation unit policy;
- 0026b hot-path policy;
- x86-64 Microsoft-ABI stack probe at `-O2` and `-Os`;
- focused magnetic reliability and FIFO runtime native executables;
- source/object code-size comparison.

Target ESP32 build, profiler and Windows/MSYS2 `check_all --clean` remain target
verification. No sensor threshold, estimator equation, persistent schema, FIFO
recovery behavior, sensor cadence or packet format changes.

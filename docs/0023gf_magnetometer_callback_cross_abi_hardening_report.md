# 0023gf magnetometer callback cross-ABI hardening

## Confirmed defect

Windows/MSYS2 GCC measured
`MagRuntimeController::processRawSample(const MagRawSample&)` at 1040 bytes,
exceeding the existing 1024-byte policy ceiling. Linux GCC measured 944 bytes,
so the previous policy left only an 80-byte ABI/compiler margin and did not
protect the Windows build.

The frame combined a 148-byte `MagRuntimeConfig`, the reliability input/output,
the yaw input/output, heading workspace and gyro-endpoint temporaries in one
compiler-visible 60 Hz callback. The 0023ge tests validated behavior and Linux
stack, but did not require phase-specific cross-ABI margins.

## Fix

The callback is now a no-inline orchestrator with separate no-inline phases for:

- coherent gyro endpoint capture;
- magnetic heading update;
- field reliability update;
- yaw correction update/application.

The phase boundaries are functional lifetime boundaries, not heap workspaces.
No queue, persistent structure, schema or calibration model changes. The
orchestrator also captures one `millis()` value and builds one
`MagRuntimeConfig` per magnetic sample; 0023ge previously called `millis()` five
times and built the 148-byte runtime config twice.

The inverse transform for an already accepted sensor-to-device matrix is now
private to `mag_runtime_controller.cpp`. The earlier public header helper could
be called without carrying the required validation decision and was unnecessary
API/maintenance debt.

## Stack and hot-path direction

Linux GCC stack after the fix:

| Function | `-O2` | `-Os` | 0023gf ceiling |
|---|---:|---:|---:|
| `processRawSample` | 336 B | 320 B | 512 B |
| `captureGyroEndpoint` | 8 B | 8 B | 192 B |
| `updateHeadingSnapshot` | 80 B | 80 B | 256 B |
| `updateFieldReliabilitySnapshot` | 384 B | 368 B | 640 B |
| `updateYawCorrectionSnapshot` | 416 B | 416 B | 640 B |

The no-inline calls execute at the magnetic rate (about 60 Hz), not the 960 Hz
IMU rate. Their call overhead is offset by removing four repeated `millis()`
reads and one complete runtime-config construction per magnetic sample. No
additional heap allocation, queue growth, persistent RAM or unbounded work is
introduced.

Host `-Os` controller-object direction versus 0023ge is `text +197 B`,
`data +0 B`, `bss +0 B`. The original monolithic function shrinks from about
1559 to 646 host text bytes; the isolated phases account for the remaining
code and make their stack lifetimes enforceable. PlatformIO linker totals and
real ESP32 timing remain mandatory acceptance gates.

## Post-audit of 0023ge

The 0023ge count/time callback budgets, fail-closed raw-timeline gate, one-frame
validation reuse, fit diagnostic retention and fit stack refactor were reviewed
again. Targeted native tests, sanitizers and source/profile policies found no
additional semantic regression after this suffix. The callback-budget logic
still guarantees at least one due magnetic callback can run at the start of a
fresh cooperative slice, then parks raw advancement if count or time limits are
reached.

Residual hardware requirements remain unchanged: Production, Production-Diag
and Slim PlatformIO builds, guided calibration on LSM6DSV/QMC6309, and perf
verification that FIFO overrun/full, rotation deadline and UDP failure deltas do
not regress.

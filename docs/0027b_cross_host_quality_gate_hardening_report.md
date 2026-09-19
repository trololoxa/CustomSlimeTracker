# 0027b cross-host quality-gate hardening report

## Scope and ordering

0027b is an additive patch. Apply it after 0027 and 0027a. It fixes three
Windows/MSYS2 host-gate failures reported after that sequence; it does not
replace either predecessor patch.

No sensor threshold, fusion equation, FIFO cadence, packet format, persistent
schema, recovery deadline or watchdog policy changes in 0027b.

## Confirmed failures

The enabled remote-console compile-only translation unit included
`sys/socket.h` on every host. That header is available to ESP/lwIP and POSIX
hosts but not to the native Windows compiler used by the standalone suite.

The Windows compiler also measured:

- `MagFieldReliabilityMonitor::update()` at 432 bytes against 384;
- `runtimeBiasFinalizePendingWindow()` at 400 bytes against 256.

Both production files were byte-identical before and after 0027/0027a, so these
were pre-existing cross-ABI headroom defects revealed by the user's gate, not
stack growth introduced by sensor-liveness recovery.

## Changes

The remote console keeps the exact ESP32 `send(..., MSG_DONTWAIT)` operation
behind a target-only boundary. The native Wi-Fi stub receives a compile-only
fallback and no longer imports POSIX socket headers. No Windows socket emulation
or extra link dependency is added.

Magnetic reliability is split into fixed no-inline phases for output reset,
heading-rate evidence, reference evidence, stationary discontinuity evidence,
state transition and final publication. The phases operate on the same monitor
and caller-owned output; there is no shadow state, heap workspace or second
trust owner. Threshold order and state-machine order are unchanged.

Runtime-bias finalization is split into evaluation, reject, probation and apply
phases. Evaluation writes the same existing diagnostic fields and returns a
small bit mask. No new estimator state or persistent/RAM workspace is added.

The predecessor stack policies now bound every new helper as well as the public
entry point, preventing a later change from hiding a large frame behind a small
wrapper. Existing ceilings are not raised.

## Host measurements

Authoring-host GCC 13.3 `-fstack-usage` measurements:

| Boundary | Before | After `-O2` | After `-Os` |
|---|---:|---:|---:|
| Magnetic public update | 224 B | 64 B | 64 B |
| Magnetic largest phase | n/a | 48 B | 128 B |
| Bias public finalizer | 208 B | 32 B | 48 B |
| Bias largest phase | n/a | 144 B | 208 B |

An additional x86-64 Microsoft-ABI compile probe measured the magnetic public
update at 112 B (`-O2` and `-Os`) with a largest phase of 208 B (`-O2`) and
272 B (`-Os`). The bias public finalizer measured 64 B, with evaluation at
272/288 B. These are host compiler estimates, not ESP32-C3 target measurements.

The maximum nested host estimate is not increased: on the Microsoft-ABI probe
magnetic `-Os` is 112 + 272 = 384 B, while the old monolithic frame was 448 B.
Bias `-Os` remains bounded at 64 + 336 = 400 B, equal to the reported old
monolithic frame; `-O2` decreases materially. Target stack high-water remains
the release authority.

## Verification

Run:

```text
python3 tools/check_all.py --clean
```

The authoring environment additionally runs the magnetic decision/recovery
suite, runtime-bias behavioral suite, `-O2`/`-Os` stack policies, sanitizer
fallback policy and the complete standalone matrix.

Windows/MSYS2 and PlatformIO target builds are not available in the authoring
environment. Rerun the same aggregate command on the reporting Windows host;
then run the existing 0027a target smoke. Compare `perf tracking`, magnetic
state/recovery counters and task stack high-water with the pre-0027b build.

## Rollback

Rollback is code-only: reverse 0027b. There is no migration, NVS cleanup or
calibration reset.

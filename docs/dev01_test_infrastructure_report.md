# DEV-01 — trustworthy assertions and sanitizer capabilities

## Base and scope

Apply after 0028d on the separate Codex preparation branch. This patch changes
host tests/tools and their documentation only. Runtime firmware sources, profiles,
partition layout, estimator, physical thresholds and diagnostic transports are
unchanged. No board flash or persistent operation is required.

## Defects and changed contracts

1. `CHECK_NEAR` rejected non-finite actual values, but could accept NaN expected
   values or NaN/infinite tolerances. It now requires finite operands and a finite
   nonnegative absolute tolerance. Zero tolerance and signed zero remain valid;
   comparison boundaries and ordinary float arithmetic are unchanged.
2. Sanitizer discovery previously compiled/linked `main(){return 0;}` without
   executing it. It could claim support with a broken runtime. Discovery now
   requires a clean execution followed by a failing known-bad execution with the
   expected sanitizer diagnostic. Combined ASan/UBSan proves both independently.
3. Explicit native/release sanitizer selection now runs the same preflight and
   fails if it is unavailable. It never silently falls back. Historical optional
   development checks may still select UBSan or none, explicitly reporting every
   capability result with an evidence path. Their optimization flags are preserved.
4. A native timeout must be finite and positive; NaN/infinity are rejected.
5. Explicit LSan enables leak detection even if inherited options disabled it.
   ASan/UBSan still excludes implicit LSan using the existing environment AND
   compiled-in runtime policy. These are distinct verification gates.

Sanitizer probes have bounded compile/run stages, no retries, private temporary
executables, and full stdout/stderr/exit status/command evidence in ignored
`build/sanitizer-probes/*.json`. Reports accumulate until the user cleans build
artifacts; full retention/summary policy belongs to DEV-03. No environment or
credentials dump is written. An unavailable result includes the failed stage;
read its retained stderr to distinguish missing libraries, timeout and runtime
restrictions. Probe source paths remain reproducible in the repository even
though their temporary executables are deleted.

## Commands

Verify the helpers and gate behavior without rebuilding firmware:

```sh
python tools/test_dev01_test_infrastructure.py
python tools/test_quality_gate_runtime.py
python tools/test_run_standalone_tests_policy.py
python tools/test_check_all_aggregation_policy.py
```

Check actual sanitizer capabilities using the desired compiler:

```sh
python tools/sanitizer_probe.py --cxx g++ --sanitizer address-undefined --sanitizer undefined --sanitizer leak
```

Each requested unsupported mode makes this command fail. There is no implicit
skip, retry or downgraded PASS. Use the actual compiler path when PATH is ambiguous.

Run an explicit native gate after the environment is ready:

```sh
python tools/run_standalone_tests.py --sanitizer address-undefined
python tools/run_standalone_tests.py --sanitizer leak
```

The native runner performs preflight automatically; do not separately repeat the
capability command before every native gate. Use the standalone command to diagnose
an environment. `--build-only` still verifies capability before compilation; it
does not turn a non-executable sanitizer setup into verified coverage.

## Self-check coverage

The assertion fixture runs 19 child-process cases: normal boolean assertion,
exact/boundary comparison, signed zero, maximum finite values, inequality, NaN
and infinities in operands/tolerance, negative tolerance, overflowing difference
and a one-ULP-outside tolerance boundary. Parent Python tests inspect real exit
codes and diagnostics rather than trusting `TestContext` to test itself.

Sanitizer regression tests cover missing/empty executable, compiler/launcher
failure, clean crash, absent/wrong diagnostic, zero exit with diagnostic, timeout
after a diagnostic, both combined runtimes, leak selection, full evidence
retention, invalid deadlines, explicit-gate refusal and suite exit propagation.
Existing process-tree timeout and check_all aggregation tests remain applicable.

Deliberate failures are confined to fixtures and captured by their parent. A
printed `FAIL`/sanitizer diagnostic in raw self-test evidence is expected only
when the parent proves the specified result. Ordinary tests keep their original
error semantics. The address fixture excludes UBSan on one deliberate access
so UBSan object-size instrumentation cannot intercept ASan's test first; a
separate signed-overflow fixture proves UBSan with the combined flags. This
exception is not applied to project tests or firmware.

## Limits

These probes prove basic execution and detection, not complete sanitizer coverage,
thread/ISR correctness, symbolizer completeness, algorithm correctness or target
memory behavior. MSan/TSan, a compiler inventory, algorithm replay, selective test
execution and HIL are later DEV work. No new physical capture is required here.

Local verification results are recorded in the distributed DEV-01 validation
report and raw logs; unsupported host capabilities must remain visible.

## Verification of the delivered revision

- Infrastructure group: 36 Python tests passed, including all 19 real assertion
  child-process cases. One additional LSan-option regression was then added;
  the affected quality-gate-runtime suite passed all 12 tests (11 existing plus
  the new case). These counts overlap; they are not 48 distinct tests.
- One complete native ASan+UBSan attempt built and passed 50 executables. The
  remaining `test_calibration_autonomy.cpp` compilation exceeded its 180-second
  per-command timeout. The complete command therefore returned FAIL, not PASS.
- Only that remaining executable was rebuilt with the same sanitizer flags and
  unchanged shared objects, with a one-off 600-second command deadline. It built
  and passed. GCC noted its variable-tracking size limit during compilation.
  All 51 native executables thus passed across these two attempts; no default
  deadline was raised and no second full native run was performed.
- Actual ASan and UBSan deliberate-fault probes passed. LSan failed its clean
  executable: this sandbox cannot read the required `/proc/<pid>/task` data.
  No leak-sanitizer PASS is claimed. Symbolizer warnings also remain visible;
  complete stack-symbolization capability is not verified here.
- The 0026d portability policy, documentation validator, whitespace check and
  patch application/byte comparison passed. All runtime `src/` bytes match the
  original supplied 0028d snapshot.
- Full check_all, Windows/MSYS2 execution, ESP32 builds, hardware tests, complete
  symbolization and release acceptance were not performed. DEV-01 does not close
  those gates or firmware roadmap 0027/0028 target acceptance.

Raw successful and failed-attempt evidence is included in the patch archive.

# DEV-03 — selective runners and retained failure evidence

## Base and scope

Additive to the supplied `Tracker_after_DEV2.zip`. The local Git baseline was
reconstructed from that archive; it is not the original project's historical
DEV-02 commit. The delivery manifest identifies the archive hash and both local
commits. Apply the patch to matching source, not by assuming commit ancestry.

Only Python development tooling and documentation change. Firmware sources,
native C++ tests, PlatformIO configuration, packet modes, calibration, sensor
gates, persistence and runtime hot paths are unchanged.

## Problem and changed contract

The existing aggregate/native runners required broad runs and produced noisy
console output. Failure details were available in console summaries, but were
not retained uniformly in per-command logs with machine-readable run state.

DEV-03 extends those runners with explicit check/test selection, list commands,
compact default output, raw combined stdout/stderr logs and atomic JSON reports.
Failed focused Python checks can be selected again from a completed report.
Only registered IDs are reused; commands, interpreters, binaries and previous
PASS results from the report are never reused.

`tools/gate_reporting.py` owns reporting; existing runners retain scheduling,
timeouts, process cleanup, sanitizer preflight, build locks and verdicts. The
new helper does not become a second runner or cache. Explicit invalid compiler
selection fails instead of silently choosing a different compiler from PATH.

The three policies for 0027b/0028/0028a previously depended on the indentation
of local check lists. Their registration assertions now inspect the shared
TOOL_CHECKS registry. Their firmware assertions remain unchanged. Behavioral
tests additionally verify that the full tool loop actually schedules the
registry and that release retains its native sanitizer matrix and replay gate.

## Verification performed

Linux host, available native compiler; no attached board.

1. One focused aggregate run: **9/9 checks passed**. It included DEV-03's 24
   tests, standalone runner policy (3), aggregate policy (9), DEV-01 (11),
   quality-gate runtime (12), check_all policy (2), and the three updated
   registration policies. Report: `check-all-3z1gna40/summary.json`.
2. One real native run: **2/2 passed**, `test_core_math_ahrs` and
   `test_sensor_calibration`, sanitizer `none`. Its 52 subprocess commands
   exercise shared compilation, an extra link object, both links and execution.
   Report: `native-wltrnmjv/summary.json`.
3. After the final compact unknown-ID diagnostic and documentation edits,
   reran only DEV-03's 24 tests and documentation validation: **both passed**.
   Report: `check-all-rb5xlvdl/summary.json`. The native matrix was not repeated for
   parser error-text/documentation edits.
4. Static validation compares the old registry entries/order to DEV-02,
   parses changed Python files, checks whitespace, and compares archive bytes
   outside tooling/docs: all 293 files unchanged; 57 tool checks and three
   validators retain their order. Delivery validation applies the patch to a fresh copy
   of the supplied archive and compares the result with the patched source.

The DEV-03 tests include real child failures, stdout/stderr retention, bounded
timeout, missing executable, interrupted report state, large successful output,
failure-to-fix rerun, and malformed/incomplete/foreign report rejection. They
also test explicit selection, release-mode exclusion, compiler selection,
required extra objects, full scheduling and PIO full-output size parsing.
Scheduling/PIO/release tests use mocks; they are not actual target builds.

## Review findings and limits

- Partial selection is labelled partial and cannot replace a release gate.
  Default full coverage and existing strict release preflight remain.
- Fresh private native directories, output validation and the native lock
  remain. There is no cross-run object cache and no stale-binary shortcut.
- Child output goes directly to files, avoiding an unbounded in-memory capture
  in the ordinary runner path. The existing PIO size parser still receives the
  full completed log. JSON updates add host filesystem I/O; no target CPU,
  stack, latency, battery or tracking improvement is claimed.
- Git metadata has bounded waits (two calls, up to five seconds each). HEAD
  and dirty state are startup metadata, not a content hash or release identity.
- Failure excerpts deliberately have size limits. A root cause in the middle
  of a long output remains available in the full raw log. Verbose output is
  printed after commands complete, not streamed live.
- Reports use atomic replacement, not power-loss durability. Interrupted or
  abruptly killed runs cannot be treated as completed PASS evidence. Retention
  is manual; there is no automatic deletion of failed or active evidence.
- Full check_all/PIO runs in one checkout still require sequential execution
  because existing replay/build outputs can be shared. See the usage guide.

## Not verified

No complete native matrix, full check_all, full sanitizer matrix, production or
release build was run for this tooling patch. Windows/UCRT64, WSL, actual PIO
execution, on-device smoke, debugger/flash, current Server interoperability and
hardware performance were not verified. Existing requirements for those gates
are not waived and no firmware audit defect is declared closed by DEV-03.

## Next

DEV-04: project AGENTS.md, owners/test map, current documentation and compilation
database. DEV-05 owns additional math/scenario replay. DEV-06 owns board access
and hardware tooling. Packet 23 is outside this patch.

Usage: [dev03_selective_checks.md](dev03_selective_checks.md).

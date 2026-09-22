# DEV-04: bounded workflow and agent context

## Base and scope

Additive after DEV-03a. Exact supplied archive: `Tracker_before_DEV04.zip`, Git
archive comment `76ff25c1b7ad4389572e3391f66361efd3b1de33`. The local verification
repository is reconstructed from those bytes; its synthetic base commit is not
the user's Git identity. Do not cherry-pick that synthetic history.

Only tooling/instructions/docs changed. Firmware, packet modes, ODR, estimator,
thresholds, persistence schema, PlatformIO pins and partition layout are unchanged.

## Defects / changes

- One native deadline previously bounded both compilation and test execution.
  The user's GCC 13.3 ASan/UBSan compile of `test_calibration_autonomy.cpp` hit
  180 s; a focused run with 600 s passed after about five minutes of compilation.
  Sanitized builds now default to 600 s per compile/link; test execution remains
  180 s. Non-sanitized build remains 180 s. Stage options override the legacy
  combined option. check_all forwards the options in all three native modes;
  its outer suite deadline remains independent and unchanged.
- Probe runtime remains capped at 30 s and bounded by the build allowance;
  a short test-only deadline does not truncate probe compilation. No retries,
  flags weakening, fallback, cache reuse or process-kill changes were added.
- Root AGENTS.md is a 455-word contract/router. No unconditional full-repo reads,
  skill-loading loop, new approval barrier for local tests or duplicate Skills.
  User preference GPT-6 Astra preserved; no account/model configuration mutated.
- `dev_source_check.py`: bounded, read-only HEAD/dirty/mismatch check before
  transferring verification between Windows/WSL checkouts. Not a replacement for
  manifest, wheel hashes or ignored-input validation. No auto-reset/commit/pull.
- `export_native_compdb.py`: reads successful local native reports and exports
  actual compile argument arrays for navigation. No report command execution.
  Validates report scope/checkout/sources; atomic file replacement retains the
  previous database on failure. This is host navigation, not target validation.
- Session commands, current test routing, previous user evidence and researched
  optional-tool decisions now have explicit documents. Root AGENTS links are
  included in the existing structural documentation validator.

## Review and tests performed here

- Focused six groups: DEV-04, DEV-03, standalone runner, check_all aggregation,
  quality-gate runtime, documentation: PASS (`check-all-u_qzrvho`).
- Final probe-budget adjustment: DEV-04, check_all policy, DEV-01 and DEV-02:
  PASS (`check-all-pnj2vzwe`). This follow-up resolves that concrete change;
  it is not another full test-suite run.
- DEV-04 has 10 test methods, with parameterized cases for default/legacy/stage
  precedence, finite-positive validation, effective metadata, main() state
  restoration, aggregator forwarding/outer deadline, real execution timeout,
  clean/mismatch/dirty/ignored Git worktrees, unavailable Git, exact argv export,
  foreign/failed/running reports and preservation of a previous database.
- Real native `test_core_math_ahrs`: 1/1 PASS, sanitizer none (`native-t3zmrgb3`).
  Recorded 47 compile/link commands at 240 s and one execution at 30 s.
  Export of that real report produced 46 compile entries; arguments preserved.
- Test-map references: 39 explicit test/policy IDs checked against registries.
- Syntax, local links, patch application and changed-file hashes are checked
  during artifact packaging. No blanket repository rebuild was needed.

Adversarial review: deadlines remain absolute in the existing process owner;
progress cannot extend them. Reporting still preserves failed raw output. Old
CLI options remain accepted. Probe failures still fail closed. Source identity
helper cannot repair mismatches silently. Native database is not injected into
firmware builds or global clangd settings. No firmware hot path changed.

## Earlier user evidence (not rerun by DEV-04)

- Windows default full check_all passed native/policies/replays and five target
  profiles before this patch; it was not `--release` or a sanitizer run.
- DEV-03a Windows focused checks and two selected native tests passed.
- WSL Ubuntu 24.04.4, Python 3.12.3, GCC 13.3.0: sanitized doctor passed actual
  clean/fault probes, source `76ff25c1b7ad4389572e3391f66361efd3b1de33` clean.
- ASan/UBSan: 50/50 built tests passed; autonomy compilation timed out at 180 s.
  User focused follow-up at 600 s passed 1/1. Combined executable coverage 51/51,
  not a retroactive PASS for the failed full-run summary.
- LSan: user reported a full 51/51 PASS. Complete raw logs of these user runs
  are not included in DEV-04; only the supplied failed ASan summary was inspected.

## Not verified / next acceptance

Actual Windows execution of DEV-04, actual clangd/Serena navigation, target
compiledb generation and debugger/board access remain unverified here. This
revision did not repeat full sanitizers, PIO matrix, replay, Server or hardware
smoke: it changed no firmware and corresponding baseline evidence stays scoped
above. Token reduction has not been measured in a Codex A/B run.

Strict release fixtures are still absent from this archive:
`tests/fixtures/replay/logver3_static_golden.log` and
`tests/fixtures/replay/logver3_static_golden.json`. Do not synthesize a fake
recording/golden or turn the missing-fixture failure into SKIP/PASS. Locate the
existing real approved evidence when preparing the release gate.

User acceptance: use the focused Windows command in [session guide](dev_session.md),
then commit and sync into WSL with exact SHA. The same focused command may be
run in WSL; a single selected native smoke is sufficient for local runner
integration. Another full Windows/sanitizer matrix is not requested by DEV-04.
Actual agent loading can be checked in the first local Codex session: ask it for
this repo's relevant test command and firmware constraints; verify it references
AGENTS.md and does not demand reading every report. User/global instructions
were not supplied and may need a separate narrowly scoped conflict check.

Next implementation: DEV-05 math/scenario replay gap analysis, then DEV-06 HIL.
Optional navigation tools can be enabled separately after a measured local trial.

## DEV-04b: user acceptance update and session documentation

Additive after DEV-04a; documentation only. Earlier "unverified here" statements
above describe the original patch environment; the user evidence below is later.
User source: `563d4b90375d2e12af7bddd3434fe9882a4d3fad`, reported clean.

- Windows five selected DEV-04/runner/aggregation/documentation checks: PASS,
  `build/gate_runs/check-all-24r9gxau/summary.json` (console output supplied).
- Windows native AHRS: 1/1 PASS, sanitizer none, build/test deadlines 240/30 s,
  `build/gate_runs/native-m5a8qfbj/summary.json`; export produced 46 entries.
- Local Codex reported reading AGENTS.md and relevant session/test-map docs,
  preserving edits, selecting explicit Conda Python and listing checks. This is
  observed instruction loading, not an exhaustive audit of global instructions.
- Standalone clangd 22.1.6 checked `src/sensor/ahrs_6dof.cpp` with the native
  database and explicit GCC query-driver: exit 0, 0 errors. User subsequently
  confirmed VS Code navigation works with competing IntelliSense disabled.
- After MSYS2 update, Windows doctor: PASS, GCC 16.2.0, GDB 17.2, clangd 22.1.6,
  `build/doctor/host-g8q__ufa.json` (console output supplied; JSON not attached).
  OpenOCD not found; no hardware or sanitizer check performed by this invocation.
- New GCC native AHRS: 1/1 PASS, sanitizer none, deadlines 240/30 s,
  `build/gate_runs/native-6kydz7op/summary.json` (console output supplied).
  Previous full Windows gate remains evidence for GCC 14.2.0 only.
- User reports completing the three Windows/WSL sync-and-check steps. The new
  WSL command outputs, exact SHA confirmation and summary paths were not supplied;
  do not invent a new independently reviewed WSL PASS. Existing results can be
  recorded without rerunning the tests solely to produce another report.

Implementation scope and Windows/native navigation acceptance are complete.
The remaining evidence detail is the reported WSL run above. Target compiledb /
ESP32 navigation, optional Serena integration and measured token savings are not
certified. Device/debugger attach belongs to DEV-06; math/replay gap work belongs
to DEV-05; release/Server evidence belongs to DEV-07. Closing this tooling patch
must not be described as completing all project preparation or release acceptance.

DEV-04b adds clangd to the existing PowerShell session block and documents the
separate editor setting and optional doctor requirement. No source, runner,
firmware, package pins or installed environment are changed by this patch.
Validation: documentation validator and clean patch application against the local
DEV-04a baseline; no firmware rebuild or sanitizer rerun for a documentation edit.

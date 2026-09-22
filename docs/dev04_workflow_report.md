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

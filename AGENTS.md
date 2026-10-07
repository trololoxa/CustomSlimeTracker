# Tracker working contract

ESP32-C3 / LSM6DSV / QMC6309 SlimeVR firmware. Firmware fixes and DEV tooling
are separate patch series. GPT-6 Astra is the preferred default, not a hard pin.
Within existing permissions and available client capabilities, another model may
be chosen without extra approval for work it can reliably handle. Keep the same
invariants, review and acceptance gates; lower cost alone is not evidence of equal
quality. If suitability or results are uncertain, use Astra or a stronger available
model and resolve the remaining checks. Briefly report actual model changes;
never promise zero regressions or claim an unavailable model switch.

## Work and evidence

Use current code as evidence; historical patch reports are not specifications.
Finish the requested change, inspect its diff, fix discovered regressions, and
run the relevant checks without stopping for approval after each local step.
Local native tests use host fixtures, not the tracker. Follow existing user
permissions for hardware and external actions; never infer permission to erase
NVS, flash a device, publish or change global settings from a host-test request.
Preserve unrelated edits. Lettered patches are additions to their numbered base.
Before editing, check the actual checkout, HEAD and existing diff/untracked files.
An archive without Git has unknown commit identity; never invent one from a handoff.
After resuming, compare task notes with the current diff and reports before acting.

Keep context proportional: find the owner with `rg`, read its implementation and
needed callers/tests. Read complete affected functions and their state/type contracts;
headers alone may omit the implementation in the matching `.cpp`. Follow only the
document sections needed for this change; a link is not proof its content was read.
State the affected behavior, invariants and selected checks briefly before editing.
Do not read all docs, index the whole repo, invoke every
skill, or run a full gate before every edit. Batch independent reads. After a
failure, inspect its raw log and rerun the affected checks once the cause changes.
Keep full evidence on disk; summarize failure, exit code, source, coverage and log
path. A focused PASS is partial, a timeout is not a sanitizer finding, and an
unavailable required check is not PASS. Expected injected ERR lines are judged
by test assertions/exit status. Do not hide failures or weaken gates to save tokens.

## Firmware invariants (when changing firmware)

- One owner for active orientation, bias, calibration and recovery state.
- Preserve frames, units, monotonic sample time and gyro propagation. Missing,
  stale or invalid accel/mag must not become fresh or block gyro indefinitely.
- Keep bounded recovery/probes. Clean observable evidence must permit recovery;
  elapsed time alone must never force acceptance of dynamic accel or interference.
- Preserve finite/valid quaternion guards, sample ordering and atomic FIFO batches.
- Validate immutable config/calibration candidates before persistence and active
  apply; retain recoverable old-good. Volatile preview must be explicit.
- No new heap/logging/blocking I/O in the hot path. No ODR/fusion-cadence decrease,
  estimator replacement, threshold weakening or raw-ABI migration as a side effect.
- Do not change packet mode, target toolchain or partitions in a DEV patch.

## Documentation and test growth

Follow [maintenance rules](docs/development/maintenance.md). Update the existing owner;
no patch-number docs/tests, per-patch reports or test suites in tools/. New files
need a distinct contract and registration/index entry. Keep within reviewed budgets
in tools/maintenance_limits.json; never auto-increase them or weaken coverage.
Do not require historical report text, duplicate builds, or source comments as proof
of behavior. Preserve ABI/sanitizer variants and fault scenarios when consolidating.

## Read only the route needed

- Owners, affected tests and evidence level: [test map](docs/development/test_map.md).
- Windows/Conda or WSL restart/sync: [session guide](docs/development/session.md).
- Test IDs, summaries and reruns: [runner guide](docs/development/runners.md).
- Frames/math: [coordinate frames](docs/architecture/coordinate_frames.md).
- State ownership: [architecture](docs/architecture/ownership.md).
- Preparation status/remaining work: [status](docs/status.md).
- Bounded tasks, context recovery and result handoff: [agent workflow](docs/development/agent_tools.md).

Use explicit Python/compiler paths from the session guide when PATH is ambiguous.
Keep PowerShell and Bash syntax separate; shell variables do not survive a new shell.
If required context/tools are unavailable, report the blocked scope and preserve
completed work; do not substitute guesses, installs or a weaker acceptance gate.
Runner defaults are sequential, bounded and fresh per run. Full/release gates
remain required for their claimed scope; run them when that scope is requested
or the changed contract needs them, not repeatedly for unrelated documentation.

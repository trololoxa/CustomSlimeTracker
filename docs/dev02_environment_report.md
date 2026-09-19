# DEV-02 — environment setup and doctor

## Base and scope

Additive after DEV-01 (`d7703cd2b26a793173d8e24d4b7dafb3185e5c4c`) on the
separate Codex preparation branch. No firmware source, native production-test
assertions, sensor rates, estimator/gates, persisted format, PlatformIO target
pins or partitions change. This does not close any A/S/F firmware defect ID.

## Problem and changed contract

Before DEV-02 there was no single bounded environment check distinguishing a
working host executable, effective sanitizer runtime and installed ESP32 tools.
A found compiler or installed Python package could be mistaken for readiness.

- `tools/doctor.py` adds explicit host/sanitized/firmware profiles, selected binary
  paths, C++20 compile/link/run proof, Core/package version checks, RISC-V target
  identity/object compile and optional tool inventory. Per-command deadlines,
  no retries, no implicit compiler substitution, no installs or hardware access.
- Sanitizer behavior is delegated to DEV-01's existing executable probes. A
  requested unsupported sanitizer remains FAIL. Optional debugger absence stays
  SKIP; `--require-tool` makes that tool required. Version inventory alone does
  not prove a useful debugging session.
- Reports retain raw command evidence in unique ignored JSON files and print
  compact summaries. Credentials/environment variables are not dumped. Report
  retention and a general selective test runner belong to DEV-03.
- `tools/requirements-platformio.txt` pins host Core 6.1.18 as a bootstrap input.
  `tools/lock_dev_wheels.py` derives SHA-256 requirements from a platform-local
  wheelhouse, validates bounded unambiguous package metadata, rejects conflicting
  versions and refuses to overwrite an accepted lock. It does not resolve or
  install dependencies. Offline pip install plus pip check is the closure proof.
- Windows/WSL instructions separate Python environments and build caches and
  distinguish target compiler readiness from actual firmware link/target smoke.
  No tool/plugin permission expansion is introduced.
- DEV-02 self-tests are registered in existing check_all. Missing optional tools
  are not made a new default check_all failure.

## Verification performed

On Linux x86_64, Python 3.12.13, GCC 13.3.0, Git 2.51.1:

| Check | Result |
| --- | --- |
| DEV-02 behavioral self-tests | 23 tests PASS after final metadata ambiguity check |
| Existing check_all aggregation tests | 9 tests PASS; synthetic FAIL/timeout messages are expected fixtures |
| Python syntax of changed tools/check_all | PASS |
| Existing documentation validator | PASS |
| Actual doctor host | PASS; dirty worktree recorded as WARN |
| Actual ASan+UBSan probes inside doctor | PASS: clean and deliberate faults detected |
| Actual doctor sanitized | FAIL: LSan clean fixture cannot read `/proc/<pid>/task`; no fallback |
| Actual doctor firmware | FAIL: PlatformIO executable absent; no fake target PASS |

The actual doctor profiles were each run once. A small self-test assertion about
check ordering was corrected during development; runtime checks continue through
independent package diagnostics. Final targeted self-tests were rerun after the
metadata guard. No full native suite, check_all all-profile builds or repeated
firmware builds were requested for this tooling-only change.

Reports/logs in the patch bundle preserve expected environment failures as
failures, alongside passing test results. The doctor snapshots precede the final
metadata-only lock guard and documentation edits; their compiler/sanitizer/target
check implementation is the delivered version.

## Review and limits

Review covered missing tools, wrong explicit paths, malformed JSON/config,
package mismatch/missing manifests, wrong target ABI, compile success without
artifact, wrong execution proof, timeout/launch error, required/optional tool
semantics, report exit status, invalid deadlines, wheel tampering/hash change,
ambiguous/missing/oversized metadata, duplicate package versions and existing
lock preservation. Hardware progress owners and hot paths are untouched; no
firmware CPU/stack/current improvement or regression measurement is claimed.

Not verified here:

- Real Windows/UCRT64 or WSL execution. These have instructions and portable
  behavioral tests, not claimed local acceptance.
- PlatformIO install, dependency closure/offline recreation, target manifests
  against an actual installed Core, ESP32 firmware build/link or board smoke.
  The environment cancelled network approval for the initial pip download;
  no bypass or replacement installation was attempted. No resolved wheelhouse
  or full dependency lock is included in this deliverable.
- LSan, symbolized stack traces, target GDB/OpenOCD attach, USB/COM bridge, flash,
  sleep, NVS, Server interoperability or release readiness.
- Bit-identical OS/Python/GCC or offline PlatformIO package reproduction. The
  generated wheel lock covers saved pip artifacts only. Preserve validated
  environment snapshots separately when reproducibility is required.

Firmware package inspection currently requires canonical Core package paths and
literal manifest versions; alternate layouts fail explicitly. Metadata/version
checks are not a cryptographic package provenance proof. User-specific paths in
raw reports are visible and can be reviewed before sharing. Python venvs,
wheelhouses and reports are local ignored data, not repository dependencies.

## Next step

Use `docs/dev_environment.md` to validate the intended Windows/WSL workstation.
DEV-03 can then add selective runners and retained compact failure summaries.
DEV-04 owns project AGENTS.md/compilation database; DEV-06 owns real debugger and
safe device workflows. None of these is represented as completed by DEV-02.

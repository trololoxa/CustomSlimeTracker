# 0025b remote CLI transport parity report

## 1. Scope and predecessor

Predecessor: `0025a_cable_free_diagnostic_capture_hardening`. Base is the
provided firmware archive containing 0024, 0025 and 0025a.

This patch removes transport-based authorization and limits from the CLI, makes
TCP command behavior match USB in Debug/ProductionDiag, permits dirty
ProductionDiag identity in cable-free capture, and makes the capture duration
range match the firmware USB range.

## 2. Confirmed defects

- `TrackerCommandDispatcher` rejected every remote command absent from an exact
  diagnostic allowlist before normal dispatch.
- TCP used 20 Hz and 900-second limits while USB used 200 Hz and 21600 seconds.
- TCP could not force-stop tests or control a log owned by another session, while
  USB could.
- `capture_telnet_log.py` rejected all dirty firmware even though firmware
  already emitted a deterministic full commit plus worktree fingerprint.
- the host tool independently enforced 60..900 seconds.
- ProductionDiag compiled `tracker_config_print.cpp` while inheriting compact
  inline config-print definitions, causing a same-translation-unit redefinition.

## 3. Implementation

- removed `trackerRemoteDiagnosticCommandAllowed` and its dispatcher rejection;
- made USB/TCP use common profile-specific help and exposed origin only as
  diagnostics;
- unified log rate at 1..200 Hz and test duration at 1..21600 seconds;
- made stop/log control origin-independent while retaining owner metadata for
  stream routing and disconnect cleanup;
- added strict clean-or-dirty capture identity validation. Dirty builds require
  exact `<40-hex-head>+<8-hex-worktree>-dirty` agreement;
- changed host capture duration validation to 1..21600 seconds;
- guarded full config-print definitions with
  `TRACKER_ENABLE_FULL_CONFIG_PRINT`;
- restored the required root `.gitattributes` whitespace contract missing from
  the provided archive (`cr-at-eol`, without normalizing existing files);
- updated native/source-policy regressions and canonical documentation.

## 4. Preserved invariants

- Production and Slim still compile the TCP listener out.
- TCP remains one-client, unauthenticated, fixed-buffered and non-blocking.
- Telnet IAC filtering, five-second host NOP, 30-second application lease,
  bounded output queues and disconnect cleanup remain unchanged.
- LOGVER3 schema, packet formats, NVS/config/calibration schemas, tracking, AHRS,
  FIFO, SlimeVR UDP and persistent defaults are unchanged.
- build identity is not weakened: dirty content is accepted only when its
  deterministic fingerprint is self-consistent and is retained in the manifest.

## 5. Compatibility and security

CLI compatibility intentionally changes: every command compiled into
Debug/ProductionDiag is reachable over TCP, including destructive and persistent
commands. The listener is unauthenticated; these images must be used only on a
trusted isolated network. Production remains the locked-down product image by
source exclusion, not by a runtime allowlist.

## 6. Regression coverage

- native transport-parity boundaries for remote 200 Hz, 21600 seconds and
  force-stop behavior;
- clean and dirty host identity acceptance plus mismatched dirty fingerprint
  rejection;
- source-policy rejection of any restored allowlist, remote-only limits or
  remote-only errors;
- retained 0025a lease/log/manifest policy.

## 7. Verification and acceptance

Executed on the provided archive:

```text
PASS  python3 tools/test_0025b_remote_cli_transport_parity_policy.py
PASS  python3 tools/test_capture_telnet_log.py  (13 tests)
PASS  python3 tools/test_0025a_diagnostic_capture_policy.py
PASS  focused native test_cli_transport_parity with -Werror
PASS  tracker_config_print.cpp compile with FULL_CONFIG_PRINT=0 and =1
PASS  validate_source_filters.py
PASS  validate_profile_matrix.py
PASS  validate_documentation.py
PASS  git diff --check
```

`tools/check_all.py --skip-native --skip-pio` passed the modified policies and
all checks reached before the execution environment timeout. The complete native
runner compiled every modified production translation unit without error, then
the environment timed out later in the long unrelated native matrix. PlatformIO
is not installed in this environment, so target linking and hardware/HIL remain
explicitly unverified.

Required release-side rerun from the repository root:

```bash
python3 tools/run_standalone_tests.py --clean --extra-cxxflag=-Werror
python3 tools/check_all.py --clean --require-pio
```

Target smoke on ProductionDiag:

```text
version
help
config print
test static 21600
test stop
log rate 200
log off
```

Through TCP verify that a formerly blocked mutation, for example `perf on` or a
non-destructive config command, reaches the normal dispatcher. Destructive/NVS
commands should only be exercised on a disposable test tracker.

Host dirty-build smoke:

```bash
python3 tools/capture_telnet_log.py --host <ip> --seconds 600 --rate 20 \
  --mode full --capture runtime --output dirty_runtime.log
```

The manifest must contain `build_dirty=yes` and the same
`<head>+<worktree>-dirty` identity printed by `version`.

## 8. Rollback

Reverse-apply this patch. Rollback restores the diagnostic allowlist, USB/TCP
limit split and clean-only capture preflight. No persistent migration is needed.

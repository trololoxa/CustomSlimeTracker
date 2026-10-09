# Current status and limitations

Runtime contracts live in [architecture](architecture/implementation.md), interfaces
in [reference](reference/cli.md), and execution instructions in [testing](development/testing.md).
This page records readiness, not a chronological patch history.

## Verified evidence scope

User artifact `verification-dev05-20260923.zip` identifies clean source
`4ac5c0c171913060582cd93d9cef2fd6665fae86`: Windows native, WSL ASan/UBSan and WSL
LSan each passed 54/54 executables; five target profiles and five replay steps
passed. The full Windows gate failed three host stack policies below. These are
historical measurements, not certification of subsequently changed sources.

Artifact `dev-verify-hs_yt0s0.zip`, source
`63c0da8d4015ba7665cb7b620b8945cbde44863c`: Windows native 54/54, replay 5/5 and
firmware profiles 5/5 passed. Documentation failed on a missing per-run report;
WSL suites did not execute because Linux Git saw the mounted Windows checkout as
dirty. Source inventories before/after matched. End-to-end cross-platform workflow
acceptance remains open; differing Git EOL settings are a hypothesis, not a proved fix.

This maintenance change removes active documentation's dependency on that absent
report. The report need not be restored to the source tree. New migration evidence
belongs in the delivered verification artifacts, not another mandatory page here.

## Retained failures and open contracts

Windows GCC 16.2 observations, deferred by the owner to firmware work:

| Current check | Measurement | Limit |
| --- | --- | --- |
| `test_magnetic_hotpath_budget` | public yaw update stack 112 B | 96 B |
| `test_yaw_microsoft_abi_budget` | Microsoft ABI public yaw update 112 B | 96 B |
| `test_imu_hotpath_budget` | TrackerApp::loop 272 B | 256 B |

These do not prove the corresponding ESP32 stack sizes. Consolidation never
waives them or increases their budgets. ABI/optimization variants remain separate.

Five module-level fusion probes remain OPEN: large-tilt reacquisition,
invalid-accel gyro propagation, missing-accel statistics, yaw-callback preservation
of accel evidence, and 180-degree magnetic yaw reacquisition. See
[fusion validation](development/fusion_validation.md). No firmware defect is
closed by renaming or moving its test.

## Next acceptance

- Documentation/test maintenance: Windows host run `check-all-y77rhkpg` passed
  native/replay and all checks except the three retained stack failures. This covers
  the structural follow-up too. No additional Windows failure was reported.
- Windows focused maintenance evidence `check-all-91q6tblu`: reporting, aggregation,
  maintenance structure, test structure and documentation passed (5/5). This closes
  the reported Cyrillic fixture failure, not the three firmware budgets above.
- Missing Linux stack records and unfinished JSON: root causes remain unproved.
  Narrow infrastructure guards reject unfinished native evidence and retain raw
  stack inputs when a function is missing; they do not reconstruct a lost PASS.
  Do not hold the roadmap for repeated structural rewrites or speculative reruns.
- Cross-platform verification: explain mounted-checkout Git differences and prove
  a complete real Windows→WSL run; do not disable dirty-source checks.
- DEV-06: owner-approved scoped bench closeout on 2026-10-09. The accepted
  Windows device-tooling subset passed 4/4 (`check-all-jjqzelme`); no full-gate
  rerun is claimed. Current hardware evidence identifies clean installed source
  `900f3f5c5eb26af765aa6f5f6f41796719fc137f`, environment
  `BOARD_LOLIN_C3_MINI_USB_DIAG`, USB serial `E8:3D:C1:93:3D:34`.
  Supplied flash report confirms esptool 4.9.0 app-only write and SHA-256 read-back;
  the supplied pre-debug smoke also passed. Bootloader/partition/NVS writes were
  not requested. The archived checkout's own Git identity is not inferred from
  these device reports.
  Live GDB/OpenOCD evidence confirms attach without reset, halt, registers, raw
  stack read, partial backtrace, continue and interrupt. Installed-build ELF
  symbolization resolves `0x4206b412` to Ahrs6Dof::update and `0x42050f22` to
  runtimeBiasUpdateEstimator/inlined Vec3::isFinite. The owner reported clean
  `disconnect`/GDB exit 0 and subsequent smoke PASS; their final raw logs/summary
  were not attached. Earlier supplied post-debug smoke has three healthy 6D
  observations, sample/pose progress and no new recovery/fault increments.
  Use the [verified target session](development/session.md#esp32-c3-live-debug-session).
  Known limitations remain explicit: GDB `detach` asserts even with `hwthread`;
  the accepted exit is `disconnect` with OpenOCD resume-on-disconnect. The old
  default attach handler resets for memprot; flash-disabled/no-reset attach is
  required. One supplied failed smoke contains `unknown command`, not silence;
  its cause is unresolved and a later PASS does not erase that failure.
  Two successful sensor recoveries preceded the supplied healthy observation
  window; attribution to CPU pauses is plausible, not proved. Full unwinding,
  flash/software breakpoints and RTOS task inspection are not accepted here.
  Boot/reset/safe-mode capture remains an outstanding device-guide evidence item;
  no supplied capture proves it, and this scoped closeout does not waive it.
  Real crash-log capture, core dumps, 9D/physical accuracy, WCET, linear-accel frame
  calibration, sleep/wake and continuous USB availability are not claimed.
  Preserve the saved ELF/BIN/manifest and available logs outside ephemeral build
  storage. Manual driver-switching usability is unresolved; a working driver
  must not be switched routinely. Do not reflash or repeat accepted checks merely
  to document this closeout. DEV-07 CI/release/Server work is next.
- Release: real reviewed LOGVER3/golden inputs, current Server interoperability,
  persistence/rollback and relevant board smoke/timing.
- Absolute physical accuracy and additional math scenarios remain unverified.

The [roadmap](roadmap.md) separates development tooling from firmware changes.

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
- Device workflow now provides explicit USB identity/lock, manifest-checked app-only
  write/read-back, bounded serial smoke/capture and offline ELF symbolization.
  Windows focused device/tooling checks passed (six checks, `check-all-y4qa0n71`).
  USB smoke `device-yrdfoliy` failed on incomplete health; supplied serial evidence
  contains 135 dropped lines. The optional USB diagnostic queue/reset variant
  addresses this bench path. Its Windows target build passed (RAM 185156 B,
  flash 1768570 B). Flash attempt `device-tup_0jnb` entered ROM/stub without
  buttons, then failed on CHANGE_BAUDRATE before any app write. Native USB now
  keeps initial 115200 baud. esptool 4.9.0 completed app write/SHA-256 read-back
  (`device-0_pazai9`, source identity suffix `0d855a37-dirty`), with no NVS/bootloader
  writes requested. After a separate manual update disabling USB_DIAG sleep, smoke
  `device-47447np4` passed 3/3 serial/6D observations at ~3h15m uptime
  (`46e1088c-dirty`); no dropped log lines or new sensor faults were observed.
  These are different builds, not one manifest-matched end-to-end acceptance.
  Mag, physical accuracy, linear acceleration/frame calibration, sleep/wake and
  continuous USB availability remain unverified by this sampled smoke. Controlled
  uploader focused Windows checks passed (`check-all-77gyb5s7`). Module-based
  flash/read-back passed (`device-_zp1ls82`, identity suffix `1e7dee4a-dirty`),
  then smoke passed (`device-4cfo6uq_`) after one initial framing failure and one
  occupied-port failure. Raw `device-pa3vbqmc` evidence shows a complete version
  response glued to a truncated boot-banner line. Initial-header resynchronization
  fixes this host parser case. The owner's subsequent handoff accepts DEV-06d:
  Windows focused `test_device_workflow`, `test_maintenance_structure`,
  `validate_test_structure` and `validate_documentation` passed 4/4
  (`check-all-jjqzelme`); installed-build-matched smoke passed three observations
  with `TRACKING_6DOF` (`device-q7umf6v_`). These are owner-reported results; the
  underlying reports and full installed-build identity are not included in the
  current source archive. They are not a new full gate or 9D/accuracy acceptance.
  Do not repeat the accepted flash/read-back or focused checks without a changed
  contract or new failure. Fake-I/O evidence does not certify hardware.
  The owner subsequently reported driver application and an OpenOCD connection;
  full live-debugger acceptance remains open: target GDB attach, halt, registers/
  stack, resume and detach; symbolize an address using the installed build's ELF;
  then verify ordinary 6D smoke recovery. Symbolizing a halted CPU address is not
  real crash-log capture acceptance; no deliberate crash is required. Hardware
  work is deferred until the bench tracker is available. Manual driver selection
  was reported as inconvenient; repeated switching and its cause are unverified.
  Follow the [device guide](development/device_smoke.md).
- Release: real reviewed LOGVER3/golden inputs, current Server interoperability,
  persistence/rollback and relevant board smoke/timing.
- Absolute physical accuracy and additional math scenarios remain unverified.

The [roadmap](roadmap.md) separates development tooling from firmware changes.

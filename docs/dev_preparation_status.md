# Codex preparation patch branch

This is a development-tools series, separate from firmware roadmap 0029–0038.
The branch starts from the supplied source snapshot through 0028d. Patch DEV-01
is additive to that snapshot; later DEV patches are additive to their predecessor.
Do not infer hardware/release acceptance of 0028d from this development work.

| Patch | Contract | Status |
| --- | --- | --- |
| DEV-01 | Assertions and executable sanitizer capability evidence | Implemented; see DEV-01 report for verification limits |
| DEV-02 | Windows/Linux setup, wheel hash locks, environment doctor | Implemented; Windows and WSL evidence received; see [DEV-04 report](dev04_workflow_report.md) for scope |
| DEV-03 | Selective runners, raw logs, compact summaries, safe reuse | Implemented; shared objects within a fresh run and failed-ID reuse only; see DEV-03 report |
| DEV-03a | Bounded Windows report replacement and progress notices | Implemented; user Windows focused acceptance passed; see [DEV-04 report](dev04_workflow_report.md) |
| DEV-04 | Project AGENTS.md, current documentation, owners/test map, compilation database | Implemented; Windows checks and native clangd navigation accepted; WSL focused results received; workflow acceptance closed; see [latest evidence](dev05_report.md) |
| DEV-04a | Mixed-EOL application guidance and flexible model choice | Implemented; local Codex/Windows acceptance received; see [acceptance](dev04_workflow_report.md) |
| DEV-04b | clangd session setup and user acceptance record | Documentation only; additive after DEV-04a; see [session guide](dev_session.md) |
| DEV-05 | Independent math references and algorithm/scenario replay | Implemented; Linux native/sanitizer and negative tests passed; Windows acceptance pending; five firmware contracts remain OPEN; see [report](dev05_report.md) |
| DEV-05a | Correct first recovery and verify actual gyro propagation | Implemented; selected ASan/UBSan and negative tests passed; user acceptance pending; see [report](dev05a_report.md) |
| DEV-06 | Device identify/flash/smoke/capture, locking, debugger, crash evidence | Pending |
| DEV-07 | CI, artifacts, Server interoperability, rollback procedures | Pending |

Plugin selection follows measured project needs. Serena is the first navigation
candidate after compilation database setup; RTK is optional for noisy commands.
DEV-04 also installs no plugin and changes no account/global permissions.

Each patch must record source scope, actual verification, environment limits,
and the next task. Extend existing tools rather than create parallel runners.
Existing gyro/accel/mag, persistence, recovery and hot-path contracts still apply.
Do not change firmware behavior or toolchain as a side effect of preparation.

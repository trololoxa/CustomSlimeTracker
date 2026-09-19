# Codex preparation patch branch

This is a development-tools series, separate from firmware roadmap 0029–0038.
The branch starts from the supplied source snapshot through 0028d. Patch DEV-01
is additive to that snapshot; later DEV patches are additive to their predecessor.
Do not infer hardware/release acceptance of 0028d from this development work.

| Patch | Contract | Status |
| --- | --- | --- |
| DEV-01 | Assertions and executable sanitizer capability evidence | Implemented; see DEV-01 report for verification limits |
| DEV-02 | Windows/Linux setup, wheel hash locks, environment doctor | Implemented; target installation and Windows/WSL execution still require local verification; see DEV-02 report |
| DEV-03 | Selective runners, raw logs, compact summaries, safe reuse | Pending |
| DEV-04 | Project AGENTS.md, current documentation, owners/test map, compilation database | Pending |
| DEV-05 | Independent math references and algorithm/scenario replay | Pending |
| DEV-06 | Device identify/flash/smoke/capture, locking, debugger, crash evidence | Pending |
| DEV-07 | CI, artifacts, Server interoperability, rollback procedures | Pending |

Plugin selection follows measured project needs. Serena is the first navigation
candidate after compilation database setup; RTK is optional for noisy commands.
No plugin installation or permission expansion is part of DEV-01.

Each patch must record source scope, actual verification, environment limits,
and the next task. Extend existing tools rather than create parallel runners.
Existing gyro/accel/mag, persistence, recovery and hot-path contracts still apply.
Do not change firmware behavior or toolchain as a side effect of preparation.

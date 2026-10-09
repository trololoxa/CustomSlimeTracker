# Roadmap

This is future work, not implementation evidence. [Status](status.md) owns current
readiness and failures. Patch numbers may identify deliveries in Git; they do not
create documentation/test owners.

## Development environment

1. Structural maintenance has Windows host acceptance with the three retained
   firmware stack failures. Keep remaining evidence diagnostics scoped; complete
   real Windows/WSL workflow acceptance when that workflow is next needed.
2. DEV-06 device workflow: scoped bench acceptance is closed with the limitations
   in [status](status.md). Reuse the verified live-debug session; do not repeat the
   accepted flash/read-back or change drivers/toolchains without a new reason.
   Retain the outstanding boot-capture evidence item; no crash capture is claimed.
   Preserve calibration/NVS by default; destructive operations require authorization.
3. DEV-07 CI/release is next: reproducible artifacts and identities, current SlimeVR Server cross-test,
   release fixture requirements and rollback procedures.
4. Evaluate Codex on fixed tasks: documentation-only edit, known AHRS math fault,
   recovery fault and full verification. Record correctness, selected checks, extra
   reads/reruns, elapsed time and actual tokens where the client exposes them.
   Lower cost is beneficial only with the same required quality/coverage.

No plugin, model change, parallel-agent work or hardware operation is implied by
this plan. Existing environment capabilities and user permissions govern execution.

## Firmware sequence and dependencies

Preserve the agreed ordering: sensor recovery/config transactions precede fusion
changes; network session truth precedes adaptive TPS and long-sleep refresh.

| Area | Remaining intended result |
| --- | --- |
| Sensor recovery / config transactions | Final current-source and target acceptance; retain bounded recovery, immutable candidates and old-good persistence |
| 6D accel | Gyro continuity; missing/dynamic samples do not poison statistics; clean gravity reacquires despite accumulated tilt drift |
| 9D magnetics | Fresh stream/references, bounded rearm and gradual yaw reacquisition with trusted tilt; no acceptance by timeout |
| Gyro bias and temperature | One effective-bias owner; fresh temperature, RAM session trim, explicit calibration lifecycle |
| Network/session/storage | Valid-packet liveness, nonblocking DNS/association, recoverable persistence and current Server compatibility |
| Adaptive output | Independent motion demand and network pressure limits; source/config ceilings; fractional microsecond scheduling; no sensor ODR reduction |
| Long-sleep refresh | One planned restart after accumulated automatic no-server sleep and a proven stable Server session, with blockers and a one-shot cookie |
| Power/observability | Battery freshness, explicit scans and yielding only with proved slack |
| Storage/security/release | Explicit codecs, recoverable migration, diagnostic trust boundaries and release gates |
| Platform migration | Separate toolchain/partition change after stable storage contracts, with board checks |
| Optional fusion research | Only after stable basics and a measured residual limitation; no estimator replacement or second active bias/tilt/yaw owner |

The [math plan](development/math_coverage.md) defines scenario directions and
measurement limits. New physical data is requested only for a changed claim that
existing fixtures cannot test. Firmware stack-policy failures are not silenced
by development-environment work.

# 0027f progress epoch contract report

## Scope and ordering

0027f is additive after 0027e. It closes two review findings without changing
production executable behavior: one stale class comment and one missing native
regression for the suppress/resume epoch boundary. Sensor ODR, FIFO operations,
watchdog deadlines, recovery behavior, AHRS, accel/mag gates, networking,
persistence and output cadence are unchanged.

## Changed contract

There is no runtime contract change. Documentation now states the existing
contract precisely:

- IRQ and FIFO-drain producers publish timestamps already observed in the local
  MCU clock domain;
- accepted-gyro and orientation producers publish timestamp-free sequence
  edges from the per-sample path;
- the bounded app watchdog service timestamps newly observed sequence edges;
- leaving the final active suppress reason starts a new epoch and snapshots the
  current sequences, so evidence accumulated during suppression is not reused.

## Regression coverage

The new native case holds two suppress reasons simultaneously, publishes
gyro/orientation progress, clears only one reason, publishes more progress, and
then clears the final reason. It proves that:

1. a partial unsuppress remains suppressed;
2. the final resume clears old observation timestamps and arms a fresh epoch;
3. a post-resume drain without post-resume gyro reports `NoAcceptedGyro`;
4. a post-resume gyro without post-resume orientation reports
   `NoOrientationPublication`;
5. new gyro and orientation edges restore the healthy verdict.

The focused policy compiles and runs the complete sensor-liveness native test,
checks the corrected source contract and verifies aggregate-gate registration.
Direct diff inspection confirms that the only production-source edit is a
comment.

## Target evidence inherited by closure

The ProductionDiag target smoke after 0027e completed successfully:

- an undisturbed run retained zero sensor-progress faults and zero FIFO data-loss
  faults while samples and publications advanced;
- one manual FIFO reset produced one request and one success, returned through
  `RECOVERING` to tracking and reacquired tilt;
- tap hardware was not repeatedly configured or logged by FIFO-only recovery;
- a normal software reboot did not increment crash boots or enter safe mode;
- accel and magnetometer/yaw correction remained passable after recovery;
- motion light sleep was verified after removing its intentional remote-console
  blocker.

The missing core-dump partition warning is pre-existing partition observability
debt and is not changed here. Physical no-IRQ fault injection and target task
stack high-water remain unverified.

## Verification

Run exactly one focused command:

```text
python3 tools/test_0027f_progress_epoch_contract_policy.py
```

No new target flash is required because 0027f changes only comments, tests and
documentation.

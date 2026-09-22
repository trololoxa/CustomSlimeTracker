# DEV-03a — Windows report replacement and bounded progress notices

Additive **after DEV-03**. Only host tooling and documentation change; firmware,
packet modes, native C++ sources, compiler flags, sanitizer selection and target
pins remain unchanged.

## Evidence and defects

The user's Windows focused native run raised WinError 5 while replacing
`summary.tmp` with `summary.json`. That report update aborted the runner before
its next child command. The trace proves access denial on replacement; it does
not identify an antivirus, editor or other particular process as the cause.

A subsequent complete DEV-03 run passed. The supplied `summary(1).json` records
72 commands, all completed with exit 0, empty failures/warnings, clean source
commit `548cbe3401732671ecf3a892cc8e50ec9abf1428`, and the default gate invoked
with `--clean --require-pio`. Native used `--sanitizer none` and took 90.578 s;
all five PIO build commands passed. This is Windows acceptance evidence for the
predecessor, not DEV-03a or a complete sanitizer/release gate. The original JSON
is retained in the delivery bundle.

DEV-03 redirected command output to logs and normally printed results only at
completion. This made a healthy long-running command look stalled. Verbose also
printed logs only after completion, so it did not solve the waiting interval.

## Changed contract and ownership

`GateReport` still owns report files and console summaries. Atomic replacement
retries only OSError values carrying Windows error 5, 32 or 33. There are seven
attempts total, with delays 10/20/40/80/160/320 ms (630 ms requested sleep total).
The same prepared temporary file is retried. Old JSON is never unlinked or
truncated before a successful replacement. Successful recovery prints a short
notice. Permanent failure propagates; it is not silently converted to PASS.

Disk-full, non-Windows permission errors, unexpected errors and failure while
writing the temporary candidate remain errors without retries. The candidate
is retained on replacement failure for diagnosis. This is not a guarantee of
fsync durability or immunity to locks lasting longer than the retry budget.
No command/test/build is rerun by this mechanism.

`run_bounded_process` retains process-tree cleanup and adds an optional progress
callback. Without a callback its old wait path is unchanged. With one, repeated
communicate waits share one monotonic absolute deadline. Collected output is
returned once, without duplicate chunks; timeout/interrupt/callback failure
still invokes process-tree cleanup. There are no worker threads or extra
processes for progress reporting.

The aggregate runner prints `RUN` before each child command. Native runs suppress
per-object startup noise and print a current command periodically. While a child
is running, the reporter polls at ten-second intervals and emits `WAIT` only
when at least thirty seconds have passed since its last notice (normally a
30–40 second cadence, subject to OS scheduling). Notices name the command/log;
they are liveness notices, not percent completion or proof of useful progress.
Short commands do not incur a ten-second delay: communicate returns immediately
on completion. Raw logs, excerpts and verbose-completed-log behavior remain.

This adds host wait polling and occasional console lines, not firmware runtime
work. Deadlines, rate/gate budgets and release requirements are not relaxed.

## Verification

- Focused seven-group check: DEV-03a, DEV-03, runtime process policy, aggregation,
  standalone runner, DEV-01 and DEV-02: PASS. DEV-03a has 14 cases: 13 passed on
  Linux and one actual Windows sharing-lock test was skipped. Report directory:
  `check-all-da5_1_a2`.
- Fault injection covers Windows 5/32/33 recovery, permanent locks, old/candidate
  JSON preservation, non-retryable errors, pre-launch refusal and exactly-once
  command execution when post-command replacement temporarily fails.
- Progress checks cover the absolute deadline with a fake clock, actual child
  stdout across multiple waits and nonzero exit, callback failure/interrupt
  cleanup, invalid intervals, and actual descendant termination on timeout.
  Console tests verify pre-launch notices and throttling.
- One real selected native run: `test_core_math_ahrs` and
  `test_sensor_calibration`: 2/2 PASS, sanitizer none. A periodic native `RUN`
  appeared during common-object compilation. Report directory: `native-qocu9iwr`.
- Documentation/syntax/whitespace and patch-application verification are recorded
  separately in the delivered validation evidence.

Not verified here: actual Windows execution of this revision, on-device work,
full native/PIO/release/sanitizer matrices, persistent ACL faults and real-world
third-party lock durations. The dedicated Windows test holds a real file handle
without FILE_SHARE_DELETE, then releases it during retry; on Windows it must run
without a skip. It does not require administrator access.

## Local acceptance and next work

Apply after DEV-03, then run in the existing activated PowerShell environment:

```powershell
& $TrackerPython tools/check_all.py --check test_dev03a_reporting --check test_dev03_runners --check test_quality_gate_runtime --check validate_documentation
& $TrackerPython tools/run_standalone_tests.py --test test_core_math_ahrs --test test_sensor_calibration
```

No full Windows matrix rerun is requested solely for this fix. A test failure
requires its own investigation; preserve the corresponding run directory.
ASan+UBSan and separate LSan remain to be run on a suitable Linux/WSL host.
Release fixtures and hardware/debugger acceptance remain separate requirements.

DEV-04 must document/reproduce the verified Miniconda/PowerShell setup, explicit
Python/compiler/PIO paths, user-site isolation and local H: storage choices.
Local machine paths must not become hardcoded runner defaults. Conda shell-hook
repair and environment installation are outside this patch.

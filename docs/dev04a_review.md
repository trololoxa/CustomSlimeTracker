# DEV-04a: application correction and model choice

Additive AFTER DEV-04, not a replacement. This patch changes instructions/docs
only; it makes no Python, firmware, build-profile or dependency changes.

## Reproduced application failure

The user's attached `.gitignore` and the archived DEV-04 base have identical
text after CRLF/LF normalization. Archive: 31 CRLF lines. Attachment: 26 CRLF
and 5 LF lines. Plain `git apply --check` reproduced `.gitignore:29` failure.
With `--ignore-space-change`, check and application succeeded. All bytes of the
attached file were preserved and the intended three-line suffix was appended.
No global autocrlf change, forced reset, reject files or removal of ignore rules.
Use the exact base-then-addendum sequence from this artifact's APPLY_RU.md.

The original packaging check covered the exact archive, but not this mixed-EOL
worktree. That portability gap was missed. Application evidence now covers the
original archive and the actual attached `.gitignore`, through DEV-04 + DEV-04a.

## Requirement review

| DEV-04 requirement | Implementation/evidence | Remaining acceptance |
| --- | --- | --- |
| Project instructions | Short root AGENTS.md; task-specific links and firmware invariants | Confirm local Codex loads it; global/user rules were not supplied |
| Low token overhead | Focused selection, existing compact logs, no compulsory doc/skill cascade | Actual token savings not measured; no percentage promised |
| Owner/test routing | Current source paths and 39 explicit IDs checked | Map is an entry point, not exhaustive dependency analysis |
| Restartable Windows/WSL environment | Explicit tool paths and session commands | User execution of the new Windows command sequence |
| Same source in both checkouts | Bounded read-only identity helper; dirty/mismatch/error tests | Ignored inputs/dependencies not certified by Git HEAD |
| Separate build/test deadlines | Native and aggregate forwarding, legacy compatibility, negative tests | Actual Windows execution of DEV-04 remains pending |
| Compilation database | Successful real native report exported to 46 command entries | clangd navigation and target PlatformIO compiledb not exercised here |
| Completion state/documentation | Prior user evidence and missing release prerequisites recorded | DEV-05/06/07 remain separate work |

Thus the implementation scope is covered, but complete local integration
acceptance is not yet closed. Host navigation metadata is not a proof of correct
ESP32 indexing. Hardware, debugger attach, current Server cross-test and release
golden fixtures are not certified by this tooling patch.

## Regression review

Re-read native deadline dispatch/CLI compatibility, sanitizer preflight,
aggregate propagation, identity and database helpers against DEV-04 changes.
No new code regression found. Existing absolute deadline/process cleanup and
raw reporting owners are retained. A deadline failure still fails; a larger
sanitizer compile allowance does not extend test execution or the outer suite.
No firmware hot path, numerical guard, packet setting, calibration or recovery
logic was changed. Unmeasured performance is not claimed as validated.

Reran only `test_dev04_workflow` and `validate_documentation` for this revision;
previous real native smoke and policy evidence remain in the DEV-04 report.
No new tests mirroring prose or full sanitizer/build reruns were introduced.

## User-authorized model selection

Astra is now a preferred default, not a hard pin. The agent may choose another
available model for suitable work within existing permissions/client capabilities,
without repeatedly asking. It must keep project invariants and acceptance gates,
briefly identify an actual switch and resolve uncertain results with Astra or a
stronger available model. Cost/speed alone does not prove equivalent quality.
A guarantee of zero future defects is neither possible nor an acceptance test.
No global model/account configuration is changed by this documentation patch.

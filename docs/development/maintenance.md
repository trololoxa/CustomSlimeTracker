# Documentation and test maintenance

Owner: development tooling. These rules apply to every change, including agent work.
Documentation ownership is checked by `tools/validate_documentation.py`; test
inventory/growth by `tools/validate_test_structure.py`. Both and their negative
regression tests run in the normal gate. A patch number identifies a delivery, never a new owner.

## Documentation

- Start at [the index](../README.md). Architecture explains current invariants;
  reference describes interfaces; development explains reproducible workflows;
  status records current limitations; roadmap records future work.
- Update the document owning a fact. Other pages link to it. Do not copy a command
  block, readiness table or changing default into several documents.
- A new page needs a distinct audience/task and an index entry with its purpose.
  A new patch does not justify a new page. No date/patch-number reports in active docs.
- Ordinary changes use the commit/PR description for defect, changed contract,
  checks and limitations. Significant durable design decisions may use `decisions/`;
  they explain a decision, not a transcript of development.
- Logs, summaries and per-run acceptance reports belong in verification artifacts.
  Preserve release/important evidence in durable artifact storage; ignored `build/`
  is only the local working location. Active docs cannot require an ephemeral file.
- A brief status record may identify source SHA, environment, scope and artifact
  identity. Do not copy all measurements or claim current acceptance from old logs.
- Historical docs removed during migration remain in Git and the supplied historical
  document archive. Extract active contracts before retiring their only explanation.

## Tests and ownership

- `tests/native`: C++ behavior, math and state machines; names describe contracts.
- `tests/tooling`: Python tools and orchestration behavior.
- `tests/contracts`: source/build/ABI/resource constraints, not patch completion.
- `tests/support`: shared test helpers. `tests/fixtures`: reviewed test inputs.
- `tools`: reusable commands and runner infrastructure, not test suites.
- Add a scenario to the existing owner before adding a file. New files require a
  distinct contract or fixture boundary. Do not put unrelated checks in one giant file.
- Behavioral assertions compare outputs/state/deadlines with independent expected
  results. Existing source-string guards remain migration-preserved constraints;
  new prose/comment/name checks are not substitutes for behavioral coverage.
- Never require a historical report, a test's own name in documentation, or a
  literal spelling of its registry entry. Registration is checked structurally.
- One runner owns native compilation, process deadlines and reports. A full native
  suite owns ordinary functional runs. Contract suites may compile distinct stack,
  feature-profile or sanitizer variants, with their actual flags visible in logs.
- Reuse a build only within a run and only for identical inputs/compiler/flags/mode.
  ABI, optimization, sanitizer and fault scenarios are independent dimensions.
  Similar names do not prove duplication. There is no persistent result cache.
- Consolidation must retain fixtures, compiler options, limits, assertions and
  negative cases. Removing a check requires an identified replacement or explicit
  evidence of obsolescence. Fewer files/PASS lines alone prove nothing.
- Shared helpers own compiler discovery, stack parsing, source-section extraction
  and bounded contract subprocesses. Reuse `contract_checks`; direct unbounded
  `subprocess.run` in contract suites is rejected. Contract child commands have a
  600 s ceiling; shared compile/run helpers limit executable runs to 180 s. Outer
  gate deadlines remain independent; no timeout causes a retry or a PASS.
- Shared helpers own common assertions. Nontrivial identical contract function
  bodies are rejected even if renamed.
  Compiler identity is explicit: invalid CXX fails rather than selecting another ABI.

## Enforced growth limits

`tools/maintenance_limits.json` is the single budget configuration. Limits count
all Python test/support files and all native sources/headers, including nested
case headers and compile fixtures. Per-file limits apply to helpers too. The native
file ceiling includes 54 executable entrypoints plus their support files (80 total);
the separate executable ceiling remains 64 and the total line ceiling remains 16000. This replaces the old executable-only count,
not a firmware stack budget. No oversized-file exemptions remain.

Large suites keep one root executable and complete scenario functions in
`tests/native/cases/<owner>/`. A case header must be reachable from an entrypoint.
Do not include fragments of statements or split a scenario merely to fit a limit.
Helpers/headers count toward the same total; a split cannot hide growth. Root native
`test_*.cpp` files are executable owners; nested executables fail discovery rather
than silently disappearing. Python selector IDs must be unique across all registries.
Both actual runners reject ambiguous/unsupported discovery before compilation.

The Python file ceiling is 65: one additional suite owns device selection,
serial framing, smoke evidence and flash safety. These hardware-tool contracts do
not belong to the native runner or cross-platform synchronization tests. The new
suite uses fake I/O, not device access or firmware builds. Total/per-file line
budgets and every firmware resource limit stay unchanged.

Limits are review triggers enforced as failures, not a metric of test quality.
Do not compress code/prose, remove useful coverage, split arbitrary chunks, or
regenerate ceilings to make a check green. A necessary expansion changes the
budget explicitly in the same reviewed change, explaining the new contract,
why an existing owner cannot contain it, and the impact on navigation/runtime.
New documents must be indexed; all Python checks must be registered exactly once.
No patch-number file names or Python test suites in `tools` are permitted.

## Migration compatibility and acceptance

Old command selectors are temporarily translated by one `check_aliases.json` map.
They emit a deprecation notice and execute the canonical check once. No old script
wrappers or duplicate registry entries. Remove aliases in one announced cleanup
when local/CI callers have migrated. Old focused JSON reruns with retired paths
remain rejected; select current IDs instead. Historical accuracy reports remain
readable, with their original schema and source/harness fingerprints unchanged.

During this maintenance migration, firmware sources, fixtures, target flags,
thresholds and partitions stay unchanged. Existing stack violations and five
open fusion requirements remain failures/open observations. Final verification
checks affected tooling, native behavior, actual build variants and source diff;
Windows/WSL and hardware evidence remains separately scoped.

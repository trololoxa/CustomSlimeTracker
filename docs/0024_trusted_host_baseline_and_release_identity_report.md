# 0024 trusted host baseline and release identity

## Scope, base and predecessor

Base: `Upgrades` commit `631fcc4c31892d5f404033b20fce6c03a4fabc3e`.

This is one consolidated infrastructure patch after
`pre_0024ad_network_pressure_pacing_and_recovery_hardening`. It owns host test
orchestration, process/temp policy, sanitizer ownership, source/build identity,
release manifests, root project entry documentation and corresponding
regressions. It deliberately combines the former small “host gate repair” and
“release identity/manifest” roadmap items so the next functional patch starts
from one trusted acceptance boundary.

The historical `pre-0024*` names are retained. Number `0024` is assigned to this
baseline patch; the unimplemented AHRS quality wave is renumbered after the
baseline/logging series.

## Defects and evidence

On the unmodified base, the focused C++ native suite had previously passed
44/44 executables, but the aggregate host gate was not trustworthy. The exact
before command used for this patch was:

```bash
python3 tools/check_all.py --skip-native --skip-pio
```

It failed seven independent policy entries:

```text
0023gk magnetometer robust-fit acceptance policy
0023gl SlimeVR UDP TX recovery policy
pre-0024 hotpath headroom policy
pre-0024a tracking deadline policy
pre-0024ab transform cache policy
pre-0024ac IMU hotpath/slack policy
pre-0024ad network pressure policy
```

Each sanitized executable completed its behavioral assertions and then failed
inside LeakSanitizer because the execution environment is ptrace-supervised.
ASan/UBSan and leak ownership were therefore coupled: an unsupported leak phase
falsely made address/undefined-behavior coverage red.

Five successor policy scripts recursively launched predecessors that were
already explicit aggregate entries; the build-identity test was likewise
repeated from the 0023gi policy. That caused identical stack compilations and
native executables to run up to four times.

The environment had no system `/tmp`. Python `TemporaryDirectory` and GCC
therefore fell back to the repository root. Interrupted or killed runs left
`tracker-*` directories plus `cc*.o`, `cc*.s`, `cc*.res` and constructor/destructor
temporaries. Commands had no timeout, so an interrupted aggregate could leave
partial zero-length `.su` data that later looked like a missing stack symbol.

Finally, the repository had deterministic firmware identity but no root README,
no strict host/release modes and no artifact manifest binding a target binary to
its full source commit, dirty state, environment and hash.

## Root cause

- GCC AddressSanitizer links LeakSanitizer by default on Linux. Runtime
  environment flags alone are not reliable under every supervised launcher.
- Historical policy scripts treated predecessor validation as a nested call
  rather than independent orchestration ownership.
- Temporary paths relied on host-global defaults that were absent in this
  container.
- `subprocess.run` calls were unbounded.
- Release acceptance was assembled from optional skip flags and short console
  identity rather than a fail-closed mode and machine-readable artifact record.

## Implementation and ownership

### Test process and temporary-file policy

`tools/quality_gate_runtime.py` owns repository-local temporary roots and child
sanitizer environments. Every policy that creates temporary files uses
`build/tmp/quality-gates`; aggregate children, native compilation and PlatformIO
receive valid `TMPDIR`, `TEMP` and `TMP` values. Legacy root artifact patterns
are ignored for existing dirty workspaces, but no new gate writes them there.

The aggregate runner bounds validators/policies/replay, the full native suite,
each native compile/link/test and every PlatformIO environment. Timeout exit
code is consistently `124` and remains visible in the aggregate failure list.
Timeout and exceptional unwinding reap the launched tree: POSIX uses the new
session's process group; Windows uses `taskkill /T /F` with direct process kill
as fallback. A terminated compiler therefore cannot leave `cc1plus`, assembler
or linker children writing after the gate has advanced.

Every native invocation owns a unique `build/native_tests/run-*` directory and
runs each validated executable immediately after linking it. This closes the
observed race in which a delayed host process or filesystem filter could
truncate one completed executable while all 44 binaries waited for the run
phase. A missing, empty, non-executable or subsequently unlaunchable output
still fails closed; there is no retry that could hide the defect.

A cross-platform advisory lock serializes complete native invocations before
`--clean` can remove `build/native_tests`. This closes the remaining race where
two legitimate gates could delete each other's private directories. Non-clean
focused runs retain a bounded set of completed directories. Temporary-directory
helpers also restore `TMPDIR`, `TEMP`, `TMP` and Python's global temp setting
when their lifetime ends.

### Sanitizer ownership

The native runner adds explicit modes:

```text
none
undefined
address-undefined
leak
```

`tests/native/sanitizer_runtime_options.cpp` disables GCC's implicit leak phase
inside address-sanitized binaries. ASan/UBSan still fail on address and undefined
behavior. The `leak` mode compiles a separate leak-only matrix with exit code 23
for leak findings. No ptrace failure is converted to success and no leak check
is claimed from an ASan/UBSan run.

Dash-prefixed values now work in either form:

```bash
--extra-cxxflag -Werror
--extra-cxxflag=-Werror
```

Predecessor policy subprocesses were removed. Source-level dependency assertions
remain where they describe a real contract; `check_all.py` remains the sole
owner of full ordering and aggregation.

### Host and release modes

`--host-only` requires all host checks and emits only
`host-verified, target build not verified`. Legacy skip options remain available
for focused development and are explicitly labelled partial.

`--release`:

- requires an available, clean Git source identity before doing expensive work;
- forbids all `--skip-*` flags and PlatformIO environment subsets;
- forces a clean normal native matrix, a separate ASan/UBSan matrix and a
  separate LSan matrix;
- runs all policies/validators/replay gates;
- requires the later strict LOGVER3 gate, clean static fixture and independent
  golden JSON;
- requires PlatformIO and all five committed validation environments;
- cleans every target environment and verifies that no recognized firmware
  artifact survived before starting its release build;
- requires `firmware.bin` and `firmware.elf` from every successful target build;
- creates one manifest per environment.

The strict LOGVER3 files are intentionally absent from this patch, so release
preflight remains red instead of accepting the legacy LOGVER2 baseline. The
later capture/gate patch activates the already-defined interface; until then
only host and ordinary target-build results can be produced.

### Manifest contract

`tools/release_manifest.py` writes canonical JSON atomically. Each manifest
contains:

- schema and UTC generation time (`SOURCE_DATE_EPOCH` is honored);
- full 40-character commit, short head, dirty flag, worktree fingerprint and
  firmware feature version;
- PlatformIO environment and version;
- host Python/platform identity;
- repository-relative artifact path, byte size and full SHA-256.

Artifacts outside the repository, missing/empty/changing artifacts,
malformed/duplicate or unknown release toolchain metadata, Git-less source,
failed Git status and dirty release source fail closed.

## Out of scope

This patch does not change firmware C++ under `src/`, PlatformIO profiles,
source filters, partitions, feature flags, default environment, monitor baud,
runtime hot paths, IMU/FIFO/AHRS/calibration behavior, SlimeVR packets, Wi-Fi,
TCP/telnet policy, CLI command semantics, NVS/config/calibration layouts or
machine-log/replay schemas. Factory reset, Debug hotpath parity, telnet capture,
deferred LOGVER3 serialization and low-overhead runtime/static tests remain
later patches.

It also does not remove legacy untracked artifacts from an existing user's
working directory. A clean checkout never contains them, and new gate runs do
not recreate them at the root.

## Compatibility and resource impact

- Firmware wire/CLI/config/NVS/calibration compatibility: unchanged.
- ESP32 CPU/RAM/flash/stack/realtime impact: none; no firmware translation unit
  is changed.
- Host storage: temporary data moves from the root to ignored `build/tmp/`;
  successful PlatformIO builds add small JSON manifests under ignored `build/`.
- Host runtime: removing recursive policy execution reduces repeated work.
  Release mode intentionally adds two full sanitizer matrices and is slower than
  the normal/host-only gate.
- Python requirement remains 3.10+ syntax already used by the project.

## Verification

Focused verification completed while authoring:

```text
python3 tools/test_quality_gate_runtime.py -> PASS, 9 tests
python3 tools/test_release_manifest.py -> PASS, 6 tests
python3 tools/test_run_standalone_tests_policy.py -> PASS, 3 tests
python3 tools/test_check_all_aggregation_policy.py -> PASS, 8 tests
python3 tools/test_build_identity.py -> PASS, 6 tests
python3 tools/validate_documentation.py -> PASS, 13 required docs, 3 root files, 11 links
```

All seven policies that failed in the before-state subsequently passed as
separate commands with their optimized and ASan/UBSan behavioral binaries.
ASan emitted symbolizer warnings in the supervised environment, but no implicit
LeakSanitizer phase occurred.

Complete post-patch host verification:

```text
python3 tools/check_all.py --clean --host-only
  -> PASS (host-verified, target build not verified), 44/44 native executables,
     all validators/policies and both committed replay fixtures

python3 tools/run_standalone_tests.py --clean \
  --extra-cxxflag=-Werror \
  --extra-cxxflag=-fsanitize=undefined \
  --extra-cxxflag=-fno-sanitize-recover=undefined
  -> PASS, 44/44 native executables
```

An additional full `--sanitizer address-undefined --timeout-s 300` matrix was
attempted. Its first two tests and the tests following the timed-out unit passed,
but compilation of `test_calibration_autonomy.cpp` exceeded 300 seconds in this
resource-constrained environment. The bounded runner killed that compiler group;
the optional diagnostic run was then stopped after the already-conclusive
failure. Therefore a complete ASan/UBSan matrix is not claimed as passed.

Patch/artifact verification against the exact base commit:

```text
git diff --check -> PASS
git apply --check 0024_trusted_host_baseline_and_release_identity.patch -> PASS
patch --dry-run -p1 -i 0024_trusted_host_baseline_and_release_identity.patch -> PASS
focused Python tests after real git apply -> PASS (7 + 2 + 6 + 4 + 5 tests)
documentation validation after real git apply -> PASS
```

## Not verified and hardware acceptance

PlatformIO is not installed in the authoring environment. Therefore none of the
five ESP32-C3 profiles, target sizes or generated target manifests are claimed
as passed here. Release mode is intentionally blocked by the absent strict
LOGVER3 gate and would also reject an uncommitted patch-authoring tree before
reaching that check.

The explicit LSan matrix was not run because this environment is
ptrace-supervised; the seven formerly failing policy binaries instead verified
their ASan/UBSan behavior with implicit leak detection disabled. LSan remains a
mandatory separate run on an unsupervised host for release acceptance.

No HIL run is required for a host-tool-only behavioral scope. A maintainer must
still run the five target builds before merging because project policy requires
target compilation for documentation/build-tool changes. This patch is not a
release candidate.

## Rollback

Reverse the patch. Generated files are confined to ignored `build/` and may be
deleted independently. No device flash, NVS migration, configuration change or
external service rollback is involved.

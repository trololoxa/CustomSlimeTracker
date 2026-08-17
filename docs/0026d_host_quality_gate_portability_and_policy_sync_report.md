# 0026d host quality-gate portability and policy sync report

## Scope

This patch changes host test/policy infrastructure only. Firmware production
C++ behavior, packet formats, magnetic algorithms, config/NVS bytes, runtime
cadence and target feature flags are unchanged.

The trigger was a Windows/MSYS2 `check_all` run after 0026c with eleven reported
failures. The supplied failure log exposed three independent gate defects:

- MinGW accepted sanitizer command-line flags but failed at link time with
  `cannot find -lasan` and `cannot find -lubsan`;
- the magnetic reliability source-policy still searched for the pre-0026b text
  `in.heading.yawInnovationRad` after the runtime switched to a lightweight
  input view while retaining the same `heading.yawInnovationRad` decision;
- the two UDP no-heap policies searched raw source for the substring `new ` and
  therefore treated an English comment containing "new default mode" as a heap
  allocation.

Two cross-ABI predecessor-policy inconsistencies were also found during review.
`0023ge` still imposed a 1536-byte mag-fit frame despite successor 0023gk owning
and documenting the final 1792-byte cross-ABI ceiling. 0026b encoded Linux
`-fstack-usage` measurements with only 16-32 bytes of margin, which is not a
portable MSYS2/Windows ABI contract.

## Changes

`quality_gate_runtime.py` now probes whether the selected compiler can compile
and *link* sanitizer runtimes. It prefers ASan+UBSan, falls back to UBSan, and
returns an explicit unsupported result when neither runtime exists. Focused
policies still run their normal `-Werror` behavioral regression; only the
additional sanitizer variant is explicitly skipped when the host toolchain
cannot link sanitizer libraries. This follows the project rule that an
incompatible sanitizer environment is recorded separately rather than reported
as a firmware defect or a sanitizer PASS.

Seven predecessor/focused policies use that shared probe. Unit tests cover both
ASan-missing/UBSan-available fallback and the no-runtime case.

The magnetic source-policy now checks `wrapPi(heading.yawInnovationRad)`, the
actual 0026b implementation of the same AHRS-yaw-invariant field signal. UDP
no-heap policies strip C/C++ comments before checking for a `new ` expression.

The 0023ge mag-calibration stack ceiling is synchronized with the final 0023gk
1792-byte cross-ABI contract. 0026b stack gates retain absolute ceilings but add
bounded ABI spill/shadow-space margin; they remain well below the project's
1 KiB runtime-path preference except for the pre-existing calibration fit frame.
No production frame was enlarged by this patch.

## Verification in the authoring environment

Focused policies after the fix:

- magnetic heading reliability policy: PASS;
- 0023gl UDP TX recovery policy: PASS;
- pre-0024ad network-pressure policy: PASS;
- 0026b hot-path optimization policy: PASS;
- 0023ge magnetometer audit hardening policy: PASS;
- 0023gk robust-fit acceptance policy: PASS;
- pre-0024 hotpath headroom policy: PASS;
- pre-0024a tracking deadline policy: PASS;
- pre-0024ab transform-cache policy: PASS;
- pre-0024ac IMU hotpath/slack policy: PASS;
- quality-gate runtime unit tests: PASS.

On Linux/GCC the sanitizer probe selects `address-undefined`, so sanitizer
coverage remains active rather than being silently removed.

A parallel reproduction of the standalone native matrix with the same project,
compile-only and Arduino aggregate sources compiled 117/117 translation units
with `-Werror`; all 48 native executables linked and 48/48 ran successfully.
The user's separate Windows standalone-suite failure was not accompanied by its
`# FAIL [stage] ...` details, so this patch does not claim a diagnosis of that
specific runner failure. It removes the independently proven policy/toolchain
failures from the same `check_all` output.

PlatformIO and Windows/MSYS2 execution are NOT RUN in the authoring environment.
The required acceptance is to rerun the same `check_all` command on the user's
MSYS2 UCRT64 toolchain and confirm sanitizer variants are reported as explicit
SKIP (or UBSan fallback) rather than linker failures.

## Compatibility and rollback

No firmware compatibility impact. Rollback is code-free: reverse this host-tool
patch. No config, calibration, packet, NVS or wire migration is involved.

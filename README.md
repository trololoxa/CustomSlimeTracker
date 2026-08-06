# CustomSlimeTracker

ESP32-C3 firmware for a custom SlimeVR tracker using an LSM6DSV IMU and
QMC6309 magnetometer. The repository contains firmware sources, five committed
PlatformIO validation environments, host-native logic tests, policy checks and
machine-log replay tooling.

Canonical project documentation starts at:

- [current implementation](docs/current_implementation.md);
- [build profiles](docs/build_profiles.md);
- [testing strategy](docs/testing.md);
- [tracking pipeline](docs/tracking_pipeline.md);
- [CLI reference](docs/cli_reference.md).

## Trusted gates

Run the complete host gate on a machine without PlatformIO:

```bash
python3 tools/check_all.py --clean --host-only
```

This result is reported only as `host-verified, target build not verified`.
The release gate requires a clean Git source identity, all host checks, the
complete five-environment clean PlatformIO matrix and SHA-256 manifests:

```bash
python3 tools/check_all.py --release
```

`--release` rejects dirty or Git-less source trees and every `--skip-*` option.
Focused skip modes remain available for development, but are labelled partial
and are not release evidence. The strict LOGVER3 parser and capture-integrity
gate are installed, but release remains intentionally blocked until a real
cable-free static fixture and its independently reviewed golden JSON are
committed. Current LOGVER2 replay cannot make a release pass.

Sanitizers have separate ownership:

```bash
python3 tools/run_standalone_tests.py --clean --sanitizer address-undefined
python3 tools/run_standalone_tests.py --clean --sanitizer leak
```

The first command runs ASan/UBSan without implicit leak checking. The second is
the explicit LSan gate and must run on a host where LeakSanitizer is supported
(not under ptrace/debugger attachment).

Successful target builds write per-environment manifests below
`build/check_all/platformio/manifests/`. A manifest records the full commit,
dirty state, build environment, UTC timestamp, tool version and SHA-256/size of
the firmware artifacts. Release mode rejects an unavailable tool version,
empty artifact, failed target clean or artifact left behind by that clean.

## Configuration and secrets

Copy local credentials into `secrets.ini` or `platformio.local.ini`; both are
ignored. Do not commit Wi-Fi credentials, generated device identity, captured
private traffic, `.pio/`, `build/` or host-test artifacts.

Native tests do not replace ESP32-C3 builds or hardware acceptance. Sensor,
FIFO, Wi-Fi, SlimeVR and long-run claims require the hardware procedures in
[testing.md](docs/testing.md).

## Cable-free LOGVER3 capture

The committed default is the locked-down `BOARD_LOLIN_C3_MINI_PRODUCTION`
profile. Flash the explicit `BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG` environment
only for diagnostics. Its TCP console exposes a fail-closed, non-persistent
allowlist and binds log/test output to the initiating session.

After a clean target build and flash, power the tracker from its battery and
disconnect USB before capturing:

```bash
python3 tools/capture_telnet_log.py \
  --host <tracker-ip> \
  --capture static \
  --seconds 600 \
  --rate 20 \
  --mode full \
  --output logver3_static_clean_001.log
```

The tool verifies full clean-build identity and magnetometer readiness, runs a
session-bound static test, drains the deferred logger, applies the strict
LOGVER3 E1 integrity gate and promotes a hash-bound log/manifest pair only after
both candidates are ready. A failed capture is retained under a unique
`.partial-*` name for diagnosis, never promoted to a golden fixture.

Use `--capture runtime` while reproducing an in-game UDP problem, with SlimeVR
and UDP left active. A complete diagnostic may intentionally report
`health_passed=false`; preserve that log and manifest. TCP shares the tracker's
Wi-Fi/lwIP resources with UDP, so a general radio/buffer failure can still
produce only a partial capture. The tool never changes persistent MAG/bias/yaw
settings and a 30-second firmware lease releases a vanished host's log/test and
sleep blocker.

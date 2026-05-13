# Project status

This document replaces the completed code-quality roadmap notes. It records the current structural state of the firmware after the architecture cleanup patches.

## Current structural baseline

- `main.cpp` is an Arduino entrypoint only.
- `app/` owns setup/loop orchestration and app-level singleton wiring.
- `app/hooks/` contains include-only glue sections; hooks must stay wiring-only.
- `runtime/` owns FIFO processing, sample pipeline, static tests, machine logs, output snapshots, runtime bias, mag runtime, and tracking state.
- `sensor/` owns math/model code for AHRS, calibration, quality, magnetometer heading, and yaw correction.
- `serial/` command domains are split into `.hpp/.cpp` pairs; parser glue remains fixed-buffer/no-heap.
- `config/` owns persisted schema, runtime apply/capture/sanitize, NVS store, and printing.
- `connection/` owns low-level hardware/protocol drivers.
- `src/network/` is reserved for future real Wi-Fi/UDP transport only.

## Deliberate developer conveniences

- Boot-time `sleep(2)` remains intentionally. It gives time to open Serial Monitor during development and can be removed in final production cleanup.
- `defines.h` is the canonical compile-time defaults header. No `defines.hpp` shim is used.
- `BOARD_LOLIN_C3_MINI_DIAG` is the normal local build/upload environment while warnings are being kept clean.

## Current quality gate

Run host tests and PlatformIO builds:

```bash
python tools/check_all.py --clean --require-pio
```

If PlatformIO is not on `PATH`, pass `--pio-bin` or set `PIO`.

Host-only check:

```bash
python tools/check_all.py --clean --skip-pio
```

## Remaining structural work

1. Keep `serial/tracker_serial_context.hpp` lightweight by moving reusable parse/print helpers to focused headers.
2. Maintain `docs/cli_reference.md` when commands or side effects change.
3. Maintain `docs/config_schema.md` when persisted schema changes.
4. Build replay/log tooling before major tracking-filter changes.
5. Do not add SlimeVR UDP output until local quaternion, diagnostics, and replay metrics are stable.

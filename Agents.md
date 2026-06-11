# AGENTS.md

## Project goal

This is a custom SlimeVR-compatible IMU tracker firmware for ESP32-C3 / nRF52840-class boards using LSM6DSV-class IMUs. The firmware must prioritize stable real-time tracking, correct calibration, low latency, and reliable Wi-Fi/UDP output.

## Core architecture

- `src/app/`: app orchestration and runtime wiring.
- `src/sensor/`: calibration, AHRS, math, quality monitor.
- `src/connection/`: IMU drivers, FIFO, timestamps, bus logic.
- `src/network/`: Wi-Fi, UDP, SlimeVR protocol/runtime.
- `src/serial/`: CLI commands.
- `src/runtime/`: tests, telemetry, LED, tap, battery, logging.
- `src/config/`: persistent config and NVS calibration storage.
- `docs/`: design notes and status docs.

## Non-negotiable tracking rules

- Do not reduce IMU ODR or sensor performance modes as an optimization unless explicitly asked.
- Do not remove gyro bias calibration, accel calibration, mag calibration placeholders, axis alignment, or temperature compensation.
- Do not replace AHRS with vendor SFLP/game rotation vector unless explicitly asked for an R&D comparison.
- Preserve FIFO timestamp correctness.
- Preserve quaternion output correctness and documented coordinate conventions.
- SlimeVR server expects firmware to send ready orientation/quaternion in normal operation.

## Embedded constraints

- Avoid heap allocation and `String` in hot paths.
- Avoid blocking calls in `loop()`.
- Avoid Serial formatting in production hot paths.
- Do not add repeated expensive config recomputation inside runtime update loops.
- Keep Wi-Fi/network work from starving FIFO/AHRS processing.
- Any long operation must be command-triggered, debug-only, or explicitly bounded.

## Build and test expectations

Before claiming success, run the most relevant available checks:

```bash
C:\Users\nikol\.platformio\penv\Scripts\platformio.exe run
python tools/check_all.py
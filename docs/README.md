# Project documentation

Read the page owning the task; historical patch reports are not current specifications.

## Readiness and plans

- [Roadmap](roadmap.md)
- [Current status and limitations](status.md)

## Architecture

- [Battery ADC filtering](architecture/battery.md)
- [Safe background calibration autonomy](architecture/calibration_autonomy.md)
- [Calibration storage and candidate framework](architecture/calibration_storage.md)
- [Coordinate frames and quaternion conventions](architecture/coordinate_frames.md)
- [Runtime hot-path optimization notes](architecture/hotpath.md)
- [Current implementation baseline](architecture/implementation.md)
- [Magnetic heading reliability and continuous axis candidates](architecture/magnetic_heading.md)
- [Module inventory](architecture/module_inventory.md)
- [Tracker firmware architecture](architecture/ownership.md)
- [Runtime boundaries](architecture/runtime_boundaries.md)
- [Motion-triggered light sleep](architecture/sleep.md)
- [Tracking pipeline](architecture/tracking_pipeline.md)
- [Wi-Fi power and thermal optimization status](architecture/wifi_power.md)

## Reference

- [Build profiles](reference/build_profiles.md)
- [Serial CLI reference](reference/cli.md)
- [Config schema policy](reference/configuration.md)
- [Firmware profile contract](reference/profile_contract.md)
- [Wi-Fi remote CLI console](reference/remote_console.md)
- [SlimeVR Wi-Fi / UDP runtime](reference/slimevr.md)
- [Source filter matrix](reference/source_filters.md)

## Development

- [инструкции и дополнения для GPT-6 Astra](development/agent_tools.md)
- [Calibration Validation](development/calibration_validation.md)
- [Capture Validation](development/capture_validation.md)
- [Changing firmware code](development/code_changes.md)
- [один запуск Windows + WSL](development/cross_platform_verification.md)
- [Device Smoke](development/device_smoke.md)
- [окружение разработки Tracker](development/environment.md)
- [independent math and algorithm scenarios](development/fusion_validation.md)
- [Documentation and test maintenance](development/maintenance.md)
- [Tracker: спецификация математических проверок](development/math_coverage.md)
- [Replay and metrics](development/replay.md)
- [выборочные прогоны и логи](development/runners.md)
- [ESP32-C3 target smoke](development/sensor_smoke.md)
- [возобновление среды и одинаковый исходный код](development/session.md)
- [owners and test selection](development/test_map.md)
- [Testing](development/testing.md)

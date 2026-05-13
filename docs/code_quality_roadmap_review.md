# Code quality roadmap review and execution plan

Дата ревью: 2026-05-09
Область ревью: `src/Code_Roadmap.md`, `src/main.cpp`, `src/serial/tracker_serial_commands.hpp`, `src/config/tracker_config.hpp`, `src/defines.h`, `platformio.ini`.

## Executive summary

`src/Code_Roadmap.md` правильно описывает целевую архитектуру: маленький `main.cpp`, разделение runtime/FIFO/static-test/mag/output/CLI/config, строгие warning-и, отсутствие placeholder-функций и секретов в репозитории. Документ годится как стратегическая карта, но в текущем виде он слишком широкий для прямой реализации: несколько фаз смешивают P0-блокеры, архитектурный рефакторинг и будущие research-задачи. Для безопасного улучшения tracking quality нужен staged-подход: сначала стабилизировать сборку/конфиг/placeholder-поведение, затем выделять runtime-модули без изменения алгоритмов, и только после этого менять AHRS/mag/bias логику.

## Что в roadmap уже хорошо

1. Верно зафиксировано главное архитектурное правило: `main.cpp` должен быть orchestration layer, а не местом жизни FIFO, static test, mag runtime, CLI hooks и tracking state.
2. Верно выделены источники истины: compile-time defaults в `defines`, persisted settings/calibration в `TrackerConfig/NVS`, временные счетчики и состояние в runtime state.
3. Правильно отмечены риски placeholder-команд: пользователь не должен включать `slimevr`/`binary` output, если backend отсутствует.
4. Правильно обозначен embedded memory policy: fixed buffers, без `String`, без heap в realtime path.
5. Документ покрывает не только “красоту кода”, но и tracking risks: FIFO recovery, timestamp health, gyro/temp calibration, mag trust, replay tests.

## Что нужно поправить в roadmap

1. Добавить отдельный P0 gate перед Phase A: sanitize repository, disable fake output modes, verify clean build baseline. Без этого большой рефакторинг будет смешиваться с исправлением очевидных дефектов.
2. Разделить “механический рефакторинг” и “изменение tracking behavior”. Перенос кода в модули должен быть поведенчески нейтральным, иначе невозможно понять, что ухудшило/улучшило tracking.
3. Для каждой фазы добавить test/log artifacts: какие CLI-команды запускать, какие counters сравнивать, какие значения считаются pass/fail.
4. Явно запретить “refactor by rewrite”: сначала extract existing code, затем tighten interfaces, потом optimize.
5. Добавить migration policy для `TrackerConfigBlob`: текущий CRC по raw struct зависит от layout/padding и требует аккуратного bump версии при любом изменении структуры.
6. Перенести “SlimeVR UDP” в отдельный milestone после стабильного quaternion/quality pipeline. Сначала firmware должна уверенно считать ориентацию локально.

## Приоритеты выполнения

### P0 — остановить рост долга

Цель: сделать поведение честным и безопасным без изменения tracking math.

- Убрать реальные Wi-Fi credentials и локальный IP из `platformio.ini`.
- Сделать один canonical defaults header и начать перевод `main.cpp`/`TrackerConfig` на него.
- Запретить выбор `output mode binary/slimevr`, пока нет backend implementation.
- Boot-time `sleep(2)` оставлен намеренно как developer convenience: он дает время открыть Serial Monitor. Удалить его можно в финальной production-cleanup фазе.
- `defines.h` остается canonical defaults header. `defines.hpp` не нужен, пока проект не принимает решение переименовать header целиком.

Acceptance:

- В репозитории нет реальных секретов.
- `output mode slimevr` возвращает `NOT_IMPLEMENTED`, а не сохраняет fake packet format.
- Пины, baud, FIFO limits и SPI defaults имеют один явный compile-time source.
- Runtime tracking behavior не меняется.

### P1 — baseline build hygiene

Цель: получить воспроизводимую сборку и предупреждения до архитектурного split.

- Добавить warning flags: `-Wall -Wextra -Wshadow -Wdouble-promotion`.
- Исправить предупреждения без изменения алгоритмов.
- Добавить `.gitignore`, `platformio.example.ini`/`secrets.example.ini` или документированный `secrets.ini` workflow.
- Зафиксировать минимальный smoke test: `status`, `fifo stats`, `quality stats`, `stream quat`, `test static 120`.

Acceptance:

- Сборка проходит с baseline warnings policy.
- Все warning suppressions локальны и объяснены.
- Smoke test output можно сравнить до/после.

### P2 — extract static test

Цель: убрать самый большой non-core блок из `main.cpp`, сохранив CLI behavior.

- Создать `runtime/static_test.hpp` и позже `.cpp`.
- Вынести active state, completed result, temp bins, progress/report functions.
- CLI должен работать через hooks/API, а не через globals.
- `cal temp fit_static` должен потреблять last completed result через API.

Acceptance:

- `main.cpp` уменьшается, но команды `test static/status/stop` и `cal temp fit_static` дают прежний output format.
- Static test не блокирует FIFO/AHRS loop.

### P3 — extract FIFO runtime

Цель: сделать timing/recovery понятными и тестируемыми.

- Создать `runtime/fifo_runtime.hpp`.
- Вынести IRQ event consumption, fallback FIFO_STATUS polling, drain rounds, perf counters.
- Явно передавать callbacks: `onRawSample`, `onMagSample`, `onRecovery`.
- Отдельно считать IRQ events, fallback events, empty polls, drain errors.

Acceptance:

- AHRS получает samples тем же порядком и с теми же timestamps.
- Recovery storm prevention остается.
- `fifo stats` и `quality stats` показывают те же или более подробные counters.

### P4 — extract tracking pipeline state

Цель: убрать scattered flags вокруг recovery/confidence/output snapshot.

- Создать `runtime/tracking_state.hpp`.
- Ввести explicit states: `STARTUP_CONVERGENCE`, `TRACKING_6DOF`, `TRACKING_6DOF_MAG_YAW`, `DEGRADED_TIMING`, `DEGRADED_ACCEL`, `DEGRADED_MAG`, `RECOVERING`, `SENSOR_FAULT`.
- Все status/log/output должны читать один state source.

Acceptance:

- State transitions логируются как `STATE` frames.
- CLI `status` и machine log не расходятся.

### P5 — extract mag runtime

Цель: защитить yaw correction от stale reference и сделать mag pipeline управляемым.

- Создать `runtime/mag_runtime.hpp`.
- Вынести QMC sample processing, calibration collector, heading estimator, auto-reference, yaw correction controller.
- AHRS reset/recovery должен инвалидировать mag reference через один API.

Acceptance:

- Mag yaw correction не применяется без valid accel + mag calibration.
- `mag status`, `mag heading`, `mag yaw` не зависят от main globals.

### P6 — split serial CLI

Цель: избавиться от 3000+ line header-only dispatcher.

Статус: в текущем коде доменные CLI-модули разделены на `.hpp/.cpp` пары. `tracker_serial_commands.hpp` оставляет fixed-buffer parser/template glue и declaration router-а, а routing implementation живёт в `tracker_serial_commands.cpp`. Domain behavior живёт в соответствующих `tracker_*_commands.cpp`.

Правила после split:

- `serial/tracker_serial_context.hpp` остаётся lightweight context/types file.
- Parser остаётся fixed-buffer/no-heap.
- Команды с side effects должны печатать explicit result и persist behavior.
- Новую команду добавлять в `.cpp`; в `.hpp` выносить только API, который нужен другим translation units.
- Command `.cpp` должен явно include-ить полный тип, если обращается к полям/методам объекта из `TrackerSerialCommandContext`.
- Shared helpers между command domains объявляются только осознанно, в header домена-владельца.

Acceptance:

- Parser остается fixed-buffer/no-heap.
- Placeholder commands отсутствуют или возвращают `NOT_IMPLEMENTED`.
- CLI reference поддерживается в `tracker_system_commands.cpp`.

### P7 — config schema hardening

Цель: сохранить калибровки надежно при изменениях структуры.

- Уточнить persisted vs runtime fields.
- Добавить schema version bump policy.
- Убрать raw-struct CRC dependency или явно zero-initialize/pad reserved bytes.
- Добавить migration v1 -> v2, где возможно.

Acceptance:

- Corrupted/mismatched config не дает silent bad runtime.
- Calibration validity независима по gyro/accel/mag/temp.

### P8 — replay/tests/docs

Цель: улучшать tracking quality измеримо.

- Добавить host-side replay для CSV/machine logs.
- Добавить tests для math/quaternion/config CRC/parsing.
- Документировать coordinate frames, tracking pipeline, calibration flow, CLI side effects.

Acceptance:

- Любое изменение AHRS/mag/bias можно проверить на одном и том же log наборе.
- Появляется regression baseline по drift, recovery count, accel trust, mag reject flags.

## First implementation batch

Этот patch set покрывает только P0:

1. `defines.h` остается canonical defaults header; `defines.hpp` не добавляется.
2. `main.cpp` остается минимальным entrypoint.
3. `platformio.ini` больше не содержит реальных credentials.
4. `output mode binary/slimevr` возвращает `NOT_IMPLEMENTED` до появления backend.
5. Boot `sleep(2)` оставлен как временный developer convenience до финальной cleanup-фазы.

Текущий безопасный batch после крупного refactor: split `app/tracker_app_hooks.hpp` на маленькие include-only hook sections, добавить host/PlatformIO quality gate, и добавить native config layout guards без runtime-cost.

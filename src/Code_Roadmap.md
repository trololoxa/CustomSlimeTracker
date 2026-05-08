# Roadmap: Code Quality / Firmware Architecture для SlimeVR IMU-трекера

Примечание - добавить compile-time флаги на трудоёмкие логи/нагрузки, которые не нужны в финале

## Главная цель

Привести проект к состоянию финального поддерживаемого firmware-продукта:

- код собирается чисто, без критичных warning-ов;
- tracking, hardware, calibration, config, CLI и output разделены по модулям;
- `main.cpp` не содержит всю прошивку целиком;
- CLI не является огромным header-only приложением;
- compile-time дефолты лежат в `defines.h`;
- runtime/persisted значения лежат в config/NVS;
- нет мёртвых файлов “на всякий случай”;
- нет placeholder-команд, которые выглядят рабочими;
- нет скрытых side effects;
- production core отделён от experimental/research кода;
- tracking math можно тестировать отдельно от железа;
- проект можно развивать без постоянного накопления технического долга.

Главный принцип:

    Если значение хардкодится — оно должно быть в defines.h.
    Если значение меняется пользователем или сохраняется — оно должно быть в TrackerConfig/NVS.
    Если значение является временным состоянием — оно должно жить в Runtime state.
    Если feature не готова — она не должна выглядеть как готовая команда/config option.

---

# Текущее состояние проекта, которое нужно исправить

По текущему проекту уже видны основные проблемы:

    src/main.cpp
      слишком большой;
      содержит setup, loop, FIFO runtime, static test, mag runtime,
      AHRS hooks, recovery, stream output, global objects и много constants.

    src/serial/tracker_serial_commands.hpp
      слишком большой;
      parser, dispatcher, stream helpers, config commands,
      calibration commands и output commands живут в одном header.

    src/defines.hpp
      выглядит как dead/legacy/double-source file;
      часть значений дублируется в main.cpp и config.

    src/config/tracker_config.hpp
      хорошая основа, но config смешивает hardware defaults,
      runtime settings, calibration placeholders и future fields.

    src/network/
      пустая папка;
      при этом output mode slimevr/config packetFormat создают ощущение,
      что SlimeVR backend уже есть.

    Некоторые поля config/CLI выглядят реализованными,
    но фактически являются placeholder.

    Есть риск compile issue:
      fitGyroTempFromLastStaticHook используется до объявления.

    Есть warning zones:
      uint64_t -> double conversion в timestamp math;
      volatile increment warning;
      unused parameters/constants/fields.

    В platformio.ini могут лежать локальные Wi-Fi credentials/server IP.
    Это не должно быть частью основного project config.

---

# Финальная целевая структура проекта

Рекомендуемая структура src:

    src/
      main.cpp

      defines.h

      core/
        math.hpp
        fixed_types.hpp            optional
        units.hpp                  optional

      platform/
        board_pins.hpp             optional if board-specific
        time_utils.hpp
        serial_io.hpp              optional

      connection/
        lsm6dsv_driver.hpp
        lsm6dsv_driver.cpp
        lsm6dsv_fifo.hpp
        lsm6dsv_fifo.cpp
        lsm6dsv_sensorhub.hpp
        lsm6dsv_sensorhub.cpp
        qmc6309_transport.hpp      optional

      sensor/
        calibration.hpp
        accel_calibration.hpp
        gyro_calibration.hpp
        gyro_temperature_compensation.hpp
        mag_calibration.hpp
        mag_heading.hpp
        mag_yaw_correction.hpp
        imu_quality.hpp
        ahrs_6dof.hpp
        tracking_filter.hpp        optional interface

      runtime/
        tracker_runtime.hpp
        tracker_runtime.cpp
        fifo_runtime.hpp
        fifo_runtime.cpp
        static_test.hpp
        static_test.cpp
        tracking_state.hpp
        tracking_state.cpp
        mag_runtime.hpp
        mag_runtime.cpp
        output_stream.hpp
        output_stream.cpp

      config/
        tracker_config.hpp
        tracker_config.cpp
        config_store.hpp
        config_schema.hpp
        config_defaults.hpp

      serial/
        command_context.hpp
        command_parser.hpp
        command_router.hpp
        commands_config.hpp
        commands_calibration.hpp
        commands_imu.hpp
        commands_fifo.hpp
        commands_ahrs.hpp
        commands_mag.hpp
        commands_static_test.hpp
        commands_output.hpp
        stream_text.hpp

      output/
        output_types.hpp
        serial_text_output.hpp
        slimevr_udp_output.hpp      only when real implementation exists
        packet_writer.hpp

      network/
        wifi_manager.hpp            only when network output is implemented
        udp_transport.hpp           only when network output is implemented

      debug/
        debug_counters.hpp
        debug_print.hpp

      experimental/
        ahrs_eskf/
        earth_rotation/
        coning/
        online_mag_calibration/

    tools/
      replay/
      calibration/
      packet_decode/
      log_analyze/

    docs/
      architecture.md
      tracking_pipeline.md
      coordinate_frames.md
      config_schema.md
      cli_reference.md
      calibration.md
      test_plan.md

---

# Architecture rule: main.cpp must be small

## Goal

`main.cpp` should only wire modules together.

Allowed in main.cpp:

    global top-level object construction;
    setup();
    loop();
    board boot sequence;
    calling runtime.begin();
    calling runtime.poll();

Not allowed in final main.cpp:

    AHRS implementation details;
    FIFO drain internals;
    static test logic;
    calibration algorithms;
    command parsing;
    mag yaw correction internals;
    config serialization details;
    long helper functions;
    hardcoded tracking constants.

Target size:

    main.cpp <= 300-500 lines

Acceptance criteria:

    1. main.cpp can be read as firmware orchestration.
    2. Tracking runtime can be understood without scrolling through CLI/config/debug code.
    3. Static test can be changed without editing main.cpp.
    4. Mag runtime can be changed without editing main.cpp.
    5. FIFO recovery behavior can be tested as a runtime module.

---

# Phase A — constants, defines.h, and source of truth

## A1. Create/normalize src/defines.h

All compile-time defaults should be moved into `src/defines.h`.

defines.h should contain:

    board identity defaults;
    pin defaults;
    SPI defaults;
    serial baud default;
    IMU ODR/FS defaults;
    FIFO buffer sizes;
    FIFO watermark defaults;
    FIFO drain limits;
    AHRS default gains;
    AHRS default gates;
    accel correction default thresholds;
    mag correction default thresholds;
    static test default durations/sample limits;
    stream default rate;
    feature flags;
    build-time safety limits.

Example categories:

    TRACKER_DEFAULT_BAUD
    TRACKER_DEFAULT_SPI_HZ

    TRACKER_PIN_IMU_CS
    TRACKER_PIN_IMU_INT1
    TRACKER_PIN_SPI_SCK
    TRACKER_PIN_SPI_MISO
    TRACKER_PIN_SPI_MOSI

    TRACKER_FIFO_RAW_BUFFER_CAPACITY
    TRACKER_FIFO_MAX_WORDS_PER_DRAIN
    TRACKER_FIFO_MAX_DRAIN_ROUNDS
    TRACKER_FIFO_DEFAULT_WATERMARK

    TRACKER_AHRS_DEFAULT_ACCEL_GAIN
    TRACKER_AHRS_DEFAULT_MAX_DT_US
    TRACKER_AHRS_DEFAULT_ACCEL_NORM_GOOD
    TRACKER_AHRS_DEFAULT_ACCEL_NORM_BAD

    TRACKER_MAG_DEFAULT_MAX_AGE_US
    TRACKER_MAG_DEFAULT_MAX_YAW_RATE_DPS

    TRACKER_STATIC_TEST_DEFAULT_SECONDS
    TRACKER_STATIC_TEST_MAX_SECONDS

Rules:

    1. No magic tracking constants directly in main.cpp.
    2. No duplicate constants between main.cpp and config defaults.
    3. Config defaults may refer to defines.h.
    4. Runtime state must not live in defines.h.
    5. Device-specific calibration values must not live in defines.h unless they are explicit factory data for one known unit.

Acceptance criteria:

    1. grep for old pin/FIFO/AHRS constants in main.cpp finds no duplicated source-of-truth values.
    2. TrackerConfig defaults are initialized from defines.h where appropriate.
    3. Changing a compile-time default requires editing one file.
    4. Config print shows effective runtime values.

---

## A2. Resolve defines.hpp vs defines.h

Current `defines.hpp` should not remain as an unused duplicate.

Options:

    Preferred:
      replace/rename it with src/defines.h and make it the actual source of compile-time defaults.

    Alternative:
      delete it if all constants are moved elsewhere.

Not acceptable:

    unused defines.hpp stays in project;
    two define files with overlapping meanings exist;
    main.cpp still owns the same constants.

Acceptance criteria:

    1. Only one compile-time defaults header exists.
    2. It is included by modules that need defaults.
    3. There are no stale hardcoded calibration seeds in it.

---

## A3. Separate compile-time defaults, persisted config, runtime state

Definitions:

    defines.h:
      factory defaults, feature flags, buffer sizes, safety limits.

    TrackerConfig/NVS:
      user/runtime settings that persist;
      calibration data;
      output settings;
      AHRS tuning values if user-configurable.

    Runtime state:
      current AHRS quaternion;
      counters;
      quality status;
      active test;
      last completed test;
      temporary calibration capture;
      current tracking state.

Acceptance criteria:

    1. No persisted state in defines.h.
    2. No compile-time hardware constants duplicated in runtime modules.
    3. No temporary runtime counters stored in config.
    4. Status/config print makes source of values clear.

---

# Phase B — clean build and warnings

## B1. Fix known compile issue: use-before-declaration

Problem:

    A function such as fitGyroTempFromLastStaticHook is used before declaration.

Fix options:

    1. Add forward declaration.
    2. Move function above first use.
    3. Preferably move static-test/temp-fit code into runtime/static_test.hpp/.cpp.

Acceptance criteria:

    1. Project builds as normal C++ without Arduino .ino auto-prototype behavior.
    2. No hidden dependency on function declaration order inside huge main.cpp.
    3. Static test code is modular after refactor.

---

## B2. Enable strict warnings

PlatformIO build should enable at least:

    -Wall
    -Wextra
    -Wshadow
    -Wdouble-promotion

Carefully evaluate:

    -Wconversion
    -Wfloat-conversion

These may be noisy in embedded math, but warnings from timestamp/math code should not be ignored.

Acceptance criteria:

    1. Normal production build has zero warnings or only explicitly documented unavoidable warnings.
    2. Any warning suppression is local and justified.
    3. No global blanket suppression hiding real issues.

---

## B3. Fix known warning categories

Known warning zones to clean:

    uint64_t -> double precision conversion in timestamp math;
    deprecated volatile increment pattern in ISR counter;
    unused parameters;
    unused constants;
    unused private fields;
    signed/unsigned mismatches;
    float/double accidental promotions;
    narrowing conversions in sensor scaling.

Guidelines:

    1. Timestamp math should prefer integer/fixed-point where possible.
    2. If double is used, precision and uptime limits must be documented.
    3. ISR counters should use safe volatile or atomic-compatible patterns for target platform.
    4. Unused parameters should be removed or marked explicitly.
    5. Unused fields should be removed unless reserved with documentation.

Acceptance criteria:

    1. Warning list is empty.
    2. Timestamp conversion behavior is documented/tested.
    3. No unused constants remain in production files.

---

# Phase C — split main.cpp

## C1. Create runtime/tracker_runtime

`TrackerRuntime` should own high-level runtime flow.

Responsibilities:

    begin();
    poll();
    processSamples();
    updateTrackingState();
    handleRecovery();
    emitOutputIfDue();

It should coordinate:

    FIFO runtime;
    AHRS;
    quality monitor;
    mag runtime;
    static test;
    output;
    command hooks.

It should not implement low-level driver logic.

Acceptance criteria:

    1. loop() becomes small:
         g_runtime.poll();
    2. setup() mostly configures objects and calls begin().
    3. Runtime state is grouped in one structure/class instead of scattered globals.

---

## C2. Create runtime/fifo_runtime

Move FIFO processing from main.cpp into a module.

Responsibilities:

    waiting for FIFO event;
    draining FIFO;
    handling FIFO status;
    detecting timing/fallback issues;
    passing raw samples to tracking pipeline;
    raising recovery events.

Interfaces:

    input:
      LSM driver/FIFO reader;
      interrupt event source;
      buffer;

    output:
      raw samples;
      FIFO health events;
      timestamp health;
      counters.

Acceptance criteria:

    1. FIFO drain logic is not in main.cpp.
    2. FIFO recovery events are explicit.
    3. FIFO runtime can be tested with fake samples/status.
    4. AHRS receives explicit timing health info.

---

## C3. Create runtime/static_test

Move static test code from main.cpp.

Responsibilities:

    start static test;
    update with samples;
    finish static test;
    store last completed result;
    report metrics;
    provide data for gyro temp fit.

Must contain:

    StaticTestActiveState;
    StaticTestResult;
    StaticTestConfig;
    StaticTestRunner;

Acceptance criteria:

    1. Static test state is not scattered in main.cpp.
    2. lastCompletedStaticTest survives after finish.
    3. temp fit can consume last completed result.
    4. CLI only calls hooks on this module.

---

## C4. Create runtime/tracking_state

Move state/confidence logic to its own module.

Responsibilities:

    current tracking state;
    confidence;
    reason flags;
    transitions;
    counters;
    state reporting.

States:

    CALIBRATION_REQUIRED
    STARTUP_CONVERGENCE
    TRACKING_6DOF
    TRACKING_6DOF_MAG_YAW
    DEGRADED_TIMING
    DEGRADED_ACCEL
    DEGRADED_MAG
    RECOVERING
    SENSOR_FAULT

Acceptance criteria:

    1. Tracking state is not inferred from scattered flags.
    2. CLI/status/output all use the same state source.
    3. Recovery transitions are explicit.
    4. Logs include state changes.

---

## C5. Create runtime/mag_runtime

Move mag sample processing, heading state, auto-reference and correction coordination.

Responsibilities:

    receive mag samples;
    apply mag calibration;
    compute heading;
    maintain reference;
    evaluate mag trust;
    call yaw correction;
    expose status.

Acceptance criteria:

    1. Mag state is reset on AHRS reset/recovery through one API.
    2. Mag correction cannot accidentally use stale reference.
    3. Mag runtime can be tested without FIFO hardware.
    4. Mag rejection reasons are available to logs/status.

---

## C6. Create runtime/output_stream

Move current serial stream helpers and output scheduling.

Responsibilities:

    stream raw;
    stream calibrated;
    stream quaternion;
    stream debug;
    rate limiting;
    formatting machine-readable output.

Acceptance criteria:

    1. Streaming code is not mixed with AHRS update code.
    2. Stream output can be disabled without changing tracking runtime.
    3. Stream rate limiting cannot block FIFO processing.
    4. Human-readable CLI and machine-readable stream are separate.

---

# Phase D — split serial CLI

## D1. Split tracker_serial_commands.hpp

Current single huge header should be divided.

Target files:

    serial/command_context.hpp
    serial/command_parser.hpp
    serial/command_router.hpp

    serial/commands_config.hpp
    serial/commands_calibration.hpp
    serial/commands_imu.hpp
    serial/commands_fifo.hpp
    serial/commands_ahrs.hpp
    serial/commands_mag.hpp
    serial/commands_static_test.hpp
    serial/commands_output.hpp

    serial/stream_text.hpp

Responsibilities:

    command_parser:
      tokenization;
      input buffering;
      no knowledge of AHRS/FIFO internals.

    command_router:
      maps command names to handlers.

    command_context:
      narrow pointers/hooks to runtime/config/calibration.

    commands_*:
      implement domain-specific CLI commands.

Acceptance criteria:

    1. No single serial header above ~500-800 lines unless justified.
    2. Parser can be tested independently.
    3. Adding a mag command does not touch config command implementation.
    4. CLI dependencies flow through command_context only.

---

## D2. Define command side effects

Every command must document:

    what it reads;
    what it changes in runtime;
    what it saves to NVS;
    whether it blocks;
    whether it affects AHRS/mag state;
    whether it is safe during tracking.

Examples:

    cal gyro
      measures and applies runtime gyro bias;
      does not save unless explicit save or config save.

    cal gyro save
      saves current gyro calibration.

    config save
      saves current config/calibration snapshot.

    ahrs reset
      resets AHRS and mag-dependent state.

    fifo reset
      resets FIFO/timestamp reconstruction and enters tracking recovery.

Acceptance criteria:

    1. No command silently saves bad calibration.
    2. No command resets AHRS without resetting mag state.
    3. Blocking commands are labeled as blocking.
    4. Commands that are unsafe during tracking either reject or enter safe mode.

---

## D3. Remove or mark placeholder commands

If a command exists, it must either work or clearly say NOT_IMPLEMENTED/EXPERIMENTAL.

Examples to audit:

    output mode slimevr
    packetFormat = slimevr/binary
    mag calibration fields not actually used
    AHRS config fields not applied

Rules:

    1. Production command must work.
    2. Experimental command must print experimental status.
    3. Not implemented command must not change config to fake working state.
    4. Config print must not imply inactive fields are active.

Acceptance criteria:

    1. No user can enable “slimevr output” if no real UDP backend exists.
    2. No user can tune AHRS config value that runtime ignores.
    3. Placeholder fields are removed, hidden, or clearly marked.

---

# Phase E — config architecture

## E1. Separate config into logical blocks

Even if physical storage is one blob, code should be organized into blocks:

    HardwareDefaults / HardwareConfig
    ImuConfig
    FifoConfig
    AhrsConfig
    MagConfig
    OutputConfig
    CalibrationConfig
    DebugConfig optional

CalibrationConfig should contain:

    gyro bias calibration;
    gyro temperature compensation;
    accel calibration;
    mag calibration;
    mag axis validation;
    calibration quality metadata.

Acceptance criteria:

    1. Config structure is readable by domain.
    2. Calibration data is not mixed with temporary runtime counters.
    3. Defaults are created from defines.h.
    4. Config apply/capture functions are domain-specific.

---

## E2. Add config schema and migration policy

Current strict magic/version/size/CRC is safe but not enough for final product.

Need:

    schema version;
    block version;
    migration functions;
    clear error reporting;
    partial invalidation;
    factory reset path;
    config dump/import optional.

Migration behavior:

    if full config version old but migratable:
      migrate fields;
      set new defaults for new fields;
      preserve valid calibration blocks;

    if one calibration block invalid:
      invalidate that block only if possible;
      do not erase unrelated valid blocks;

    if CRC invalid:
      reject full blob and enter safe defaults.

Acceptance criteria:

    1. Adding a field does not silently wipe all calibrations if migration is possible.
    2. Status shows why config was rejected/migrated.
    3. Factory reset behavior is explicit.
    4. Partial calibration validity is supported.

---

## E3. Make CRC robust

Avoid CRC over uninitialized padding.

Options:

    1. Ensure all config structs are zero-initialized.
    2. Add static_asserts for sizes.
    3. Use field-by-field serialization for CRC.
    4. Use packed storage structs carefully.
    5. Store block-level CRCs.

Acceptance criteria:

    1. CRC does not depend on random padding.
    2. Config saved twice without changes produces same CRC.
    3. Config layout changes are detected.
    4. Migration path handles old sizes/versions.

---

## E4. Separate apply and capture paths

Config should not magically mutate runtime or vice versa.

Required APIs:

    applyToRuntime(config, runtime)
    captureFromRuntime(runtime, config)
    applyCalibration(config.calibration, calibrationRuntime)
    captureCalibration(calibrationRuntime, config.calibration)

Rules:

    1. Loading config applies settings explicitly.
    2. Saving config captures runtime explicitly.
    3. Calibration save is explicit and traceable.
    4. Runtime temporary data is not accidentally persisted.

Acceptance criteria:

    1. config save does not save incomplete temporary calibration.
    2. cal save saves only valid computed calibration.
    3. config load resets dependent runtime state where needed.
    4. AHRS/mag state is reset after relevant calibration reload.

---

# Phase F — dead code, legacy code, and experimental separation

## F1. Audit all files

Create module inventory:

    file path;
    purpose;
    owner domain;
    included by;
    used by;
    production/debug/experimental/legacy;
    keep/delete/refactor decision.

Known candidates to audit:

    src/defines.hpp
    src/debug/rolling_gyro_stats.hpp
    src/sensor/ahrs_go.hpp
    src/network/ empty folder
    old gyro calibration classes if replaced by FIFO-compatible calibration
    outdated comments in lsm6dsv_driver.hpp

Acceptance criteria:

    1. Every file has a known purpose.
    2. No unused production files remain.
    3. Legacy code is deleted or moved to experimental with explanation.
    4. Empty folders are removed unless documented.

---

## F2. Remove dead files safely

Procedure:

    1. grep references;
    2. build;
    3. remove file;
    4. build;
    5. run smoke test;
    6. commit as separate cleanup.

Do not delete:

    files that are part of current runtime;
    files needed by tests/tools;
    files intentionally kept in experimental with README.

Acceptance criteria:

    1. Project builds after cleanup.
    2. No include paths point to deleted files.
    3. No duplicate AHRS/calibration implementation remains in production path.

---

## F3. Separate experimental features

Create:

    src/experimental/
    tools/experimental/
    docs/experimental/

Move or create there:

    MEKF/ESKF prototype;
    Earth rotation compensation;
    coning variants;
    online mag calibration prototypes;
    alternative AHRS filters;
    debug-only AHRS implementations.

Rules:

    1. Experimental code is not compiled into production build by default.
    2. Experimental config options are hidden unless feature flag enabled.
    3. Experimental CLI commands are clearly marked.
    4. Experimental code cannot change production behavior silently.

Acceptance criteria:

    1. Production build does not depend on experimental code.
    2. Experimental features require explicit build flag.
    3. README explains each experimental module.

---

# Phase G — interfaces and dependency boundaries

## G1. Reduce global state

Current project uses many global runtime objects.

Goal:

    group runtime state into explicit objects.

Suggested top-level:

    TrackerRuntime runtime;
    TrackerConfig config;
    TrackerConfigStore configStore;
    TrackingState trackingState;
    FifoRuntime fifoRuntime;
    MagRuntime magRuntime;
    StaticTestRunner staticTest;
    OutputManager outputManager;

Rules:

    1. Global hardware objects are acceptable if needed for embedded setup.
    2. Algorithm modules should receive dependencies explicitly.
    3. CLI should not directly mutate random globals.
    4. Runtime state ownership should be clear.

Acceptance criteria:

    1. Most globals in main.cpp are removed or grouped.
    2. Command handlers use context/hooks.
    3. Unit/replay tests can instantiate AHRS/mag/static test without Arduino globals.

---

## G2. Define narrow interfaces

Interfaces do not need to be virtual classes. They can be structs of pointers/functions.

Useful interfaces:

    IImuSampleSource
    IFifoEventSource
    ITrackingFilter
    IMagHeadingProvider
    ICalibrationProvider
    IConfigStore
    IOutputSink
    ICommandRuntimeHooks

Acceptance criteria:

    1. AHRS does not depend on Serial.
    2. Mag correction does not depend on CLI.
    3. Static test does not depend on main.cpp globals.
    4. Output code does not own tracking logic.

---

## G3. Separate hardware drivers from algorithms

Drivers should do:

    register read/write;
    sensor configuration;
    raw FIFO parsing;
    raw sample delivery;
    status/error reporting.

Algorithms should do:

    scaling;
    calibration;
    AHRS;
    mag heading;
    yaw correction;
    quality assessment.

Avoid:

    driver applying AHRS logic;
    AHRS reading hardware directly;
    calibration code printing directly to Serial.

Acceptance criteria:

    1. LSM/QMC drivers are usable without tracking runtime.
    2. AHRS can run on recorded samples.
    3. Calibration can run on captured sample arrays.
    4. Drivers expose errors, not policy decisions.

---

# Phase H — logging and debug output structure

## H1. Separate human-readable CLI from machine-readable logs

Human-readable:

    help;
    status;
    config print;
    calibration report.

Machine-readable:

    RAW,...
    CAL,...
    Q,...
    MAG,...
    YAW,...
    FIFO,...
    STATE,...

Rules:

    1. Machine-readable logs must have stable format.
    2. Human-readable messages can be verbose.
    3. Replay tools parse only machine-readable logs.
    4. Debug stream must be rate-limited.

Acceptance criteria:

    1. Changing help text does not break replay parser.
    2. Machine logs include version/prefix if format changes.
    3. Logging cannot block FIFO runtime for too long.
    4. Stream mode can be disabled fully.

---

## H2. Centralize output formatting

Do not spread Serial.print formatting across tracking modules.

Create:

    serial/formatters.hpp
    runtime/output_stream.hpp
    output/serial_text_output.hpp

Rules:

    1. Algorithm modules return data/status.
    2. Output modules format data.
    3. CLI handlers may print reports but not implement tracking logic.

Acceptance criteria:

    1. AHRS code has no Serial.print.
    2. Calibration algorithms do not require Serial.
    3. Reports can be generated from data structs.

---

# Phase I — tests and host-side tools

## I1. Add host-side tests for math/tracking modules

Test on PC where possible.

Targets:

    core/math.hpp:
      quaternion multiplication;
      normalization;
      vector rotation;
      axis-angle conversion;
      yaw extraction if implemented.

    sensor/ahrs_6dof.hpp:
      dt handling;
      rejected timestamp behavior;
      accel correction gates;
      quaternion norm.

    sensor/mag_yaw_correction.hpp:
      yaw sign;
      rate limiting;
      innovation gate;
      stale/reference reset behavior.

    sensor/accel_calibration.hpp:
      six-face solve;
      bad face detection;
      residuals.

    sensor/mag_calibration.hpp:
      sphere/ellipsoid solve;
      poor coverage rejection.

    connection/lsm6dsv_fifo.hpp:
      timestamp wrap;
      fallback timestamp;
      tag parsing;
      overrun/recovery reporting.

Acceptance criteria:

    1. Core tracking math can be tested without ESP32.
    2. Known edge cases have tests.
    3. Regression tests catch timestamp/reset bugs.
    4. Tests are documented and easy to run.

---

## I2. Add replay tools

Tools:

    tools/replay/parse_log.py
    tools/replay/replay_ahrs.py
    tools/replay/compare_6dof_mag.py
    tools/replay/metrics.py
    tools/replay/plot_quat.py
    tools/replay/plot_mag.py

Purpose:

    compare filter changes;
    tune gains;
    test mag rejection;
    test future ESKF offline;
    reproduce bugs from logs.

Acceptance criteria:

    1. One log can be replayed through multiple settings.
    2. Metrics are automatically generated.
    3. Firmware changes can be compared against baseline logs.
    4. Replay format is documented.

---

## I3. Add config tests

Test:

    default initialization;
    CRC;
    invalid CRC;
    version mismatch;
    migration;
    partial calibration invalidation;
    apply/capture logic.

Acceptance criteria:

    1. Config corruption does not cause silent bad runtime state.
    2. Migration preserves valid calibration where possible.
    3. Config save/load is deterministic.

---

# Phase J — documentation

## J1. Required docs

Create and maintain:

    docs/architecture.md
      module overview and dependency boundaries.

    docs/tracking_pipeline.md
      raw sample to quaternion pipeline.

    docs/coordinate_frames.md
      sensor/board/tracker/world/output frames.

    docs/config_schema.md
      config blocks, versions, migration.

    docs/calibration.md
      gyro, accel, mag, temp calibration flow.

    docs/cli_reference.md
      commands, side effects, blocking/non-blocking status.

    docs/test_plan.md
      required logs, metrics, acceptance tests.

    docs/experimental.md
      optional research features and build flags.

Acceptance criteria:

    1. New contributor can understand the firmware structure.
    2. Coordinate frames are not guessed from code.
    3. CLI side effects are documented.
    4. Config migration is documented.

---

# Phase K — security and project hygiene

## K1. Remove secrets from platformio.ini

Do not store Wi-Fi credentials or local server IP in committed project config.

Move to one of:

    secrets.ini ignored by git;
    local build flags ignored by git;
    NVS/provisioning;
    serial setup command;
    environment variables.

Rules:

    1. platformio.ini may contain placeholders.
    2. Real SSID/password must not be committed.
    3. Example config can be committed.

Acceptance criteria:

    1. No real credentials in repository.
    2. Project still builds with example/default config.
    3. User-specific config path is documented.

---

## K2. Normalize repository hygiene

Add/update:

    .gitignore
    README.md
    platformio example config
    docs/
    tools/
    test/

Ignore:

    .pio/
    build outputs;
    logs unless intentionally saved;
    local secrets;
    local calibration dumps if device-specific/private.

Acceptance criteria:

    1. Clean clone builds with documented steps.
    2. Local private files are not committed accidentally.
    3. Logs/calibration exports have clear policy.

---

# Phase L — output/network code quality

This roadmap is about code quality, not implementing SlimeVR output, but placeholders must be handled.

## L1. Output backend abstraction

Create output abstraction before adding real protocol backends.

Interface:

    begin();
    poll();
    sendQuaternion(sample);
    sendStatus(status);
    isConnected();
    getError();

Backends:

    SerialTextOutput
    SlimeVrUdpOutput only when implemented
    DebugOutput optional

Acceptance criteria:

    1. Tracking runtime does not know packet details.
    2. Serial output and SlimeVR output are separate.
    3. Unimplemented backend cannot be selected as working mode.
    4. Output failure does not block tracking loop.

---

## L2. SlimeVR placeholder policy

Until real UDP backend exists:

    output mode slimevr must return NOT_IMPLEMENTED;
    config must not persist fake slimevr active state;
    status must say network output unavailable in this build.

When implemented:

    put it in output/slimevr_udp_output.hpp/.cpp and network/ modules;
    include packet writer tests;
    verify quaternion order/frame;
    avoid mixing network code into AHRS/runtime internals.

Acceptance criteria:

    1. No fake SlimeVR mode.
    2. Protocol code is isolated.
    3. Tracking quality does not depend on network output.
    4. Network reconnect cannot block FIFO/AHRS.

---

# Phase M — coding rules

## M1. Embedded memory rules

Allowed:

    static allocation;
    fixed-size buffers;
    stack allocation with known small sizes.

Avoid in production runtime:

    new/delete;
    malloc/free;
    Arduino String;
    std::vector;
    std::string;
    heap-heavy containers.

Current project is already good here; preserve this property.

Acceptance criteria:

    1. No heap allocation in real-time tracking path.
    2. No Arduino String in CLI/runtime.
    3. Buffer sizes are defined and bounded.
    4. Parser handles overflow safely.

---

## M2. Time and units rules

All time values must have units in names:

    timestampUs
    dtUs
    dtSeconds
    periodUs
    timeoutMs

All sensor values must have units in names/types where possible:

    gyroRadS
    gyroDps
    accelG
    accelMs2
    magRaw
    magCal
    tempC

Rules:

    1. Do not mix dps and rad/s silently.
    2. Do not mix microseconds and milliseconds.
    3. Conversion functions should be explicit.

Acceptance criteria:

    1. Function signatures reveal units.
    2. Scaling code is centralized.
    3. AHRS receives gyro in expected units only.

---

## M3. Error handling rules

Every hardware/config/runtime operation should return meaningful status.

Use:

    enum class ErrorCode
    bool + lastError
    result structs

Avoid:

    silent fallback without counters;
    Serial-only error reporting;
    ignoring return values.

Acceptance criteria:

    1. Sensor init failure reaches tracking state.
    2. Config load failure is visible.
    3. FIFO recovery failure is visible.
    4. CLI/status can print last errors.

---

## M4. Header/source split

Large modules should not remain header-only unless there is a reason.

Move implementation to .cpp for:

    tracker runtime;
    config store;
    serial commands;
    static test;
    output backends;
    network code.

Header-only acceptable for:

    small math types;
    tiny templates;
    constexpr definitions;
    simple inline functions.

Acceptance criteria:

    1. Large 1000+ line header files are eliminated or justified.
    2. Compile times improve.
    3. Dependencies become clearer.

---

# Mandatory final acceptance checklist

The code-quality roadmap is complete when:

    1. main.cpp is small and only orchestrates setup/loop.
    2. compile-time defaults live in defines.h.
    3. no duplicated source-of-truth constants exist.
    4. project builds cleanly with strict warnings.
    5. serial CLI is split by domain.
    6. tracking runtime is split from CLI/config/output.
    7. static test is a module, not embedded in main.cpp.
    8. mag runtime is a module with explicit reset/state.
    9. config has schema/version/migration policy.
    10. config CRC is deterministic and not padding-fragile.
    11. placeholder features are removed, hidden, or marked NOT_IMPLEMENTED.
    12. dead/legacy files are deleted or moved to experimental.
    13. experimental code is not compiled into production by default.
    14. no real Wi-Fi credentials are committed.
    15. logs are separated into human-readable and machine-readable.
    16. tracking math can be tested/replayed off-device.
    17. coordinate frames/config/CLI/test plan are documented.
    18. output/network code cannot block tracking.
    19. no heap/String usage appears in real-time tracking path.
    20. command side effects are documented and predictable.

---

# Recommended implementation order

## Step 1 — stop obvious debt from growing

    1. Create real defines.h.
    2. Move compile-time constants into defines.h.
    3. Remove/replace unused defines.hpp.
    4. Fix use-before-declaration compile issue.
    5. Enable baseline warnings.
    6. Fix current warnings.

## Step 2 — split the largest files

    7. Move static test out of main.cpp.
    8. Move FIFO runtime out of main.cpp.
    9. Move mag runtime out of main.cpp.
    10. Move output stream out of main.cpp.
    11. Create TrackerRuntime.
    12. Reduce main.cpp to setup/loop orchestration.

## Step 3 — split CLI

    13. Create command_context.
    14. Create command_parser.
    15. Split commands by domain.
    16. Document command side effects.
    17. Remove/mark placeholder commands.

## Step 4 — clean config

    18. Split config into logical blocks.
    19. Add schema/migration policy.
    20. Make CRC deterministic.
    21. Separate apply/capture paths.
    22. Add partial calibration validity.

## Step 5 — remove dead code

    23. Create module inventory.
    24. Delete or move legacy files.
    25. Move experimental code to src/experimental.
    26. Remove empty/stale folders and comments.

## Step 6 — add tests/tools/docs

    27. Add host-side math/tracking tests.
    28. Add replay tools.
    29. Add config tests.
    30. Add docs/architecture.md.
    31. Add docs/tracking_pipeline.md.
    32. Add docs/coordinate_frames.md.
    33. Add docs/cli_reference.md.
    34. Add docs/test_plan.md.

## Step 7 — harden output/project hygiene

    35. Remove credentials from platformio.ini.
    36. Add secrets/example config workflow.
    37. Add output backend abstraction.
    38. Make unimplemented SlimeVR output impossible to enable silently.
    39. Ensure output/network cannot block tracking loop.

---

# Final principle

Качество кода здесь нужно не ради красоты.

Оно нужно, чтобы можно было безопасно улучшать tracking:

    исправлять AHRS;
    менять mag compensation;
    добавлять calibration;
    проверять FIFO recovery;
    внедрять ESKF offline/experimental;
    не ломать CLI/config/output случайно.

Финальный проект должен быть устроен так, чтобы каждый модуль имел одну понятную ответственность:

    driver читает sensor;
    calibration исправляет raw values;
    AHRS считает orientation;
    mag runtime решает, можно ли доверять magnetometer;
    tracking state объясняет состояние системы;
    output отправляет результат;
    CLI только управляет и диагностирует;
    config хранит настройки и calibration;
    main.cpp только связывает всё вместе.
# Roadmap: Quality Orientation Tracking для SlimeVR IMU-трекера

## Главная цель

Сделать прошивку, которая даёт качественный, стабильный и удобный orientation tracking:

- стабильный quaternion без резких скачков;
- минимальный yaw drift в нормальных условиях;
- стабильный roll/pitch;
- автоматический запуск без ручного ввода команд при каждом включении;
- сохранение калибровок в NVS/flash;
- автоматический fallback в 6DoF, если magnetometer ненадёжен;
- понятный tracking state/confidence;
- невозможность “тихо” работать в плохом состоянии, притворяясь нормальным трекером.

Финальная стратегия tracking:

    Gyro = основной источник динамики и кратковременного движения.
    Accel = gravity reference для roll/pitch, только когда accel trustworthy.
    Mag = медленная yaw-only compensation, только когда magnetic field trustworthy.
    Плохой magnetometer должен быть отключён, а не использован “чуть-чуть”.

Прошивка не должна быть “9DoF любой ценой”.

Правильный финальный режим:

    robust 6DoF gyro+accel core
    + runtime gyro bias refinement
    + gyro temperature compensation
    + validated accel calibration
    + validated mag calibration
    + cautious yaw-only mag compensation
    + fallback to 6DoF when mag is bad
    + tracking state/confidence
    + machine-readable diagnostics

---

# Product UX: как это должно работать для пользователя

## Обычный запуск

Пользователь просто включает трекер.

Прошивка автоматически:

1. Загружает config/calibration из NVS.
2. Проверяет валидность config.
3. Проверяет валидность gyro/accel/mag/temp calibration.
4. Инициализирует IMU, FIFO, timestamps, magnetometer.
5. Применяет сохранённые calibration.
6. Запускает AHRS.
7. Запускает 6DoF tracking.
8. Если mag calibration валидная и magnetic field выглядит нормальным — включает mag yaw compensation.
9. Если magnetic field плохой — остаётся в 6DoF и помечает DEGRADED_MAG или TRACKING_6DOF.
10. Runtime gyro bias estimator уточняет bias позже, когда трекер неподвижен.

Команды при каждом запуске вводить не нужно.

---

## Первый запуск / полный setup

Должен быть один guided setup command, например:

    setup calibrate

Он должен провести пользователя через:

    1. gyro stationary calibration
    2. accel 6-position calibration
    3. mag calibration
    4. mag axis/sign validation
    5. calibration quality check
    6. config save
    7. tracking self-test

После этого всё сохраняется в NVS.

---

## Когда нужна ручная перекалибровка

Ручная перекалибровка НЕ нужна при каждом запуске.

Она нужна только если:

    первый запуск;
    замена платы / IMU / magnetometer;
    датчик физически повернули в корпусе;
    изменился корпус, крепёж, винты, магниты, металл рядом с magnetometer;
    mag calibration quality стала плохой;
    accel calibration invalid;
    NVS повреждён или сброшен;
    пользователь явно сделал factory reset.

Gyro bias должен уточняться автоматически в runtime при stationary condition.

---

# Основные tracking states

Нужно ввести явное состояние tracking.

Минимальный набор состояний:

    CALIBRATION_REQUIRED
    STARTUP_CONVERGENCE
    TRACKING_6DOF
    TRACKING_6DOF_MAG_YAW
    DEGRADED_TIMING
    DEGRADED_ACCEL
    DEGRADED_MAG
    RECOVERING
    SENSOR_FAULT

## Значение состояний

### CALIBRATION_REQUIRED

Нет обязательных калибровок или они invalid.

Примеры:

    нет accel calibration;
    gyro calibration invalid;
    mag correction запрошена, но mag calibration invalid;
    config CRC invalid;
    NVS пустой после factory reset.

Поведение:

    трекер может выдавать quaternion в ограниченном режиме;
    mag correction запрещена;
    confidence низкий;
    CLI/status явно говорит, чего не хватает.

---

### STARTUP_CONVERGENCE

Трекер только включился и стабилизирует начальную ориентацию.

Поведение:

    AHRS стартует;
    accel correction может быть временно сильнее;
    mag reference создаётся только если mag trustworthy;
    runtime gyro bias refinement включается только если трекер stationary.

---

### TRACKING_6DOF

Основной безопасный режим:

    gyro + accel;
    yaw может медленно drift-ить;
    mag не используется.

Причины:

    mag disabled;
    mag invalid;
    mag temporarily rejected;
    mag calibration poor;
    magnetic disturbance.

---

### TRACKING_6DOF_MAG_YAW

Лучший нормальный режим:

    gyro + accel core;
    mag используется только для slow yaw-only drift correction;
    roll/pitch не исправляются magnetometer.

---

### DEGRADED_TIMING

Проблемы с FIFO/timestamps:

    large dt gap;
    timestamp discontinuity;
    FIFO overrun;
    fallback timestamp burst;
    sample loss.

Поведение:

    confidence понижается;
    mag correction временно disabled;
    при серьёзном gap AHRS может reset/reinit;
    система переходит в RECOVERING после стабилизации.

---

### DEGRADED_ACCEL

Accel временно ненадёжен:

    accel norm далеко от 1g;
    сильное линейное ускорение;
    удар;
    вибрация;
    accel saturation.

Поведение:

    accel correction ослабляется или отключается;
    gyro integration продолжается;
    mag correction может быть отключена, если требует stable accel/tilt.

---

### DEGRADED_MAG

Mag временно ненадёжен:

    field norm abnormal;
    horizontal component abnormal;
    heading innovation too large;
    mag saturation;
    mag sample stale;
    magnetic jump;
    poor calibration;
    bad axis validation.

Поведение:

    mag yaw correction disabled;
    tracking остаётся 6DoF;
    после восстановления mag correction возвращается плавно.

---

### RECOVERING

Система восстанавливается после серьёзного сбоя:

    FIFO reset;
    sensor reset;
    large timing gap;
    AHRS reset;
    mag reference reset.

Поведение:

    confidence постепенно восстанавливается;
    mag reference создаётся заново;
    yaw correction включается только после прохождения gates.

---

### SENSOR_FAULT

Железо или driver в плохом состоянии:

    IMU не отвечает;
    WHO_AM_I wrong;
    FIFO не восстанавливается;
    magnetometer не отвечает;
    SPI/I2C/sensor hub failure.

Поведение:

    quaternion output disabled или marked invalid;
    status явно показывает fault.

---

# Output confidence / flags

Каждый quaternion output должен иметь:

    timestamp;
    quaternion;
    tracking state;
    confidence;
    active correction modes;
    quality flags;
    rejection reason flags.

Минимальные flags:

    AHRS_OK
    AHRS_RESET_RECENTLY
    TIMING_BAD
    FIFO_OVERRUN
    FIFO_FALLBACK_TS
    GYRO_SATURATION
    ACCEL_SATURATION
    ACCEL_REJECTED
    MAG_REJECTED
    MAG_STALE
    MAG_DISTURBED
    MAG_CAL_INVALID
    ACCEL_CAL_INVALID
    GYRO_BIAS_INVALID
    TEMP_COMP_OUT_OF_RANGE
    TRACKING_RECOVERING

---

# Phase A — исправить tracking-critical bugs

Эти задачи обязательны. Без них дальнейшее улучшение фильтра может давать ложный эффект.

---

## A1. Исправить AHRS timestamp semantics

Проблема:

    AHRS не должен обновлять last integration timestamp, если sample был rejected.

Нужно разделить:

    lastSeenTimestampUs
    lastIntegratedTimestampUs

или обновлять lastIntegratedTimestampUs только после реально принятого update.

Правильное поведение:

    bad dt sample:
      не интегрируется;
      не меняет lastIntegratedTimestampUs;
      увеличивает counter rejected/bad dt.

    large gap:
      не создаёт скрытый неверный dt;
      переводит tracking в DEGRADED_TIMING или RECOVERING.

Acceptance criteria:

    1. Non-monotonic timestamp не ломает следующий нормальный dt.
    2. Rejected sample не влияет на integration time.
    3. Large gap даёт явный state/flag.
    4. Quaternion не делает скачок после rejected timestamp.

---

## A2. Связать FIFO recovery с AHRS recovery

Проблема:

    FIFO/timestamp recovery не должен быть невидимым для AHRS.

События, которые должны влиять на tracking:

    FIFO overrun;
    FIFO full;
    timestamp discontinuity;
    fallback timestamp burst;
    large dt gap;
    unknown FIFO tags;
    sensor reset;
    sample loss.

Поведение:

    малый gap:
      продолжить tracking;
      понизить confidence;
      отметить flag.

    большой gap:
      отключить mag correction;
      tracking state = DEGRADED_TIMING или RECOVERING;
      возможно reset/reinit AHRS от accel.

    FIFO reset:
      сбросить timestamp reconstruction;
      сбросить mag reference/controller;
      state = RECOVERING.

Acceptance criteria:

    1. После FIFO reset mag correction не использует старую reference.
    2. После large gap confidence падает.
    3. После recovery tracking state возвращается в normal только после стабильных samples.
    4. Quaternion не получает неконтролируемый jump.

---

## A3. При AHRS reset сбрасывать весь mag-dependent state

Любой reset ориентации должен сбрасывать:

    mag heading reference;
    mag yaw correction controller;
    auto-reference state;
    last accepted heading;
    cooldown/rejection accumulators, если они завязаны на старую orientation;
    mag correction integrator, если он есть.

События, при которых это обязательно:

    ahrs reset command;
    factory reset;
    FIFO serious recovery;
    sensor reset;
    manual orientation reset;
    config/calibration reload;
    mag axis remap change;
    mag calibration change.

Acceptance criteria:

    1. После AHRS reset yaw correction не тянет quaternion к старому yaw.
    2. Mag reference создаётся заново только после good magnetic state.
    3. После calibration reload старые mag states не используются.

---

## A4. Починить static-test → gyro temperature fit workflow

Нужно разделить:

    activeStaticTest
    lastCompletedStaticTest

activeStaticTest — текущий тест.

lastCompletedStaticTest — последний завершённый тест, доступный для анализа и temp fit.

lastCompletedStaticTest должен хранить:

    duration;
    sample count;
    gyro mean;
    gyro stddev/residual;
    accel norm mean/std;
    temperature min/max/mean;
    temperature coverage;
    timestamp stats;
    quality flags;
    estimated gyro bias;
    drift metrics.

Acceptance criteria:

    1. Провёл static test.
    2. Тест завершился.
    3. Данные не сбросились.
    4. Можно выполнить temp fit.
    5. Можно сохранить temp compensation в config.
    6. Новый static test заменяет lastCompleted только после успешного завершения.

---

## A5. Убрать hardcoded accel calibration как universal default

Проблема:

    Чужая accel calibration может испортить roll/pitch на другом устройстве.

Правильное поведение:

    если NVS пустой:
      accel calibration invalid;
      state = CALIBRATION_REQUIRED или DEGRADED_ACCEL;
      tracking может работать, но с warning;
      mag correction запрещена, потому что tilt может быть неточным.

    если есть factory calibration:
      она должна быть unit-specific;
      явно импортирована/записана;
      иметь serial/device id или checksum.

Acceptance criteria:

    1. Новое устройство не получает calibration от другого устройства.
    2. Config print ясно показывает accelCal.valid=false.
    3. Mag correction не включается без валидной accel calibration.
    4. Пользователь видит понятное сообщение, что нужна accel calibration.

---

# Phase B — сделать 6DoF core крепким

6DoF core должен быть хорошим сам по себе. Mag correction — дополнение, а не костыль.

---

## B1. Реально применить AHRS config

Все AHRS параметры должны быть в config/defaults, а не случайно зашиты в коде.

Параметры:

    accel correction gain;
    accel norm good threshold;
    accel norm bad threshold;
    accel innovation threshold;
    max accel correction step;
    max dt;
    min dt;
    startup convergence gain;
    startup convergence duration;
    gyro saturation policy;
    large gap policy;
    AHRS reset/recovery policy.

Источник значений:

    defines.h:
      compile-time factory defaults;

    TrackerConfig/NVS:
      persisted runtime settings;

    Runtime:
      current calculated/adaptive values.

Acceptance criteria:

    1. Изменение AHRS config реально меняет поведение фильтра.
    2. Config print показывает effective values.
    3. Factory defaults воспроизводимы.
    4. В коде нет скрытых magic numbers для AHRS core.

---

## B2. Добавить runtime stationary gyro bias estimator

Startup gyro calibration недостаточно для долгих сессий.

Нужно добавить медленное уточнение gyro bias во время stationary periods.

Stationary detector должен учитывать:

    gyro norm;
    gyro variance;
    accel norm около 1g;
    accel variance;
    отсутствие saturation;
    хорошие timestamps;
    отсутствие FIFO recovery;
    температуру;
    достаточное количество samples.

Логика:

    if stationary:
      обновлять gyro bias очень медленно;
      учитывать temperature compensation;
      обновлять confidence bias estimator;

    if not stationary:
      bias не трогать.

Важно:

    Нельзя обучать gyro bias во время движения.
    Плохой stationary detector хуже, чем отсутствие runtime estimator.

Acceptance criteria:

    1. В статике gyro residual уменьшается со временем.
    2. Во время движения bias не уезжает.
    3. При прогреве bias корректируется плавно.
    4. Bias update логируется.
    5. Bias update можно отключить для A/B теста.

---

## B3. Довести gyro temperature compensation

Минимальная обязательная модель:

    bias_x(T) = bias_x_ref + slope_x * (T - T_ref)
    bias_y(T) = bias_y_ref + slope_y * (T - T_ref)
    bias_z(T) = bias_z_ref + slope_z * (T - T_ref)

Config должен хранить:

    reference temperature;
    reference bias;
    slope x/y/z;
    temperature range min/max;
    fit quality;
    valid flag;
    sample count;
    last calibration timestamp/version.

Поведение:

    если current temperature внутри calibrated range:
      применять temp compensation;

    если температура далеко за range:
      применять осторожно или помечать TEMP_COMP_OUT_OF_RANGE;
      confidence ниже.

    если temp calibration invalid:
      использовать обычный gyro bias;
      state/flags показывают отсутствие temp comp.

Acceptance criteria:

    1. Static-test temp fit работает после завершения теста.
    2. Плохой fit не сохраняется.
    3. Fit quality виден в status/config print.
    4. После прогрева residual gyro меньше, чем без temp compensation.
    5. Temp compensation можно отключить для A/B теста.

---

## B4. Улучшить accel calibration validation

Accel calibration нужна для стабильного roll/pitch и tilt compensation магнетометра.

Для каждой face capture хранить:

    face id;
    sample count;
    mean accel vector;
    stddev accel vector;
    mean gyro norm;
    accel norm mean/std;
    quality flags.

После compute хранить:

    bias;
    scale/matrix;
    post-calibration norm residual;
    face residuals;
    valid flag;
    quality score.

Проверки:

    face sample count enough;
    gyro motion low;
    accel norm plausible;
    face directions не перепутаны;
    bias physically plausible;
    scale physically plausible;
    post-cal norm close to 1g.

Acceptance criteria:

    1. Плохая face capture не принимается.
    2. Перепутанная face ловится или даёт warning.
    3. После calibration accel norm ≈ 1g на всех faces.
    4. Mag correction не включается без valid accel calibration.

---

## B5. Поддержать full 3x3 accel calibration format

Финальный runtime должен использовать формат:

    a_cal = M_acc * (a_raw - b_acc)

Где:

    b_acc = accel bias vector;
    M_acc = 3x3 correction matrix.

Даже если текущий solver выдаёт diagonal matrix, runtime/config должны поддерживать full 3x3.

Acceptance criteria:

    1. Diagonal scale является частным случаем Mat3.
    2. Runtime path всегда использует Mat3.
    3. Config может хранить full 3x3.
    4. Позже можно добавить cross-axis correction без переписывания pipeline.

---

## B6. Уточнить accel correction strategy

Accel correction должна быть adaptive.

Использовать accel correction, когда:

    accel norm близок к 1g;
    accel variance low/moderate;
    gyro не в saturation;
    нет сильного линейного ускорения;
    innovation angle plausible.

Ослаблять/отключать accel correction, когда:

    accel norm далеко от 1g;
    удар;
    вибрация;
    fast body movement;
    accel innovation too large.

Acceptance criteria:

    1. При fast shake roll/pitch не ломаются accel correction.
    2. В статике roll/pitch стабилизируются.
    3. Accel rejected % виден в логах.
    4. Accel correction не создаёт заметный lag в динамике.

---

# Phase C — сделать mag yaw compensation безопасной и полезной

Magnetometer должен уменьшать yaw drift, но не должен портить quaternion.

---

## C1. Оставить yaw-only mag correction

Запрещено делать aggressive full-vector mag correction как default.

Финальная логика:

    6DoF AHRS считает quaternion;
    mag даёт heading/yaw measurement;
    вычисляется yaw error;
    применяется маленький yaw correction step вокруг world-up;
    roll/pitch magnetometer не исправляет.

Обязательные ограничения:

    max yaw correction rate;
    max correction step per update;
    heading innovation gate;
    cooldown after rejection;
    mag correction disabled during recovery;
    mag correction disabled when accel tilt unreliable.

Acceptance criteria:

    1. Mag disturbance не создаёт roll/pitch jump.
    2. Roll/pitch движения без yaw не создают ложный yaw correction.
    3. Yaw correction smooth, без резких скачков.
    4. При плохом mag система остаётся в 6DoF.

---

## C2. Заменить min/max mag calibration на ellipsoid/full matrix calibration

Финальный формат:

    m_cal = M_mag * (m_raw - b_mag)

Где:

    b_mag = hard iron bias;
    M_mag = soft iron / scale / misalignment correction matrix.

Mag calibration должна сохранять:

    hard iron bias;
    soft iron matrix;
    expected field norm;
    expected horizontal norm;
    coverage score;
    residual error;
    sample count;
    valid flag;
    quality score;
    calibration timestamp/version.

Calibration должна reject-иться, если:

    coverage poor;
    residual too high;
    field norm implausible;
    matrix singular/unstable;
    sample count too low;
    points collected in magnetic disturbance.

Acceptance criteria:

    1. После calibration mag point cloud становится близким к sphere.
    2. Poor coverage не сохраняется как valid calibration.
    3. Residual виден в status.
    4. Mag correction не включается без good quality calibration.

---

## C3. Добавить mag calibration quality gates

Mag correction может активироваться только если:

    mag calibration valid;
    coverage score good;
    residual acceptable;
    expected field norm known;
    axis mapping validated;
    mag sample fresh;
    mag not saturated;
    field norm in allowed range;
    horizontal component in allowed range;
    heading innovation plausible;
    current tracking state allows mag correction.

Acceptance criteria:

    1. При плохой calibration mag correction не включается.
    2. При magnetic disturbance mag correction отключается.
    3. После исчезновения disturbance mag correction возвращается плавно.
    4. Rejection reason виден в логах/status.

---

## C4. Сделать mag gates относительными к calibration reference

Не полагаться только на абсолютные thresholds.

Использовать:

    mag_norm_ratio = current_norm / expected_field_norm;
    horizontal_ratio = current_horizontal / expected_horizontal_norm;

Состояния:

    good range;
    degraded range;
    reject range.

Пример логики:

    if norm ratio good and horizontal ratio good:
      mag can be trusted;

    if degraded:
      reduce mag gain or keep correction disabled;

    if reject:
      disable mag correction.

Acceptance criteria:

    1. Gates работают на разных устройствах.
    2. Gates не завязаны на одну комнату/одно поле.
    3. Calibration reference используется в runtime.

---

## C5. Guided mag axis/sign validation

Нужна процедура проверки:

    mag axis test

Она должна попросить пользователя:

    1. Повернуть трекер yaw +90°.
    2. Повернуть yaw -90°.
    3. Сделать pitch/roll checks.

Процедура должна определить:

    axis mapping OK;
    axis sign suspicious;
    heading reversed;
    horizontal projection invalid;
    mag/accel frame mismatch.

Acceptance criteria:

    1. Ошибка axis permutation/sign ловится до включения mag correction.
    2. CLI/status говорит, что именно подозрительно.
    3. Mag correction запрещена, если axis validation failed.

---

## C6. Улучшить mag timestamp / sample age handling

Для каждого mag sample нужно знать:

    sample timestamp;
    sample age relative to current AHRS timestamp;
    source of timestamp;
    fresh/stale flag.

Поведение:

    fresh mag sample:
      можно использовать, если gates OK;

    old/stale mag sample:
      reject или reduce gain;

    unknown timestamp quality:
      conservative behavior.

Acceptance criteria:

    1. Старый mag sample не применяется.
    2. При задержке mag correction ослабляется.
    3. Mag age логируется.

---

## C7. Auto-reference только при хорошем magnetic state

Mag reference нельзя создавать “когда попало”.

Auto-reference allowed only if:

    tracking state stable;
    accel trustworthy;
    mag trustworthy;
    mag calibration valid;
    heading innovation stable;
    gyro motion low/moderate;
    mag sample fresh.

Acceptance criteria:

    1. Reference не создаётся во время magnetic disturbance.
    2. Reference сбрасывается после AHRS reset/recovery.
    3. Reference status виден в CLI/status.

---

# Phase D — calibration persistence and startup behavior

Цель: нормальная работа без ручных команд при каждом запуске.

---

## D1. Config/calibration validity model

Каждая calibration должна иметь:

    valid flag;
    version;
    quality score;
    sample count;
    residual/error;
    timestamp/version;
    device-specific marker, если применимо.

Калибровки:

    gyro bias calibration;
    gyro temperature compensation;
    accel calibration;
    mag calibration;
    mag axis validation;
    AHRS/output frame config.

Acceptance criteria:

    1. Прошивка знает, какие calibration valid/invalid.
    2. Status ясно показывает, чего не хватает.
    3. Invalid calibration не применяется тихо.
    4. Partial calibration state поддерживается.

---

## D2. Automatic startup sequence

При включении:

    1. load config
    2. validate config CRC/schema
    3. validate calibration blocks
    4. init sensors
    5. init FIFO/timestamps
    6. apply calibration
    7. init AHRS
    8. enter STARTUP_CONVERGENCE
    9. enter TRACKING_6DOF
    10. enable mag yaw only if allowed by gates

Acceptance criteria:

    1. Пользователь не вводит команды для обычного запуска.
    2. Если calibration valid — tracking стартует автоматически.
    3. Если calibration missing — state говорит calibration required.
    4. Если включили в движении — tracking всё равно стартует, stationary refinement ждёт покоя.

---

## D3. One-command guided calibration

Нужна команда:

    setup calibrate

Она выполняет guided flow:

    gyro stationary calibration;
    accel 6-face calibration;
    mag calibration;
    mag axis/sign validation;
    quality report;
    save config;
    optional static self-test.

Также нужны отдельные expert-команды:

    cal gyro
    cal accel
    cal mag
    cal mag axis_test
    cal temp
    config save
    config print
    tracking status

Acceptance criteria:

    1. Обычный пользователь может выполнить полный setup одной командой.
    2. Expert commands остаются для отладки.
    3. После setup calibrate обычный restart не требует команд.

---

# Phase E — logging, metrics, tests

Нельзя улучшать tracking без измерений.

---

## E1. Machine-readable logs

Добавить стабильные форматы логов.

Обязательные строки:

    RAW,t,ax,ay,az,gx,gy,gz,temp,flags
    CAL,t,ax,ay,az,gx,gy,gz,temp,flags
    Q,t,w,x,y,z,flags,confidence,state
    MAG,t,x,y,z,norm,horizontal,heading,flags
    YAW,t,error,step,trust,rejected_reason
    FIFO,t,dt,fallback,dropped,overrun,flags
    BIAS,t,bx,by,bz,temp,source,quality
    STATE,t,state,reason,confidence

Acceptance criteria:

    1. Логи можно парсить replay tool.
    2. Human-readable help не ломает machine-readable parser.
    3. Логи имеют version/prefix.

---

## E2. Обязательные tracking tests

Минимальный набор:

    static 10 min;
    static 60 min;
    startup warm-up;
    slow yaw 360°;
    fast yaw 360°;
    roll/pitch without yaw;
    fast shake;
    magnetic disturbance near tracker;
    return from magnetic disturbance;
    power cycle after calibration;
    FIFO recovery simulation;
    tracking with mag disabled;
    tracking with mag enabled.

Acceptance criteria:

    1. Для каждого теста есть лог.
    2. Для каждого теста есть pass/fail metrics.
    3. Mag correction сравнивается с 6DoF baseline на тех же логах.

---

## E3. Metrics

Обязательные метрики:

    yaw drift deg/min in 6DoF;
    yaw drift deg/min with mag yaw;
    quaternion norm error;
    dt mean/std/max;
    fallback timestamp ratio;
    gyro residual after calibration;
    gyro bias stability;
    accel norm residual;
    accel rejected ratio;
    mag rejected ratio;
    mag correction applied ratio;
    mag heading innovation mean/max;
    max yaw correction step;
    recovery time after disturbance;
    time spent in each tracking state.

Acceptance criteria:

    1. Улучшение считается улучшением только по метрикам.
    2. Mag correction считается полезной только если уменьшает drift без jumps.
    3. Любой tracking regression виден на replay/static tests.

---

## E4. Replay harness

Нужны host-side tools:

    tools/replay/parse_log.py
    tools/replay/replay_ahrs.py
    tools/replay/compare_6dof_mag.py
    tools/replay/plot_quat.py
    tools/replay/plot_mag.py
    tools/replay/metrics.py

Replay должен позволять:

    прогонять один и тот же лог через разные настройки;
    сравнивать 6DoF vs mag yaw;
    сравнивать разные gains;
    тестировать future ESKF offline;
    проверять sign/frame bugs.

Acceptance criteria:

    1. Tracking math можно тестировать без прошивки.
    2. Любой новый фильтр сначала проверяется offline.
    3. Firmware changes сравниваются с baseline logs.

---

# Phase F — coordinate frames and output correctness

Качественный quaternion бесполезен, если frame/sign/order неверные.

---

## F1. Документировать frames

Документировать:

    raw sensor frame;
    IMU package frame;
    board frame;
    tracker/device frame;
    world frame;
    SlimeVR/output frame.

Для каждого перехода:

    axis permutation;
    sign inversion;
    rotation matrix/quaternion;
    порядок применения.

Acceptance criteria:

    1. По документации понятно, куда смотрит каждая ось.
    2. Не нужно угадывать frame из кода.
    3. Axis mapping можно проверить тестом.

---

## F2. Проверить quaternion convention

Зафиксировать:

    internal quaternion order: w,x,y,z;
    output protocol order: according to target protocol;
    multiplication order;
    q_world_from_sensor или q_sensor_from_world;
    positive-W normalization behavior.

Acceptance criteria:

    1. Поворот вокруг X/Y/Z даёт ожидаемый quaternion.
    2. Yaw correction имеет правильный знак.
    3. Output не отправляет w,x,y,z туда, где ожидается x,y,z,w.

---

## F3. Frame validation commands

Нужны команды:

    tracking frame_test
    mag axis_test
    ahrs orientation_test

Они должны помогать проверить:

    roll sign;
    pitch sign;
    yaw sign;
    world-up axis;
    mag heading sign;
    output quaternion order.

Acceptance criteria:

    1. Ошибка осей ловится до использования трекера в SlimeVR.
    2. CLI выдаёт понятный report.

---

# Phase G — acceptance definition for final mandatory tracking

Финальный обязательный tracking считается готовым, если выполнено:

    1. Обычный запуск не требует ручных команд.
    2. Калибровки сохраняются и валидируются.
    3. Без mag трекер стабильно работает в 6DoF.
    4. С good mag calibration yaw drift уменьшается.
    5. С bad magnetic field трекер уходит в 6DoF, а не портит quaternion.
    6. AHRS корректно переживает FIFO/timestamp recovery.
    7. Runtime gyro bias update работает только в stationary.
    8. Gyro temp compensation работает и имеет quality checks.
    9. Accel calibration не чужая и не hardcoded для всех устройств.
    10. Mag calibration имеет quality/coverage/residual checks.
    11. Tracking state/confidence выводятся в status/logs.
    12. Machine-readable logs позволяют воспроизвести и сравнить tracking.
    13. Coordinate frames документированы и проверены.
    14. Quaternion output имеет правильный order/sign/frame.

---

# Experimental / optional roadmap

Эти задачи не обязательны для финального базового продукта. Они должны быть отделены от production core и включаться только explicit feature flag.

---

## X1. Offline MEKF / ESKF

Сначала только offline/replay.

Минимальный state:

    nominal:
      quaternion q
      gyro bias b_g

    error state:
      small angle error δθ
      gyro bias error δb_g

Updates:

    gyro prediction;
    accel direction update;
    yaw-only mag update;
    adaptive accel measurement noise;
    adaptive mag measurement noise;
    innovation gating.

Критерий переноса в firmware:

    ESKF стабильно лучше текущего AHRS на одинаковых логах;
    нет ухудшения при magnetic disturbance;
    CPU/RAM budget приемлем;
    код поддерживаемый.

---

## X2. Coning compensation

Полезно для быстрых multi-axis rotations и FIFO batches.

Добавлять после стабильного baseline.

Acceptance criteria:

    1. На fast shake / multi-axis rotation logs ошибка меньше.
    2. Нет увеличения lag/noise.
    3. Можно отключить для A/B теста.

---

## X3. Online mag calibration with rollback

Разрешить только safe mode:

    collect samples;
    estimate coverage;
    estimate residual;
    propose new calibration;
    do not overwrite good calibration automatically;
    support rollback.

Acceptance criteria:

    1. Плохая магнитная среда не портит сохранённую calibration.
    2. New calibration применяется только после quality checks.
    3. Old calibration можно восстановить.

---

## X4. Earth rotation compensation

Не включать по default.

Полезно только если:

    gyro bias/temp stability лучше или сравнимы с Earth rate;
    известна latitude;
    есть long static validation;
    есть A/B test on/off.

Acceptance criteria:

    1. Earth compensation улучшает long static yaw drift на логах.
    2. Не ухудшает обычное движение.
    3. Off by default.

---

# Recommended implementation order

## Сначала исправить tracking-critical edge cases

    1. AHRS timestamp semantics.
    2. FIFO recovery → AHRS recovery.
    3. AHRS reset → mag state reset.
    4. Static test result persistence.
    5. Убрать hardcoded accel calibration default.

## Затем укрепить 6DoF

    6. Реально применить AHRS config.
    7. Runtime stationary gyro bias estimator.
    8. Gyro temperature compensation workflow.
    9. Accel calibration validation.
    10. Full 3x3 accel calibration format.
    11. Adaptive accel correction.

## Затем сделать mag yaw пригодным для постоянного использования

    12. Yaw-only mag correction оставить как core strategy.
    13. Ellipsoid/full matrix mag calibration.
    14. Mag calibration quality gates.
    15. Relative mag norm/horizontal gates.
    16. Guided mag axis/sign validation.
    17. Mag timestamp/sample age handling.
    18. Safe auto-reference.

## Затем product behavior

    19. Calibration validity model.
    20. Automatic startup sequence.
    21. One-command guided calibration.
    22. Tracking states/confidence.

## Затем validation

    23. Machine-readable logs.
    24. Required tracking tests.
    25. Metrics.
    26. Replay harness.
    27. Coordinate frame docs.
    28. Frame/quaternion validation commands.

## Потом experimental

    29. Offline ESKF/MEKF.
    30. Coning compensation.
    31. Online mag calibration with rollback.
    32. Earth rotation compensation.

---

# Final principle

Не цель — “всегда 9DoF”.

Цель:

    Всегда выдавать лучший безопасный quaternion из доступных данных.

Это значит:

    если gyro+accel хорошие, а mag плохой:
      хороший 6DoF лучше, чем плохой 9DoF;

    если mag хороший:
      использовать его для медленной yaw-only drift compensation;

    если timing/calibration плохие:
      явно показать degraded state, а не скрывать проблему.
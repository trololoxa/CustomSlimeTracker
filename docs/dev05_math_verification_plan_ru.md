# Tracker: спецификация математических проверок

Дата: 2026-09-23. Статус: план расширения проверок, не отчёт о выполненных новых тестах.

Основание: доступный исходный код после DEV-04b с DEV-05, AGENTS.md, coordinate_frames.md,
dev_test_map.md, dev05_algorithm_accuracy.md, firmware roadmap 0027–0038 и правила аудита.
Локальная история восстановлена из приложенных файлов; это не проверка текущего Windows checkout пользователя.
Этот документ описывает план. Он включён в DEV-05a как спецификация следующих расширений;
фактические изменения DEV-05a перечислены в [отчёте](dev05a_report.md).

## 1. Цель и границы

Стенд должен отвечать на четыре разных вопроса:

1. Правильно ли реализованы формулы, системы координат, единицы и время?
2. Соблюдаются ли контракты безопасности, свежести и восстановления?
3. Стала ли точность/задержка лучше или хуже на одинаковых входах?
4. Какой ценой это достигнуто: вычисления, stack, latency, сохранность samples?

Полный перебор всех физических движений, float-входов и бесконечных историй невозможен.
Цель — проверяемая матрица требований, входных классов, границ, переходов и опасных сочетаний.
Количество тестов или процент покрытия строк сами по себе не доказывают такую полноту.
У каждого требования должны быть owner, сценарий, независимый expected result, уровень доказательства
и явный статус: покрыто, частично, отсутствует, неприменимо.

DEV-05 уже даёт независимый double-эталон, 24 сценария модулей и A/B-сравнение.
Он ещё не является полным стендом всей математики трекера. Имеющиеся тесты других owners сохраняются;
новые проверки расширяют их или вызывают настоящие production-модули, а не вторую реализацию приложения.
Замены estimator, изменения ODR, thresholds, packet mode или добавления runtime logger здесь нет.

## 2. Математические соглашения и независимый эталон

В текущем проекте Hamilton quaternion хранится как [w,x,y,z]. Активное вращение:
v_world = q_world_from_device * v_device * conjugate(q_world_from_device).
При gyro в body frame propagation выполняется справа: q_next = q * dq_body.
World-yaw correction применяется слева вокруг world up. Это разные операции.
World up = +Z; gyro поступает в rad/s, accel — в g. В wire-layout порядок компонент
может отличаться от внутреннего; это проверяется на границе сериализации.

В config текущего исходника AHRS_ACCEL_CORRECTION_DIVISOR = 4; nominal gyro = 960 Hz,
mag = 60 Hz; prepared output ограничен интервалом не меньше 4000 us. Nominal accel-correction
cadence при полном admission — 240 Hz, но фактические update/correction counts надо измерять.
Нельзя переносить старое предположение «accel всегда 60 Hz» в тестовый эталон.

Независимый host-эталон:

- double, без вызова production quaternion/vector/normalization helpers;
- аналитические движения и Rodrigues matrices для перекрёстной проверки quaternion algebra;
- для сложной непрерывной траектории — независимая интеграция с уменьшением шага до сходимости;
  ошибка эталона должна быть заметно меньше допуска проверяемого алгоритма;
- ручные опорные положения и некоммутирующие вращения проверяют сам эталон;
- генератор движения согласует q(t), body angular velocity, ускорение и поле в одном времени;
- измерительные искажения добавляются после чистой физической истины, в соответствующей системе координат;
- float-квантизация входа и шум датчика отделяются от численной ошибки вычислений;
- усреднение за интервал, instant sample, задержка и sample-and-hold задаются явно;
  нельзя незаметно интерполировать отсутствовавшие данные.

Для принятого проектом accel-знака синтетический specific force в device frame:
f_device_g = R^T * (a_world_mps2 / g0 + world_up), g0 = 9.80665 m/s².
В покое он даёт +gravity в принятой системе координат. Для вращения с плечом датчика
учитываются angular-acceleration и centripetal terms. Плечо нулевое — отдельный простой случай.
Чистый magnetometer: m_device = R^T * B_world. Bias, scale, soft iron, alignment и noise
добавляются независимо; calibrated units фиксируются в метаданных.

## 3. Обязательная карта математических областей

«Есть» ниже означает найденный код/семейство тестов, а не повторный PASS всего семейства сегодня.

| ID | Область и production owner | Что проверять | Что измерять / ожидаемый результат | Основа и пробел |
| --- | --- | --- | --- | --- |
| M01 | Vec3/Quat/Mat3, core/math.hpp | dot/cross, norm, inverse, normalization, exp/rotation, порядок произведений, q/-q, SO(3), fast/exact branches | угловая/векторная ошибка, norm residual, RᵀR-I, det R; finite/reject; разрешение малых углов | core_math_ahrs + dev05_math_reference; добавить системные границы, произвольные оси и conditioned matrices |
| M02 | Frame/calibration chain | raw→IMU→device→world→wire, порядок bias/matrix/rotation, proper rotations, обратимость, cache revision | согласованность gyro/accel/mag, восстановление вектора, неизменность физического результата | sensor_to_device_alignment, frame_calibration_lifecycle, slimevr_motion_frame; расширять существующие |
| M03 | Gyro propagation, Ahrs6Dof | arbitrary axis, noncommuting rotations, coning, знак, единицы, dt, нормализация, длинное интегрирование | orientation error, slope drift, norm, timestamps, skipped/duplicate integration | DEV-05 частично; coning/сходимость и sweeps добавить |
| M04 | Accel correction, Ahrs6Dof | направление gravity correction, admission, missing/invalid/saturated/dynamic, innovation, накопление, reacquire | tilt error, correction-only step/rate, false acceptance на доказанно плохом входе, потеря evidence, resumption/settling | DEV-05 частично; 0029 OPEN и составные сценарии |
| M05 | Mag geometry/heading/field reliability/yaw | horizontal projection, norm/dip, squared-vs-linear scale, wrap, tilt dependency, field classification, stale cache, relock | heading observability, yaw error, norm/dip residual, correction sign/rate, reject/return | mag_heading_reliability и yaw tests уже содержательны; DEV-05 сейчас задаёт часть trust извне |
| M06 | Linear acceleration / prepared output | вычитание gravity, device/world relation, coherent q+accel timestamp, dynamic-but-publishable | error XYZ/norm в g и m/s², gravity leakage, validity, age, frame consistency | prepared_output_motion_snapshot и motion_frame; добавить известные физические траектории |
| M07 | Gyro bias / temperature | base + valid temperature delta + RAM trim, signs/units, frame, enable/disable, stationary qualification, extrapolation | bias error XYZ, residual rate/drift, convergence, learning during motion, discontinuity at switch/rollback | gyro_temp*, runtime_bias*; некоторые будущие контракты принадлежат 0031 |
| M08 | Accel calibration / frame solve | bias, full 3×3 scale/cross-axis, six faces, polar separation rotation/scale, degeneracy, invalid candidates | held-out vector/norm error, bias error, condition/determinant, proper rotation, rejection reason | sensor_calibration, sensor_to_device_alignment; независимый synthetic holdout расширять |
| M09 | Mag calibration / axis alignment | ellipsoid, SPD square root, hard/soft iron, 24 proper mappings + continuous SO(3), reflected inputs, timing | held-out corrected-vector/radius error, alignment angle, conditioning, coverage, degeneracy | mag_calibration, mag_heading_reliability уже проверяют ряд сложных случаев; не дублировать |
| M10 | Statistics / gates / confidence | stable mean/variance/covariance, denominator, ring eviction, EWMA, hysteresis, thresholds, finite samples only | reference error, accepted counts, time dependence, monotonic trust, stale/invalid contributions | разные owners; дополнить независимыми batch-double references |
| M11 | Time/FIFO/multirate arithmetic | ticks→us, frequency correction, fractional cadence, wrap, sample pairing, batches, rebase | dt error, monotonicity, count/order, epoch, association error, maximum age | fifo_pair_coherency, fifo_runtime_processor, imu_quality, recovery tests |
| M12 | Output numerical boundary | g→m/s², quaternion layout/sign, quantization/clipping для существующих modes, finite guards, snapshot coherence | decoded angle/vector error, range handling, same-sample q+accel | packet_writer, output_runtime, motion_frame; current Server interop отдельно |
| M13 | Auxiliary numerical owners | ADC/divider/filter battery, elapsed/rate counters, будущий fractional scheduler и adaptive rate caps | units, rounding, bounds, wrap, freshness, exact average/no burst | battery/runtime tests; формулы 0033 проверять при его реализации, не менять rate сейчас |

Для calibration параметры сравниваются только при зафиксированной параметризации и gauge.
Одинаковая физическая коррекция может иметь несколько алгебраических представлений; главный критерий —
ошибка на независимых held-out ориентациях и правильное разделение SPD/rotation.
Train/holdout разделяются по независимым сегментам/покрытию, а не соседними коррелированными samples.

## 4. Направления проверки каждой области

1. Алгебра: тождества, знаки, размерности, порядок операций, frame semantics.
2. Численная устойчивость: float resolution, cancellation, near-zero, overflow/underflow,
   плохо обусловленные matrices, branch boundaries, finite validation до нормализации.
3. Физика: согласованные движение, gravity, field, bias, noise; наблюдаемость.
4. Время: реальный sample dt, очередь/задержка, неравномерность, wrap, discontinuity.
5. Состояния: boot/reset/recovery/reconfigure/rollback и поведение накопленных statistics.
6. Точность: static/dynamic error, drift, noise rejection, gain/lag, convergence, worst case.
7. Отказоустойчивость: bad input не создаёт fresh valid output и не блокирует исправный gyro.
8. Стоимость: host test overhead отдельно от target cycles/stack/heap/deadlines.

Сначала проверяется finite/norm исходного выхода. Нормализация для вычисления angular metric
не должна скрывать невалидный quaternion production-модуля.
Scalar variance/gate checks должны учитывать count и finite denominator, а не только итоговое значение.

## 5. Генератор сценариев и матрица покрытия

| Измерение | Обязательные классы |
| --- | --- |
| Положение | шесть основных положений, произвольные SO(3), upside-down, Euler singularity neighbourhoods; равномерное покрытие rotations, не равномерные Euler angles |
| Движение | покой, constant rate XYZ/oblique, знакопеременные движения, noncommuting composition, coning, smooth ramps, stop/start/reversal, bounded impulses, sine/chirp, realistic mixed motion |
| Интенсивность | возле нуля/deadband, возле gates, штатная, высокая допустимая, saturation, вне valid dt×rate диапазона; high rate без invalidity не объявлять hardware fault |
| Accel | clean gravity, translation, vibration, impact, centripetal force, freefall/zero norm, axis saturation, partial missing, NaN/Inf, повторенный/stale sample, persistent dynamics→clean window |
| Mag | clean, weak horizontal field, high dip, near-vertical field, amplitude/direction step, transient/persistent disturbance, saturation/NaN, missing tags/cache stale, changed environment→probation/rollback |
| Gyro | zero, signed rates, bias, slow bias ramp, fixed-seed noise, quantization, clipping, NaN/Inf, missing/duplicate/backward/gap |
| Начальная ошибка | малые/средние/большие tilt и yaw ошибки; 0, 25, 90, 170, 180 и окрестности 180 градусов; несколько осей, q/-q |
| Temperature | constant, monotonic ramp, heating/cooling, fresh/stale/NaN/jump, range boundary, bounded extrapolation, insufficient fit coverage |
| Время | exact nominal cadence, real configured period/frequency trim, jitter, skew, irregular mag, latency/backlog, packet/batch boundaries, wrap, independent host/sensor clocks |
| Жизненный цикл | cold/warm initialization, manual FIFO reset, failed/successful recovery, sleep/wake, frame/calibration changes, rollback, enable/disable, repetition after prior failure |
| Численные границы | ноль/-0, subnormal где уместно, very small finite, max relevant magnitude, near-singular matrices, threshold predecessor/equal/successor |

Для float thresholds использовать nextafter ниже/выше порога, а не универсальное «±0.001».
Для времени и count — boundary-1/boundary/boundary+1, ноль, wrap и длинный интервал
с учётом разрешённого горизонта wrap-safe arithmetic.

Нельзя запускать полное декартово произведение таблицы на каждом commit.
Обязательны:

- каждый входной класс и граница отдельно;
- каждый разрешённый/запрещённый переход и возвращение к штатной работе;
- pairwise combinations для независимых факторов;
- отдельные опасные тройки и временные последовательности (ниже);
- воспроизводимые property-based/random sequences с seed и bounded length;
- минимизация найденного сбоя до короткого regression fixture;
- отдельная расширенная кампания для relevant long-run/noise/aliasing риска.

Обязательные составные сценарии:

1. Missing accel + длительное вращение + накопленный tilt drift → fresh clean gravity.
2. Invalid accel + valid gyro → проверить реальный шаг q и integrated timestamp, не bool update().
3. Mag disturbance + yaw drift + временно untrusted tilt → возврат tilt → clean mag.
4. Clean mag + bad tilt: candidate evidence допустима, yaw correction закрыта.
5. Persistent dynamic accel / доказанная magnetic interference: timeout не force-accept.
6. FIFO gap + partial accumulator + mag callback → новая epoch без stale accel/mag.
7. Все relative callback phases, в том числе before/after update с одинаковым timestamp,
   изменение batching/backlog без изменения sample order.
8. Temperature stale + motion + accumulated trim → base/frozen policy по текущему контракту;
   motion не обучает bias, trim выключается без изменения persisted base.
9. Calibration/frame revision в накопленном окне → reset именно зависимой evidence;
   rollback возвращает прежнюю модель и соответствующую ей frame/validity semantics.
10. Recovery succeeded → evidence spoiled → recovery again: first recovery не перезаписывается,
    оба episodes видны, стабильность после возврата измеряется отдельно.
11. Sleep/wake или reinit + timestamp wrap + старый cached mag: свежесть не наследуется.
12. Длительная магнитная потеря при исправном gyro/accel: 6D продолжает выдавать ориентацию;
    после достаточного clean observable window 9D возвращается без ручного reset.

Не все эти сценарии уже реализованы. Для full lifecycle нужна интеграция настоящих owners,
а не ручная подстановка fieldReliable/trusted, как в части текущих DEV-05 module probes.

## 6. Наблюдаемость: не требовать физически невозможного

- Без внешнего heading reference абсолютный yaw в 6D не наблюдаем. Можно измерять gyro integration
  against synthetic truth и induced bias drift; нельзя требовать автоматический возврат абсолютного yaw.
- Ускорение и gravity не всегда разделимы по одному accelerometer. Стабильный norm около 1 g
  не доказывает покой. Для неоднозначных входов нужен статус insufficient observability и измерение
  поведения policy, а не выдуманная гарантия распознавания каждого движения.
- При почти вертикальном поле или ненадёжном tilt heading не имеет достаточной опоры.
- Однородное изменённое magnetic field может быть неотличимо от другого окружающего поля/heading.
  Инъекция помехи известна генератору, но не обязательно определима из доступных firmware observations.
  Проверять fail-closed по доказательствам, probation и bounds; не обещать универсальную классификацию.
- При ровно противоположном gravity vector ось tilt relock неоднозначна. Требуются finite,
  bounded и воспроизводимая политика, уменьшение tilt при наблюдаемой gravity; не единственный quaternion
  или восстановление абсолютного yaw. Окрестности 180 градусов проверяются с обеих сторон.

В отчёте различать физически unobservable, реализационно rejected и сломанный observable recovery.
Наблюдаемость не назначается задним числом самим тестируемым фильтром, чтобы скрыть его ошибку.

## 7. Метрики точности и восстановления

Для нормализованных q_est и q_ref:

q_error = q_est * conjugate(q_ref)
angle_error = 2 * atan2(norm(q_error.xyz), abs(q_error.w)).

Это кратчайшая SO(3) ошибка, инвариантная к q/-q. Итог переводится в градусы.
Эталон и преобразование ошибки не используют production math. Сам эталон имеет тесты.

Обязательные результаты по каждому сценарию и фазе (до отказа / во время / после):

| Группа | Поля |
| --- | --- |
| Ориентация | RMS, median, p95, p99 при достаточном числе samples, max, final; tilt отдельно |
| Yaw | heading error на наблюдаемых участках; 6D integration drift отдельно; wrap-aware unwrap, slope deg/min, accumulated signed drift |
| Публикации | finite/norm violations, expected/consumed/published counts, missing/duplicate/reordered, blackout maximum |
| Динамика | gain/amplitude error, signed phase и delay по частоте, overshoot, settling, jitter/noise RMS/PSD на стационарных окнах |
| Correction | correction-only step/rate, wrong-way events в контролируемых применимых случаях, evidence losses, reason/dwell/transitions |
| Recovery | начало достаточной clean evidence, первая разрешённая коррекция, первое устойчивое достижение допуска, endpoint retention, повторные episodes |
| Calibration/bias | held-out XYZ/norm/direction error, bias XYZ error, conditioning/coverage, fit residuals, false candidate acceptance |
| Freshness/time | sample age, integrated timestamp, dt error, sensor/host domain, epoch, delay of q/accel/mag association |
| Стоимость | host wall time отдельно; target cycles/WCET/stack/heap/backlog/deadlines отдельно при наличии измерения |

Recovery latency считается от момента, когда вход действительно стал достаточно чистым/наблюдаемым,
а не от начала помехи. first correction и settled orientation — разные метрики.
Первое прошедшее dwell фиксируется один раз; удержание результата и последующие episodes — отдельно.
Время clean dwell, точный первый sample, observation horizon и tolerance сохраняются.
Не восстановился в пределах горизонта = not recovered within horizon, а не ноль или доказанное «никогда».
Значение не измерялось/не определено = null + reason, не 0.

Euler yaw возле вырождения выбранной heading projection не усредняется как достоверное число.
Указать валидную геометрию heading/разложения и её границы; SO(3)/tilt остаются отдельными метриками.
При смене знака представления q скачка физической ориентации нет.
Допустима заранее определённая initial frame alignment для ground truth; per-frame alignment,
best time shift или удаление drift, скрывающие ошибку/latency, запрещены в основном verdict.
Если диагностическая компенсированная метрика нужна, она выводится отдельно от raw.

## 8. A/B сравнение и статистика

Сравнение implementation-only требует одинаковых input/truth, harness, config, sample timing,
initial state, seeds, compiler/platform/flags/sanitizer и метрик. Исходники candidate отличаются ожидаемо.
Протокол/config/threshold experiment — отдельный явно названный режим сравнения с перечнем различий,
а не молчаливое ослабление strict compatibility. В текущем DEV-05 такого режима нет.

Группы noise/bias используют несколько фиксированных seeds с парным before/after сравнением.
Количество seeds увеличивается по требуемой разрешающей способности, а не ради красивой цифры тестов.
Confidence intervals считаются по независимым trials/segments; тысячи соседних samples не являются
тысячами независимых экспериментов. Report хранит ensemble definition и uncertainty.
Одного seed достаточно для воспроизводимой регрессии, недостаточно для общего stochastic improvement claim.

Допуски задаются по виду проверки:

- algebra: численная погрешность + обусловленность + точность эталона;
- physical accuracy: baseline и обоснованный budget, uncertainty и practical relevance;
- contracts: explicit exact/bounded invariants;
- performance: сопоставимый target baseline;
- known defects: явно открытые требования, без автоматического принятия новых нарушений.

Нельзя усреднить улучшение одного сценария и ухудшение другого в общий PASS.
Снижение accepted percentage не является само по себе регрессией; повышение — улучшением.
Нельзя улучшать RMS выбросив плохие/stale samples, заморозив quaternion или уменьшив publication rate:
все пропуски учитываются в coverage/blackout; метрики valid outputs идут вместе с этим verdict.
Длительное motion без коррекции допустимо при отсутствии observable reference; это не равнозначно
неисправности. Но старый drift/reject epoch не должен блокировать первый достаточный clean window.

## 9. Проверка стенда и критерии verdict

Сам стенд должен обнаружить контролируемые ошибки:

- обратный gyro multiplication order, wrong accel sign, world/body yaw multiplication;
- deg вместо rad, ms вместо us, factor g вместо g², linear ratio вместо squared geometry;
- stale accepted as fresh, missing accel добавляет 0 в mean/variance;
- bool success без движения q/timestamp, замороженный output, потерянный sample;
- перестановка XYZ/wxyz, reflection вместо proper rotation;
- неверный bias sign/double subtraction, learned bias during motion;
- waveform delay/gain mutation; повторная запись first recovery;
- неполный/NaN/несовместимый report, отсутствующий expected scenario.

Mutation experiments делаются в временных копиях. У каждой ожидается конкретный invariant/failure;
compile error не засчитывается как обнаруженная численная регрессия. Набор выбирается по изменённому owner.

Три независимых verdict в отчёте:

1. Execution/evidence: complete / incomplete / incompatible / tool error.
2. Contracts: satisfied / new violation / known open / unverified.
3. Quality comparison: improved metrics / unchanged within resolution / regression / inconclusive.

Новый hard-invariant failure всегда FAIL. Известные OPEN имеют явный ID, owner и baseline;
новый сбой нельзя автоматически добавить в разрешённый список. Strict acceptance исправляемого
контракта требует PASS, даже если локальный обзорный запуск допускает известные OPEN.
Документированное отсутствие target данных не превращается в PASS target timing.

## 10. Вывод: компактно в консоль, подробно в артефакты

Предлагаемый формат, НЕ фактический новый вывод уже реализованных tools:

```text
SCOPE owners=ahrs,mag focused=yes evidence=synthetic
EXECUTION complete; CONTRACTS new_fail=0 known_open=5
COMPARISON regression=0 improved_metrics=3 inconclusive=1
COVERAGE required=... executed=... unverified=... unobservable=...
REPORT summary.json; DETAILS comparison.json; REPRO failures/<case>.json
```

При сбое печатать ID требования/сценария, seed, sample index, timestamp/epoch,
проверяемую метрику, before/after, допуск, exit code и путь к подробностям.
Успешные samples не печатать. Прогресс длительной compilation/run — через существующий reporter.

Машинный отчёт должен хранить:

- source identity + content hash, harness/generator/schema version, config/build identity;
- input/truth hashes, units/frames, cadence, clock conventions, seed/initial state;
- owner и requirement ID, expected/observed states, phase windows, fault schedule;
- actual/expected samples, валидность/observability denominator, причины исключений;
- все метрики с единицами, tolerances и причинами budget, before/after/delta/uncertainty;
- отдельные execution/contracts/quality/performance verdict и явные coverage gaps;
- точную команду воспроизведения как данные, без её автоматического исполнения из чужого отчёта;
- минимальный fail fixture и ограниченный контекст samples/state transitions вокруг сбоя.

Полные traces и plots — по запросу, при регрессии или для заявлений о динамической точности;
не выводить их целиком агенту и не генерировать постоянно без нужды.
Хранение артефактов и retention должны иметь явный бюджет; не чистить пользовательские данные автоматически.

## 11. Порядок реализации и защита от технического долга

1. DEV-05a: исправить два подтверждённых дефекта стенда (first recovery и проверка реального gyro step),
   добавить минимальные регрессионные tests. Не смешивать с большим расширением scenario framework.
2. Следующее дополнение: ядро M01–M06/M10–M11 для 0029/0030 — границы, composition, phases,
   multi-episode recovery, observable/unobservable labels и требуемые поля отчёта.
3. Следующее расширение: noise ensembles и frequency/amplitude response, связанные calibration/bias
   cases на существующих owners. Выбирается перед соответствующими changes/accuracy claims.
4. Интеграционные сценарии через реальные FIFO/recovery/calibration/output owners, затем dense recorded
   inputs с достаточным sample coverage. Нынешний decimated CAL log не объявлять 960 Hz replay.
5. DEV-06: реальные timing/stack/heap, device smoke, capture и fault recovery. DEV-07: CI/release/Server.

Эта карта — контракт развития, а не требование реализовать весь будущий 0031–0038 в DEV-05a.
Новые sensor captures нужны только при недостаточности существующих данных для конкретного claim.
Ground-truth accuracy на устройстве требует подходящего внешнего эталона и оценки его timing/error;
сравнение двух фильтров между собой не является абсолютной истиной.

Не создавать второй runner/event bus/estimator. Сценарии и метрики имеют один owner/schema;
registry/counts/config definitions по возможности не дублировать в нескольких ручных таблицах кода.
Размеры/длительности tests bounded. Хеширование/JSON/plots — host only.
При изменении harness старый и новый firmware прогоняются с одним новым harness;
нельзя автоматически сравнивать несопоставимые старые baseline files.

Уровни запуска:

- Каждый local change: owner-focused deterministic invariants и воспроизведение исправляемого сбоя.
- Fusion/math change: соответствующая scenario matrix, existing replay, sanitized host tests,
  A/B; target verification согласно L2 при затронутом hot path.
- Расширенный кандидат: multi-seed/sweeps/long virtual runs и relevant integration transitions.
- Release/default: требуемые полные gates, profiles, target/Server/rollback; synth PASS их не заменяет.

Сборка общих объектов остаётся общей внутри свежего существующего run. Не повторять полный check_all
для каждой метрики и не отключать проверки ради токенов. Расширение runtime тестирования вводится
по реальному риску; длинные virtual runs не должны означать sleep на часы.

## 12. Критерий готовности расширенного математического стенда

1. Все применимые M01–M13 имеют owner/сценарии и честный статус покрытия; future owners помечены future.
2. Базовые mathematical identities и generators проверены независимым способом.
3. Все выбранные hard invariants имеют positive/negative/boundary tests.
4. Есть переходы, возврат после отказа и критические сочетания, а не только статические значения.
5. Ошибки измеряются вместе с publication continuity и observability.
6. Известные нарушения не скрыты, новый сбой даёт FAIL, неполные данные не дают PASS.
7. Characteristic mutations действительно обнаруживаются исполнением проверок.
8. A/B воспроизводим и не позволяет компенсировать регрессию чужим улучшением.
9. Windows/WSL проверены отдельно; real target/absolute accuracy отмечаются только по фактическим данным.
10. Новая инфраструктура не меняет firmware behavior и не дублирует существующих owners.

## Источники и ограничения исследования

Основные источники — перечисленные project files, tests и current production owners.
Новые полные test suites в ходе составления этой спецификации не запускались.

Для внешней сверки использованы первичные документы авторов AHRS:

- https://ahrs.readthedocs.io/en/latest/metrics.html — различение SO(3) и vector metrics.
- https://ahrs.readthedocs.io/en/latest/filters/fqa.html — отсутствие accel-опоры для yaw,
  ограничения gravity inference при linear acceleration и сингулярности отдельных представлений.

Их NED convention не переносится в наш +Z-up код. Формулы и signs проверяются по проекту.
Никакая внешняя библиотека не добавляется в firmware и не назначается эталонным estimator.

# выборочные прогоны и логи

Выбор проверок, ограниченные сроки выполнения и сохранение доказательств.

`python` в примерах ниже обозначает выбранный проектный интерпретатор.
На Windows используйте `& $TrackerPython ...` после блока [session](session.md);
в WSL — его Linux Python. Не полагайтесь на случайный `python` из PATH и не
переносите PowerShell-переменные в Bash или новое окно без инициализации.

## Обычный рабочий цикл

Узнать доступные ID, без компиляции и запуска тестов:

```sh
python tools/check_all.py --list-checks
python tools/run_standalone_tests.py --list-tests
```

Выбрать Python policies/validators:

```sh
python tools/check_all.py --check test_selective_runners --check test_check_all_aggregation --check validate_documentation
```

Выбрать native tests:

```sh
python tools/run_standalone_tests.py --test test_core_math_ahrs --test test_sensor_calibration
```

Флаги выбора повторяются, дубликаты выполняются один раз. Неизвестный ID
завершает команду ошибкой до компиляции. Автоматического угадывания тестов по
изменённым файлам нет: используйте [карту владельцев и тестов](test_map.md).

Native selection компилирует PROJECT_SOURCES один раз на прогон, затем выбранные
тесты и необходимые им extra link objects. Остальные compile-only/Arduino-stub
проверки пропускаются **только при явном --test**. Это partial coverage;
одиночный тест всё ещё требует сборки общих объектов.

Явный компилятор задаётся через `--cxx` или `CXX`. Ошибочный явный путь теперь
завершается ошибкой, без подмены другим компилятором из PATH. При отсутствии
явного выбора сохраняется поиск g++/clang++/c++.

```sh
python tools/run_standalone_tests.py --test test_core_math_ahrs --sanitizer address-undefined
```

Sanitizer preflight остаётся обязательным, в том числе с `--build-only`.
Нет fallback к другому sanitizer/none. Отдельный предварительный doctor/probe
перед каждым native run не нужен: runner сам проверяет выбранный runtime.

## Полные gates

Без selectors прежняя матрица остаётся:

```sh
python tools/check_all.py --host-only
python tools/check_all.py --release
```

Focused check_all несовместим с --host-only, --release, --skip-*, --clean,
--require-pio и --pio-env. Partial PASS не означает full/release PASS.
Release сохраняет preflight, три native режима, пять target profiles, clean
build и manifests. Выборочный прогон не закрывает отсутствующие release prerequisites.

Зависимые replay-команды остаются в полном tool-smoke потоке. Они не превращены
в независимые selectors, которые могли бы использовать старые output files.
Для прямого запуска replay следуйте существующей документации соответствующего
инструмента; численные эталоны описаны в [fusion validation](fusion_validation.md).

## Где результаты

Общий runner печатает RUN перед командой и редкие WAIT во время
ожидания. Native runner подавляет сообщения для каждого быстрого object compile.
Интервал уведомлений обычно 30–40 секунд; это признак ожидания процесса, а не
процент готовности.

По умолчанию консоль краткая. Оба потока каждой child-команды пишутся напрямую
в raw log без накопления всего вывода в RAM:

```text
build/gate_runs/check-all-<unique>/summary.json
build/gate_runs/check-all-<unique>/0001.log
build/gate_runs/native-<unique>/summary.json
```

В конце при ошибке печатаются exit code, ограниченные фрагменты начала/конца
лога и путь к нему. Причина из середины длинного лога может оказаться только
в полном файле. Предупреждения успешного компилятора также сохраняются в raw log.
Строка ERR из negative test сама по себе не становится failure: остаются прежние
exit-code и semantic gates.

`--verbose` печатает полный вывод после каждой команды; это не live relay.
Для live-наблюдения можно читать текущий raw log. В полном check_all native
runner создаёт вложенный отдельный отчёт; его путь виден в родительском логе.
Verbose передаётся в native runner.

JSON содержит scope, selection, состояние running/completed/interrupted,
returncode, команды, cwd, таймауты, elapsed time, ссылки на raw logs и notes.
Source metadata — HEAD/dirty на старте, без fingerprint содержимого рабочего
дерева. Две Git-команды ограничены пятью секундами каждая; их недоступность
фиксируется как unknown, а не как доказательство clean source. Это не замена
release identity.

JSON обновляется atomic replace перед/после команды. При резком завершении
может остаться running; такой отчёт нельзя принять за завершённый PASS.
Это не гарантия fsync при потере питания. Переменные окружения, credentials и
remote URLs не выгружаются, но пути и собственный вывод запускаемой программы
остаются видны. Перед внешней отправкой логи можно просмотреть.

Для существующего PIO size policy полный текст читается после завершения
команды: классификация size-only failure не делается по короткому excerpt.
Ненулевой exit отдельной команды не всегда равен результату всего gate;
например, прежняя policy может классифицировать конкретный Debug overflow.
Итоговый returncode/warnings сохраняют решение существующего gate.

## GitHub CI и сохранённые firmware bundles

[Workflow](../../.github/workflows/ci.yml) запускается на pull request, push в
`main`, `master`, `codex/dev-preparation` и вручную через workflow_dispatch
(доступность ручного запуска зависит от наличия workflow в default branch).
Никакой записи в репозиторий, публикации release, секретов или доступа к трекеру.
Первый push с этим файлом — реальная проверка GitHub orchestration; локальные
unit tests/actionlint её не заменяют. Не ставьте новые jobs обязательными для
merge до разбора первого запуска. В PR проверяется checkout merge commit;
его SHA, а не SHA исходной ветки, попадает в build identity и имя артефакта.

Независимые jobs: полный Linux host gate; отдельные полные native ASan/UBSan и
LSan с обязательными capability probes; пять обычных target environments плюс
USB_DIAG. Matrix fail-fast выключен, параллелизм каждой matrix ограничен двумя.
Новый запуск того же ref отменяет устаревший. Бюджеты jobs — 60 минут для host/
sanitizer, 40 для firmware; target compile дополнительно ограничен 1800 секундами.
Штатные внутренние deadlines runners не ослаблены. `shell: bash` обеспечивает
pipefail: `tee` не превращает ошибку теста или сборки в успешный шаг.

Host runner — Ubuntu 24.04, Python 3.12, системный GCC. Это не доказательство
Windows/MSYS ABI и не воспроизведение известных Windows стековых чисел. Все
действующие политики запускаются без allow-failure и без увеличения лимитов.
Даже при failed host job firmware jobs сохраняют свои результаты независимо;
успешная сборка из красного workflow не считается принятой прошивкой.

Каждая target job стартует без бинарного cache; PlatformIO Core 6.1.18 и target
pins не меняются. SOURCE_DATE_EPOCH берётся из проверяемого commit. Actions
закреплены полными SHA, однако Python patch version, образ runner и транзитивные
pip-зависимости не полностью заморожены: версии пакетов записываются в логи.
Это прослеживаемая сборка, не обещание побайтовой воспроизводимости.

Logs/JSON и готовые firmware ZIP загружаются с `if: always()` на 14 дней; имя
включает SHA, run_id и run_attempt. Если runner потерян/жёстко остановлен, сохранение
не гарантируется. В firmware artifact нет неподтверждённых частичных BIN: архив
создаётся только после успешной сборки и проверки всех четырёх файлов. Логи
неудачной сборки сохраняются без выдачи ей PASS. Важные комплекты скачайте до
истечения retention; наличие artifact не является результатом release gate.

`release_manifest.py --bundle OUTPUT.zip` дополняет существующие `--environment`,
`--artifact`, `--output`, `--toolchain` и `--require-clean`. Нужны ровно
firmware.bin, firmware.elf, partitions.bin, bootloader.bin и чистый полный SHA.
Проверяются исходные hash/size, поток копирования и источник после копирования.
Изменившийся файл прерывает упаковку без готового ZIP; существующий ZIP не заменяется.
ZIP содержит новый manifest с repository-relative путями
`build/firmware-bundles/<identity>/<environment>/...`, не ссылками на `.pio`.
Поэтому последующая компиляция не перезаписывает скачанный комплект.

Восстановление: скачайте artifact, из него выберите firmware ZIP и распакуйте
с сохранением путей в корень отдельного checkout соответствующего commit.
Не перезаписывайте уже существующий комплект того же identity/environment:
сначала сравните SHA-256 или используйте отдельный checkout. Перед аппаратной
работой укажите извлечённый `manifest.json` в `device_workflow.py flash-plan`.
Этот шаг проверяет embedded identity/профиль и partition layout без открытия USB;
сам ZIP доказывает соответствие байтов manifest, а не аппаратную работоспособность.
DEBUG_LINKCHECK остаётся внутренним build artifact, не uploadable profile.
Дальнейшая запись требует явного разрешения по [device guide](device_smoke.md).

CI не вызывает `--release`: реальные LOGVER3/golden, Server, boot evidence и
rollback ещё имеют открытые пункты. Release-gate не ослаблен и по-прежнему требует
их применимых входов. Следующий этап приёмки CI — первый GitHub run и проверка
скачанного bundle через flash-plan; это не требует повторной прошивки.

## Повторить только упавшие проверки

```sh
python tools/check_all.py --failed-from build/gate_runs/check-all-EXAMPLE/summary.json
```

Подставьте реальный путь к завершённому failed **focused check_all** отчёту.
Повторяются только failed IDs, через текущий Python и текущий реестр scripts.
Команды из JSON не исполняются. Новый прогон создаёт новый отчёт; старый не
меняется, новый PASS тоже остаётся partial.

Success/running/interrupted, full/release/native, чужая schema, неизвестные IDs,
лишние arguments и несогласованные records отвергаются. Для native ошибки
выберите --test явно: ошибка общего объекта не равна ошибке одного теста.

## Safe reuse и хранение

Общие объекты используются несколькими тестами внутри одного свежего native
run. Между прогонами переиспользуется только список failed IDs. Нет кэша PASS
или повторного исполнения старых binaries. Private build dirs, native lock,
удаление output перед компиляцией и проверки созданного файла сохранены.
Кэш объектов без полного ключа compiler/headers/flags/sanitizer/system libraries
не добавлен: экономия не должна давать ложный PASS.

Отчёты находятся вне native build dir и переживают --clean. Разные вызовы
имеют разные каталоги. Автоматического удаления failed/active evidence нет:
архивируйте нужные завершённые reports, затем вручную удаляйте ненужные.
Не очищайте активный прогон. Полные check_all/PIO builds в одном checkout
выполняйте последовательно: часть старых outputs и target cache общие.
Для параллельной работы используйте отдельные checkouts.

## Проверка инструментов

```sh
python tools/check_all.py --check test_selective_runners --check test_run_standalone_tests --check test_check_all_aggregation --check test_sanitizer_infrastructure --check validate_documentation
```

Полный gate нужен по соответствующему merge/release контракту, а не после
каждого изменения документации. Текущие ограничения
находятся в [статусе](../status.md).

## Неполные результаты

Перед финальной строкой `report=` runner проверяет чтением записанный JSON.
В полном check_all успешный exit code native-процесса дополнительно требует
завершённого успешного native JSON и завершённых успешных child-команд.
Если доказательство отсутствует или неполно, gate фиксирует FAIL с `evidence_error`,
сохраняя настоящий `process_returncode=0`. Исходный child JSON не переписывается.
Это проверка завершения отчёта; полнота сценариев остаётся обязанностью runner
и отдельных проверок coverage. Power-loss durability/fsync здесь не заявляются.

Если stack check не нашёл функцию, общий helper сохраняет доступные `.su` и
`inventory.json` в `build/gate_runs/stack-evidence-<unique>/` до удаления временной
папки компилятора. Максимум 64 файла / 8 MiB, усечение явно маркируется.
Отсутствие записи всё равно FAIL; повторов компиляции и повышения лимитов нет.
При обычном превышении бюджета или PASS дополнительный архив не создаётся.
Эти файлы нужны только при повторении сбоя; сам факт пропуска не доказывает
ошибку компилятора, парсера или среды без соответствующих данных.

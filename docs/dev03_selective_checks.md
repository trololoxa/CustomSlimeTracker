# DEV-03: выборочные прогоны и логи

Применять после DEV-02 на базе `Tracker_after_DEV2.zip`. Это продолжение ветки
подготовки проекта; packet 23 и весь код прошивки остаются без изменений.

## Обычный рабочий цикл

Узнать доступные ID, без компиляции и запуска тестов:

```sh
python tools/check_all.py --list-checks
python tools/run_standalone_tests.py --list-tests
```

Выбрать Python policies/validators:

```sh
python tools/check_all.py --check test_dev03_runners --check test_check_all_aggregation_policy --check validate_documentation
```

Выбрать native tests:

```sh
python tools/run_standalone_tests.py --test test_core_math_ahrs --test test_sensor_calibration
```

Флаги выбора повторяются, дубликаты выполняются один раз. Неизвестный ID
завершает команду ошибкой до компиляции. Автоматического угадывания тестов по
изменённым файлам нет: owners/test map относится к DEV-04.

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
build и manifests. DEV-03 не закрывает отсутствующие release prerequisites.

Зависимые replay-команды остаются в полном tool-smoke потоке. Они не превращены
в независимые selectors, которые могли бы использовать старые output files.
Для прямого запуска replay следуйте существующей документации соответствующего
инструмента; новые сценарии и численные эталоны относятся к DEV-05.

## Где результаты

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
python tools/check_all.py --check test_dev03_runners --check test_run_standalone_tests_policy --check test_check_all_aggregation_policy --check test_dev01_test_infrastructure --check validate_documentation
```

Полный gate нужен по соответствующему merge/release контракту, а не после
каждого изменения документации. Фактическая проверка этого патча и ограничения
описаны в `dev03_runners_report.md`.

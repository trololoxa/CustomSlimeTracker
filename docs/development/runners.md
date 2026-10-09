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

[Workflow](../../.github/workflows/ci.yml) `Build and Test` проверяет pull requests
и push в `main`/`master`. Для рабочей ветки откройте draft PR: следующие push
обновляют его проверки, отдельного дублирующего branch-push run нет. В PR
проверяется merge commit; его SHA отличается от SHA исходной ветки.
Нет секретов, записи в репозиторий, публикации release или доступа к устройству.

| Запуск | Проверки | Артефакты успешного запуска |
| --- | --- | --- |
| Только Markdown в docs/ или корне | documentation + maintenance | Нет |
| Обычный PR/push с другими файлами | Полный Linux host + Production build | Нет |
| workflow_dispatch | Host, ASan/UBSan, LSan, шесть target profiles | Три отчёта; один выбранный firmware bundle или ни одного |
| Явный push тега `ci/full/**` | Та же полная матрица | Три отчёта и ProductionDiag bundle |

Diff берётся целиком, без ограничения API первыми 300 файлами; rename проверяет
старый и новый путь. Неизвестный base, пустой diff и изменения вне узкого
Markdown-набора выбирают host + Production. Полный запрос всегда сильнее diff.
Нет пропуска тестов по тексту commit message, автоматических повторов или
continue-on-error. Сохранены полные suites, probes, build profiles и бюджеты.
Финальный `CI result` требует success всех выбранных jobs; ожидаемые skipped
учитываются только для выбранного режима. После приёмки workflow именно этот
стабильный check можно сделать обязательным для PR; он не означает release PASS.

Для ручного запуска `workflow_dispatch` файл должен присутствовать в default
branch. До слияния dev-ветки используйте одноразовый тег на проверяемом commit:

```powershell
$TrackerCiTag = 'ci/full/dev07-' + (Get-Date -Format 'yyyyMMdd-HHmmss')
git tag $TrackerCiTag HEAD
if ($LASTEXITCODE -ne 0) { throw 'Cannot create CI tag' }
git push origin "refs/tags/$TrackerCiTag"
if ($LASTEXITCODE -ne 0) { throw 'Cannot push CI tag' }
```

Это явный запрос полной проверки и ProductionDiag bundle, не release tag.
Не используйте `git push --tags`. После появления workflow в main можно выбрать
Actions → Build and Test → Run workflow → нужную ветку и `firmware_bundle`.
Официальное ограничение: [manual workflows](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/manually-run-a-workflow).

Отчёты неудачных проверок/сборок сохраняются на 7 дней. Полный запуск сохраняет
также успешные host/sanitizer отчёты; firmware — только выбранный комплект с
логами его сборки. DEBUG_LINKCHECK проверяется в полной матрице, но его бинарник
не предлагается для прошивки. На обычном успешном PR нет загружаемых артефактов;
штатный Actions log остаётся доступен. Имена показывают назначение, профиль,
12-значный SHA и attempt; полный SHA находится в манифесте и метаданных run.
Важные комплекты и отчёты скачайте до истечения retention. Отмена старого run
того же ref не удаляет его уже сохранённые артефакты.

Ubuntu 24.04/Python 3.12/системный GCC не воспроизводят Windows/MSYS ABI.
Все политики сохраняют прежние лимиты. Target jobs независимы от host FAIL:
наличие firmware artifact из красного workflow не означает принятую сборку.
Matrix fail-fast выключен, параллелизм ограничен двумя; host/sanitizer timeout
60 минут, firmware 40 минут, отдельный compile 1800 секунд. Bash pipefail
сохраняет ошибку команды перед tee. Checkout не сохраняет credentials; actions
закреплены SHA. Сборки без бинарного cache, PlatformIO 6.1.18 и прежние target pins;
SOURCE_DATE_EPOCH — время commit. Версии pip/target packages записываются в логи.
Python patch version, runner image и transitive dependencies не полностью
заморожены: это прослеживаемость, не побайтовая воспроизводимость.

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
их применимых входов. Приёмка изменённого CI требует hosted run и проверки
скачанного bundle через flash-plan; это не требует повторной прошивки.
Порядок завершения DEV описан в [testing](testing.md#development-branch-acceptance).

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

# один запуск Windows + WSL

Owner — `tools/dev_verify.py`: orchestration, проверка
одинаковых входных файлов и сбор доказательств. Компиляцией, запуском тестов,
таймаутами процессов, sanitizer probes и форматами метрик по-прежнему владеют
существующие doctor, check_all, native runner, gate_reporting и algorithm_accuracy.
Прошивка, compiler flags, packet mode, partitions и физические пороги не меняются.

## Зачем

Инструмент выполняет синхронизацию, проверяет одинаковые входы и собирает отчёты
без одноразовых скриптов. Его запускают для явно запрошенного полного прогона;
обычные изменения проверяют выборочно по [карте тестов](test_map.md).

## Запуск из PowerShell

Сначала обычный блок окружения из [сессионного руководства](session.md). Требуются уже
установленные Windows Python/GCC/PlatformIO и WSL Ubuntu с Linux GCC, git,
Python и `.venv-dev-linux`. Среда WSL не использует Windows venv. Sudo, установка,
изменение PATH всей системы, flash и стирание NVS здесь не выполняются.
PlatformIO использует свою обычную систему зависимостей: при отсутствии пакетов
сборка может потребовать их загрузку; это не offline/reproducible SDK gate.

Просмотр плана ничего не синхронизирует и не запускает:

```powershell
& $TrackerPython tools/dev_verify.py plan
```

Перед `run` просмотрите и закоммитьте выбранные изменения обычным способом.
Windows и Linux checkout должны быть чистыми; скрипт ничего не коммитит и не
удаляет ради достижения чистоты. Linux origin должен указывать именно на этот
Windows checkout, как при ранее выполненном локальном clone.

```powershell
& $TrackerPython tools/dev_verify.py run --cxx $TrackerCxx --pio-bin $TrackerPio
$TrackerVerifyExit = $LASTEXITCODE
```

По умолчанию: дистрибутив `Ubuntu`, пользователь `ubuntu`, Linux checkout
`/home/ubuntu/src/SlimeTracker`. При отличающейся установке укажите
`--wsl-distro`, `--wsl-user`, `--wsl-root`. Пути с пробелами передаются отдельными
аргументами без shell-конкатенации. Внутренние `_sync`/`_linux` вручную не запускают.

## Что выполняется

1. Сохраняются SHA и хеши всех tracked файлов, а также ignored входов в
   `src/include/lib/test/tests/tools/partitions` (кроме Python bytecode).
2. В WSL проверяются чистота, origin, отсутствие divergence и локальных файлов,
   которые fast-forward затёр бы. Затем `fetch` точного SHA и `merge --ff-only`.
   Reset, stash, создание нового clone и смена origin не выполняются.
3. Windows: doctor `firmware`, затем существующий
   `check_all.py --clean --require-pio`. Проверяются полнота registry, native
   selection/executables, пять replay-команд и пять firmware profiles.
4. WSL: doctor `sanitized`, полный ASan/UBSan и затем полный LSan. Windows FAIL
   не отменяет Linux; обычный ASan/UBSan FAIL не отменяет LSan. Непригодная среда
   или несовпадение исходников блокирует соответствующий этап, без fallback.
5. Из успешных native отчётов экспортируются fusion-сценарии. Экспорт — упаковка
   метрик, он не превращает OPEN 0029/0030 в закрытые требования.
6. После проверок снова сверяются входы. Копируются summaries, raw logs,
   doctor/probe reports, replay JSON, build manifests, accuracy JSON и provenance.
   Linux-копирование проверяется по SHA-256 и полному перечню файлов.
7. Печатаются компактные результаты и пути к итоговым JSON/Markdown/ZIP.

Текст известных форматов допускает различие LF/CRLF; бинарные fixtures сравниваются
побайтово. Symlinks/submodules блокируются до явного определения их контракта.
Инвентаризация не сертифицирует внешний SDK, произвольные external includes или
вредоносную подмену исходников/отчётов. Во время прогона не редактируйте проект.

## Ограничения времени, параллелизм, ошибки

Этапы последовательны. `--stage-timeout-s` по умолчанию 7200 секунд для Windows
full gate и каждого Linux native suite. Внутри Linux: build 600 s / executable
180 s; отдельные sanitizer probes сохраняют свои короткие ограничения. Внешний
WSL wrapper имеет бюджет двух suites плюс 1200 s на подготовку/копирование.
Никаких автоматических повторов полного прогона. RUN/WAIT обеспечивают существующие
runners; подробные строки subprocess остаются в логах.

Отдельный workflow lock не допускает два таких запуска в одном checkout. Native
runner сохраняет собственный lock. Не запускайте одновременно ручные full gates
или сборки в тех же каталогах. При Ctrl+C/таймауте Windows wrapper нет гарантии,
что Linux child уже завершился: он ограничен своим deadline и держит Linux lock.
Проверьте процессы WSL перед повтором. Скрипт не выключает WSL и не убивает чужие
процессы. Диск/сеть/разрешения могут прервать упаковку; тогда частичный каталог
сохраняет доступные логи, но полная переносимость доказательств не заявляется.

Exit 0 — все запрошенные локальные этапы завершены успешно и их полнота доказана.
Ненулевой exit — FAIL/блокировка/неполный результат; 130 обозначает обработанное
прерывание. Успех части этапов не маскирует общий FAIL. Отчёт с неизвестной или
неполной native selection, отсутствующим summary/doctor, изменившимися входами,
ошибкой переноса либо неоднозначным summary не даёт общий PASS.

Три известных stack-policy FAIL (`test_magnetic_hotpath_budget`, `test_yaw_microsoft_abi_budget`,
`test_imu_hotpath_budget`) остаются в общем
gate. По решению владельца они отложены до firmware-серии. Workflow не увеличивает
лимиты, не исключает тесты и не объявляет эти ошибки исправленными.

## Результат и документация

Путь печатается как `archive=.../build/gate_runs/dev-verify-<id>.zip`. Внутри:
`verification.json`, краткий `verification.md`, inventories, исходные логи и
копии с provenance, включая `linux-origin` с проверенным transfer manifest.
Большие бинарники/объекты и SDK в архив не включаются. Отчёт — доказательства,
не набор команд для последующего исполнения. Повторный запуск всегда новый.

Агент читает сначала `verification.json`, затем raw log только нужного FAIL.
Он обновляет только подтверждённые pending, указывая SHA и scope; исторический
FAIL не стирает. Скрипт сам не редактирует документацию: hardware, target timing,
Server, release, absolute accuracy и будущие математические сценарии не закрывает.
Доступ к USB/отладка остаются DEV-06; CI/Server — DEV-07.

## Проверка orchestration

```powershell
& $TrackerPython tools/check_all.py --check test_cross_platform_verification --check test_check_all_aggregation --check test_gate_reporting --check validate_documentation
```

Local acceptance: Windows/WSL end-to-end remains pending until source synchronization succeeds. Host fixtures do not prove the real cross-platform workflow.

Приёмка эффективности агента — отдельная задача [roadmap](../roadmap.md).

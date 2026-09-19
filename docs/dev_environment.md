# DEV-02: окружение разработки Tracker

Применять **после DEV-01**. Этот этап готовит инструменты и диагностику среды.
Он не меняет ESP32 toolchain, firmware, ODR, fusion, thresholds или partitions.
Doctor ничего не устанавливает, не прошивает и не подключает отладчик.

## Что означает готовность

| Проверка | Команда | Что доказывает |
| --- | --- | --- |
| Host | `python tools/doctor.py --profile host` | Python >=3.10, Git, выбранный C++ компилятор, C++20 compile/link/run, finite arithmetic |
| Sanitized | `python tools/doctor.py --profile sanitized --cxx /usr/bin/g++` | Host плюс реальные clean/fault probes ASan+UBSan и отдельного LSan из DEV-01 |
| Firmware tools | `python tools/doctor.py --profile firmware --pio-bin /path/to/pio` | Host, версия Core, установленные exact package versions из platformio.ini, RISC-V ABI и компиляция объекта |
| Optional debugger | `python tools/doctor.py --require-tool gdb` | Найденный GDB запускается; подключение и совместимость с ESP32 НЕ доказаны |

PASS относится только к выбранным проверкам. Host PASS не означает sanitizer,
firmware build или target smoke PASS. Отсутствие необязательных инструментов —
SKIP; неисправная необязательная программа — WARN; требуемая — FAIL. Dirty Git
даёт WARN и не мешает локальной разработке. Release требует отдельной проверки.

Каждый внешний процесс ограничен `--timeout-s` (по умолчанию 30 секунд **на
команду**, не на весь doctor). Повторов нет. stdout/stderr/exit code/команда и
source/config identity остаются в `build/doctor/*.json`; консоль краткая.
Новый запуск не затирает предыдущий отчёт. Пути и версии могут содержать имя
пользователя; перед отправкой наружу отчёт можно просмотреть. Переменные среды,
Git remotes, Wi-Fi/NVS credentials и pip configuration не выгружаются.

`--cxx`/`CXX` и `--pio-bin`/`PIO` принимают один исполняемый файл, включая путь
с пробелами. Shell aliases и строки вида `ccache g++` не поддерживаются.
Явно ошибочный путь не заменяется программой из PATH. Для необязательного PIO
в host-режиме отсутствие допустимо; проверку установки требует firmware-режим.

## Windows: native host и PlatformIO

Используйте Windows Python >=3.10 (рекомендуемая линия для этой инструкции —
3.12), Git и MSYS2 UCRT64 GCC. Точные версии фиксирует doctor; это не обещание
одинаковых stack sizes между Windows ABI и Linux ABI. Python и компилятор могут
жить в разных установках, но runtime DLL выбранного GCC должны быть доступны.
Пример PowerShell из корня проекта (адаптировать пути к установленным программам):

```powershell
py -3.12 -m venv .venv-dev
$TrackerPython = (Resolve-Path '.venv-dev\Scripts\python.exe').Path
$TrackerCxx = 'C:\msys64\ucrt64\bin\g++.exe'
$env:PATH = 'C:\msys64\ucrt64\bin;' + $env:PATH
& $TrackerPython tools/doctor.py --profile host --cxx $TrackerCxx
```

Активация venv и изменение PowerShell execution policy не нужны. Не используйте
один venv одновременно из Windows и WSL. Не подменяйте C++ compiler молча,
если native gate сообщает превышение stack budget: сохраняйте ABI/compiler
identity и используйте существующую cross-host policy.

## Linux / WSL2: native и sanitizers

Нужны Python >=3.10 с venv, Git, GCC с C++20 и соответствующие GCC sanitizer
runtime libraries. Установка системных пакетов зависит от дистрибутива и
выполняется отдельно владельцем машины. Clang — допустимый явно выбранный
альтернативный компилятор, если его runtime проходит те же probes.

```sh
python3 -m venv .venv-dev-linux
.venv-dev-linux/bin/python tools/doctor.py --profile host --cxx /usr/bin/g++
.venv-dev-linux/bin/python tools/doctor.py --profile sanitized --cxx /usr/bin/g++
```

Для WSL храните Linux checkout и build cache в файловой системе Linux, отдельно
от Windows checkout/cache. Не запускайте параллельно native gates в одном build
каталоге. Windows инструменты сборки/прошивки могут оставаться в Windows;
санитайзеры — в WSL/Linux. USB/JTAG bridge и доступ к COM — отдельный DEV-06.

LSan — отдельный режим, не побочный эффект ASan. Если runtime не может читать
`/proc` или работать под debugger/ptrace/sandbox, результат остаётся FAIL.
Не отключайте leak detection и не повышайте разрешения автоматически ради PASS.
Повторите требуемый gate в подходящем обычном Linux/WSL окружении. Наличие
symbolizer проверяется как инструмент; качество stack trace ещё надо проверить.
TSan/MSan не добавлены как фиктивные обязательные gates: для них нужна отдельная
область проверки и инструментированная среда. ESP32 firmware не запускается
под host ASan/LSan.

## PlatformIO: bootstrap и воспроизводимая установка

`tools/requirements-platformio.txt` фиксирует **host Core 6.1.18**, а не меняет
ESP32 platform/toolchain. Это bootstrap pin, **не полный dependency lock**.
Зависимости сначала разрешаются один раз под конкретные OS/architecture/Python,
затем сохраняются все wheels и lock их байтов. Не смешивайте wheelhouses разных
сред. Используйте новый пустой каталог для нового решения зависимостей.

Windows PowerShell (после создания venv выше):

```powershell
$TrackerWheels = 'build\dev-env\windows-py312'
# Каталог должен быть новым; если он уже существует, выберите другое имя.
New-Item -ItemType Directory -Path $TrackerWheels -ErrorAction Stop | Out-Null
& $TrackerPython -m pip download --only-binary=:all: --dest $TrackerWheels -r tools/requirements-platformio.txt
# Продолжать только если предыдущая команда завершилась с кодом 0.
& $TrackerPython tools/lock_dev_wheels.py --wheelhouse $TrackerWheels --output "$TrackerWheels\requirements.lock"
& $TrackerPython -m pip install --no-index --require-hashes --find-links $TrackerWheels -r "$TrackerWheels\requirements.lock"
& $TrackerPython -m pip check
$TrackerPio = (Resolve-Path '.venv-dev\Scripts\pio.exe').Path
& $TrackerPio pkg install -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG
& $TrackerPython tools/doctor.py --profile firmware --cxx $TrackerCxx --pio-bin $TrackerPio
```

**PowerShell не останавливается автоматически на ненулевом коде native
программы. Выполняйте по одной команде и проверяйте `$LASTEXITCODE`; при ошибке
остановитесь.** Нельзя считать частично скачанный wheelhouse полным lock.

Linux/WSL, shell с остановкой при ошибке:

```sh
set -e
TrackerPython=.venv-dev-linux/bin/python
TrackerWheels=build/dev-env/linux-py312
mkdir -p build/dev-env
mkdir "$TrackerWheels"
"$TrackerPython" -m pip download --only-binary=:all: --dest "$TrackerWheels" -r tools/requirements-platformio.txt
"$TrackerPython" tools/lock_dev_wheels.py --wheelhouse "$TrackerWheels" --output "$TrackerWheels/requirements.lock"
"$TrackerPython" -m pip install --no-index --require-hashes --find-links "$TrackerWheels" -r "$TrackerWheels/requirements.lock"
"$TrackerPython" -m pip check
.venv-dev-linux/bin/pio pkg install -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG
"$TrackerPython" tools/doctor.py --profile firmware --cxx /usr/bin/g++ --pio-bin .venv-dev-linux/bin/pio
```

`lock_dev_wheels.py` не выполняет download/install/resolve, не читает pip config,
не перезаписывает существующий lock. Он проверяет bootstrap version, метаданные
и отсутствие нескольких версий одного пакета. SHA-256 защищает последующее
использование сохранённых байтов; он не доказывает доверенность первоначального
источника. Dependency closure проверяется **pip install + pip check**. Если
пакет требует source build, команда download завершается ошибкой; молчаливого
перехода к неповторяемой сборке sdist нет.

Для доказательства повторной установки создайте второй чистый venv
`.venv-dev-verify`, установите тот же lock с `--no-index --require-hashes`,
выполните `pip check` и `pio --version`. Один такой прогон на платформу достаточен;
не повторяйте всю native suite после каждой установки. Сохраните wheelhouse,
lock, версию Python/pip, OS/architecture и doctor JSON вне временного build перед
его очисткой. Не сохраняйте credentials или весь пользовательский PlatformIO
home. Для полной offline ESP-сборки нужны также уже разрешённые PlatformIO
platform/packages и их отдельный проверенный snapshot: pip wheels их не включают.
DEV-02 не заявляет bit-for-bit reproducibility системного GCC, Python или ESP
пакетов и не подменяет её одним файлом requirements.

## Проверка целевого окружения и её границы

Doctor читает exact pins из `env:BOARD_LOLIN_C3_MINI` в `platformio.ini`:
проверяет канонические `platforms/<name>/platform.json` и
`packages/<name>/package.json` в core directory, полученном через
`pio system info --json-output`. Версии должны совпадать буквально. Эта проверка
не доказывает registry provenance или целостность всех файлов пакета. Другой
layout, version-suffixed directories или несогласованные manifest versions
дают FAIL, а не предположение об успехе. Сохраните JSON и вывод
`pio pkg list -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG`, если doctor не распознаёт
установку. Не редактируйте firmware pins или package manifests ради зелёного gate.

После doctor отдельно выполните настоящую сборку:

```sh
pio run -e BOARD_LOLIN_C3_MINI_PRODUCTION_DIAG
```

Используйте свой явный путь к pio, как выше. Doctor компилирует только маленький
RISC-V объект, без Arduino SDK/linker scripts; это не замена firmware build.
Не запускайте `upload`, `erase`, reset, sleep или calibration автоматически.
Подготовка безопасного device identify/flash/target smoke/JTAG будет в DEV-06.

`gdb`, `openocd`, `clangd`, `llvm-symbolizer` пока только инвентаризируются.
Host GDB не обязательно поддерживает ESP32-C3; конкретный Espressif debugger,
transport и аппаратная сессия требуют отдельной проверки. Compilation database
и корректная конфигурация навигации относятся к DEV-04.

## Проверка самого DEV-02

```sh
python tools/test_dev02_environment.py
python tools/validate_documentation.py
```

Self-tests включены в существующий `check_all.py`. Сам environment doctor туда
не добавлен обязательным default: отсутствие optional target/debug tools не
должно выключать рабочий host gate. Выбранный `--profile sanitized` или
`--profile firmware` сам возвращает ненулевой код при недостающем требовании.
Для дальнейшей работы достаточно запускать нужный профиль после изменения
окружения. Не требуется запускать все профили перед каждым редактированием.

Официальные справки: [PlatformIO installation](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html),
[system info](https://docs.platformio.org/en/latest/core/userguide/system/cmd_info.html),
[pkg list](https://docs.platformio.org/en/latest/core/userguide/pkg/cmd_list.html),
[Core 6.1.18](https://pypi.org/project/platformio/6.1.18/).

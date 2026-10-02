# возобновление среды и одинаковый исходный код

Эти команды используют уже установленную среду владельца проекта. На другом
компьютере измените пути. Ничего не устанавливается при каждом открытии сессии;
первичная установка и wheel locks описаны в [руководстве по установке](environment.md).
Каждая строка — отдельная команда для указанной консоли. При ошибке остановитесь.
PowerShell сам по себе не прерывает последующие команды по ненулевому exit code.

Работа с устройством (выбор USB, smoke, явная прошивка и crash evidence):
[единый device workflow](device_smoke.md#bounded-device-workflow). Команды подключения
не входят в обычный host gate.

## PowerShell после перезагрузки

Conda activation не требуется для этих явных путей. Сначала задайте и проверьте:

```powershell
Set-Location 'H:\Programming\VRC\CustomSlime\SlimeTracker'
$TrackerCondaEnv = Join-Path $env:USERPROFILE '.conda\envs\tracker-dev'
$TrackerPython = Join-Path $TrackerCondaEnv 'python.exe'
$TrackerPio = Join-Path $TrackerCondaEnv 'Scripts\pio.exe'
$TrackerCxx = 'C:\msys64\ucrt64\bin\g++.exe'
$TrackerClangd = 'H:\TrackerTools\clangd-22.1.6\bin\clangd.exe'
$TrackerPython,$TrackerPio,$TrackerCxx | ForEach-Object { if (-not (Test-Path -LiteralPath $_ -PathType Leaf)) { throw "Missing tool: $_" } }
$env:PYTHONNOUSERSITE = '1'
$env:PLATFORMIO_CORE_DIR = 'H:\TrackerTools\platformio'
$env:CXX = $TrackerCxx
$env:PIO = $TrackerPio
$env:PATH = "$TrackerCondaEnv;$TrackerCondaEnv\Library\bin;$TrackerCondaEnv\Scripts;C:\msys64\ucrt64\bin;$env:PATH"
if (Test-Path -LiteralPath $TrackerClangd -PathType Leaf) { $env:PATH = "$(Split-Path -Parent $TrackerClangd);$env:PATH" } else { Write-Warning "Optional clangd not found: $TrackerClangd" }
& $TrackerPython --version
```

Переменные `$Tracker*` и изменения `$env:*` здесь относятся к этому PowerShell
и его дочерним процессам, не к будущим окнам. Ранее сохранённые Conda env vars
сработают при активации соответствующего окружения. Они не заменяют эти
PowerShell-переменные. Не используйте bare `python`: раньше PATH выбирал MSYS2
Python без pip. Не меняйте execution policy ради этих команд.

Если нужна команда управления Conda, используйте полный путь к `conda.exe`
(или `& $env:CONDA_EXE`, если переменная существует), обходя неисправную shell
обёртку. Не создавайте заново удалённый `H:\TrackerDevVerify`.

Проверка среды после изменения установки (не перед каждым тестом):

```powershell
& $TrackerPython tools/doctor.py --profile firmware --cxx $TrackerCxx --pio-bin $TrackerPio
```

Точечная проверка runner workflow:

```powershell
& $TrackerPython tools/check_all.py --check test_workflow_deadlines --check test_selective_runners --check test_run_standalone_tests --check test_check_all_aggregation --check validate_documentation
```

## clangd и отладчик в Windows

Основной блок выше добавляет установленный clangd в PATH текущей сессии: теперь
его видят doctor и дочерние процессы, без отдельной ручной команды. На другой
машине/после обновления ZIP измените `$TrackerClangd`. Отсутствие необязательного
clangd выдаёт предупреждение и не блокирует host-тесты. Версия 22.1.6 здесь —
проверенный локальный путь, а не новый обязательный toolchain pin.

VS Code хранит путь отдельно: `clangd.path` должен указывать на тот же EXE.
Текущий shell не меняет окружение уже запущенного VS Code/Codex. Для native
навигации в `clangd.arguments` используются:

```text
--compile-commands-dir=H:/Programming/VRC/CustomSlime/SlimeTracker/build/clangd-native
--query-driver=C:/msys64/ucrt64/bin/g++.exe
```

Microsoft C/C++ можно оставить для отладки, отключив его конкурирующий
IntelliSense в Workspace (`C_Cpp.intelliSenseEngine = disabled`). Это не меняет
компилятор проекта. Native database содержит host stubs; ESP32 indexing не
считается проверенным. После изменения compiler/includes/flags экспортируйте
базу из нового успешного native report, как описано в [карте тестов](test_map.md).

Когда проверяется именно доступность этих инструментов, а не каждый запуск:

```powershell
& $TrackerPython tools/doctor.py --profile host --cxx $TrackerCxx --require-tool clangd --require-tool gdb
```

`doctor` проверяет запуск `--version`, а не breakpoint/attach. GDB из UCRT64 —
host debugger; target GDB/OpenOCD и доступ к плате проверяются отдельно в DEV-06.
Текущие ограничения и приёмка находятся в [статусе](../status.md).
При перенаправлении Python-логов с Unicode используйте
`$env:PYTHONIOENCODING = 'utf-8'`, чтобы CP1251 не мешала выводу диагностики.

## WSL после перезагрузки

В PowerShell:

```powershell
wsl -d Ubuntu --cd '~'
```

Далее в Linux-консоли:

```sh
cd ~/src/SlimeTracker
.venv-dev-linux/bin/python --version
```

Ubuntu уже перенесена в `H:\WSL\Ubuntu`; повторный export/move не нужен.
Windows PlatformIO остаётся в Windows; native sanitizers запускаются в Linux.
Не используйте Windows venv или compiler в Linux. Отсутствие clangd/OpenOCD/PIO
в Linux не отменяет прошедшие GCC sanitizer probes. Установка редактора и
аппаратная отладка — отдельные задачи.

## Обновить Linux-копию после Windows-патча

Для полного прогона используйте [единый workflow](cross_platform_verification.md).
Ниже ручной путь для отдельного обновления без полного gate.

1. Windows: просмотрите diff и закоммитьте выбранный патч обычным способом;
   не делайте слепой `git add .` с неизвестными файлами.
2. В Windows запустите:

```powershell
& $TrackerPython tools/dev_source_check.py
```

Скопируйте полный SHA из PASS. Dirty/unknown остаётся FAIL: helper ничего
не коммитит, не стирает и не синхронизирует. Проверка нужна при переносе
результатов между checkout, а не как запрет работать над незакоммиченным патчем.

3. В Linux сначала проверьте свои изменения:

```sh
git status --short
```

При непустом выводе сохраните/разберите их перед обновлением. Если чисто:

```sh
git remote get-url origin
```

Ожидается `/mnt/h/Programming/VRC/CustomSlime/SlimeTracker`. При другом origin
не меняйте его автоматически; выясните, откуда должна поступать версия.

```sh
git fetch origin
```

Подставьте скопированный SHA вместо `COMMIT_FROM_WINDOWS`:

```sh
git merge --ff-only COMMIT_FROM_WINDOWS
.venv-dev-linux/bin/python tools/dev_source_check.py --expect-commit COMMIT_FROM_WINDOWS
```

Не продолжайте при ошибке. Diverged history не исправляется `reset --hard`;
нужно разобраться с ветками. Совпадение SHA/clean не проверяет ignored fixtures,
внешние библиотеки или содержимое submodules: wheel locks, doctor и release
manifest имеют свои области доказательств. Linux cache/venv не копируются назад.

## Проверки и короткая передача результатов

```sh
.venv-dev-linux/bin/python tools/check_all.py --check test_workflow_deadlines --check validate_documentation
.venv-dev-linux/bin/python tools/run_standalone_tests.py --cxx /usr/bin/g++ --test test_core_math_ahrs
```

Sanitizer suite запускайте при соответствующем изменении/приёмке, не каждый раз
после открытия shell. Пример только для недостающего autonomy-теста:

```sh
.venv-dev-linux/bin/python tools/run_standalone_tests.py --cxx /usr/bin/g++ --sanitizer address-undefined --test test_calibration_autonomy --build-timeout-s 600 --test-timeout-s 180
```

Для продолжения работы достаточно передать: цель, commit/dirty, изменённые файлы,
что уже прошло (режим/selection), путь к summary/raw log, оставшуюся проблему.
Не пересылайте весь исторический чат или успешные compile logs. Failed evidence
сохраняйте: исходный FAIL не превращается задним числом в full PASS.

Очистка: не удаляйте весь `build` — там wheelhouse и отчёты. Native runner сам
ограничивает число своих build directories; отчёты удаляются только осознанно
после сохранения нужных результатов. Резервную копию Ubuntu можно удалить позднее
после проверки нужных данных и решения владельца; инструменты проверки этого не делают.

## Применение Git-патча

Укажите фактический путь к полученному файлу патча. Сначала выполните проверку:

```powershell
$TrackerPatch = 'C:\Users\nikol\Downloads\maintenance_after_DEV05b.patch'
git apply --check $TrackerPatch
```

Только при exit code 0 выполните `git apply $TrackerPatch`. Если check не прошёл,
файлы не изменены: сначала проверьте базовую версию и локальный diff. Если доказано,
что различается только whitespace контекста CRLF/LF, допустимы check и apply с
`--ignore-space-change`. Не используйте его для обхода смыслового конфликта,
`--reject` или изменение глобального `core.autocrlf` ради применения патча.

## Если Windows и WSL видят разную чистоту

Это блокировка синхронизации, а не разрешение пропустить проверку. Сравните
`git status --short` с обеих сторон и происхождение настроек. В PowerShell:

```powershell
git config --show-origin --get core.autocrlf
git config --show-origin --get core.filemode
wsl -d Ubuntu --exec git -C /mnt/h/Programming/VRC/CustomSlime/SlimeTracker config --show-origin --get core.autocrlf
wsl -d Ubuntu --exec git -C /mnt/h/Programming/VRC/CustomSlime/SlimeTracker config --show-origin --get core.filemode
wsl -d Ubuntu --exec git -C /mnt/h/Programming/VRC/CustomSlime/SlimeTracker status --short
```

`--exec` избегает промежуточного разбора команд Bash. Отсутствующая настройка
даёт exit 1 и пустой вывод. Это диагностика: не выполняйте автоматический reset,
массовую нормализацию EOL или изменение global Git settings.

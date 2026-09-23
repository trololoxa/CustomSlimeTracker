# DEV-04: возобновление среды и одинаковый исходный код

Эти команды используют уже установленную среду владельца проекта. На другом
компьютере измените пути. Ничего не устанавливается при каждом открытии сессии;
первичная установка и wheel locks описаны в [DEV-02](dev_environment.md).
Каждая строка — отдельная команда для указанной консоли. При ошибке остановитесь.
PowerShell сам по себе не прерывает последующие команды по ненулевому exit code.

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

Обычная точечная проверка DEV-04:

```powershell
& $TrackerPython tools/check_all.py --check test_dev04_workflow --check test_dev03_runners --check test_run_standalone_tests_policy --check test_check_all_aggregation_policy --check validate_documentation
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
базу из нового успешного native report, как описано в [карте тестов](dev_test_map.md).

Когда проверяется именно доступность этих инструментов, а не каждый запуск:

```powershell
& $TrackerPython tools/doctor.py --profile host --cxx $TrackerCxx --require-tool clangd --require-tool gdb
```

`doctor` проверяет запуск `--version`, а не breakpoint/attach. GDB из UCRT64 —
host debugger; target GDB/OpenOCD и доступ к плате проверяются отдельно в DEV-06.
После обновления MSYS2 пользователь подтвердил GCC 16.2.0, GDB 17.2 и AHRS
smoke 1/1. Старый full gate относится к GCC 14.2.0 и не переносится на новую
версию автоматически; полный прогон не нужен для этой правки документации.

Обновление 2026-09-23: [полная локальная проверка после DEV-05a](dev05_local_verification_2026-09-23.md)
выполнена на GCC 16.2.0: native 54/54, replay 5/5 и firmware profiles 5/5 прошли,
но полный Windows gate завершился FAIL по трём stack-budget policies. WSL GCC
13.3.0: отдельные полные ASan/UBSan и LSan — 54/54 каждая. Для запуска из Codex
потребовался штатный доступ вне sandbox к WSL, старым native-артефактам и
PlatformIO lock; ACL и глобальные настройки не менялись. При перенаправлении
Python-логов в этой проверке использовали `$env:PYTHONIOENCODING = 'utf-8'`,
чтобы избежать обнаруженного UnicodeEncodeError при выводе ошибок в CP1251.
Исходный FAIL сохранён отдельно; это не рекомендация ослаблять тестовые gates.

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
.venv-dev-linux/bin/python tools/check_all.py --check test_dev04_workflow --check validate_documentation
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
после проверки нужных данных и решения владельца; DEV-04 этого не делает.

## Применение Git-патча при смешанных CRLF/LF

Для DEV-04 подтверждён случай: текст `.gitignore` совпадает с присланным архивом,
но последние пять строк имеют LF вместо CRLF. `git apply --check` ничего не
изменяет; при такой ошибке база ещё не установлена. После сверки текста можно
применить DEV-04, разрешив различия whitespace только при сопоставлении контекста:

```powershell
$TrackerPatch = 'C:\Users\nikol\Downloads\Tracker_DEV-04_after_DEV-03a\DEV-04_after_DEV-03a.patch'
git apply --check --ignore-space-change $TrackerPatch
git apply --ignore-space-change $TrackerPatch
```

Каждая строка отдельно; apply выполняется только после успешного check.
На приложенном `.gitignore` проверено сохранение всех исходных байтов плюс
добавление нового ignore rule. Не используйте этот флаг для обхода смыслового
конфликта, `--reject` или глобальное изменение `core.autocrlf` ради одного патча.
После DEV-04 накладывается DEV-04a; буква означает дополнение, не замену базы.

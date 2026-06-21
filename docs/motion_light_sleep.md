# Motion-triggered light sleep

## Purpose

`TRACKER_ENABLE_MOTION_LIGHT_SLEEP` adds an opt-in idle state for an ESP32-C3
tracker whose LSM6DSV `INT1` is connected to a normal GPIO such as GPIO10. It
is **light sleep**, not deep sleep: ESP32 RAM survives and the CPU resumes at
the statement after `esp_light_sleep_start()`.

When SlimeVR Server has been continuously absent for the configured timeout,
the firmware:

1. detaches the FIFO ISR from the shared `INT1` pin;
2. stops SlimeVR UDP, resets the Wi-Fi state machine and calls `WiFi.mode(WIFI_OFF)`;
3. suspends QMC6309 and stops the LSM6DSV sensor hub without modifying saved
   magnetometer settings;
4. disables FIFO, gyro, tap and DRDY routing;
5. configures the high-pass LSM6DSV wake-up detector on `INT1` using the
   low-power accelerometer at 60 Hz;
6. turns the software-controlled status LED off and ends the serial console;
7. enters ESP32 light sleep with high-level GPIO wake on `PIN_LSM_INT1`.

A qualifying acceleration event raises latched `INT1`, wakes ESP32, clears the
wake source, restores Serial, performs the normal LSM/FIFO initialization,
rebuilds the magnetometer/tap runtimes and restarts Wi-Fi/SlimeVR discovery.
The tracker starts a new server-absence timeout after every wake attempt.

## Enable

The feature is off in every build profile. Set this in a private build config
or in `src/build_config/feature_flags.hpp`:

```cpp
#define TRACKER_ENABLE_MOTION_LIGHT_SLEEP 1
```

Relevant `runtime_tuning.hpp` overrides are:

```cpp
#define TRACKER_MOTION_LIGHT_SLEEP_SERVER_ABSENCE_MS 60000UL
#define TRACKER_MOTION_LIGHT_SLEEP_WAKE_THRESHOLD 12u
#define TRACKER_MOTION_LIGHT_SLEEP_WAKE_DURATION 0u
```

The driver selects the LSM6DSV 62.5 mg/code wake-up-threshold resolution and
`WAKE_THRESHOLD=12` is therefore approximately 750 mg.
Increase it if normal table vibration causes false wakes. `WAKE_DURATION` is
0..3; begin at 0 and raise it only after physical tests.

## Manual sleep command

With the feature enabled, enter this in the serial console:

```text
sleep
```

The command prints `motion_light_sleep=queued` and returns first; the app
starts the transition only after the CLI poll has completed. This prevents a
use-after-reconfigure of the serial parser. The command is unavailable and
absent from `help` when `TRACKER_ENABLE_MOTION_LIGHT_SLEEP=0`.

## Hardware and behaviour limits

- The feature requires `PIN_LSM_INT1` to be directly wired to LSM6DSV `INT1`.
  GPIO10 is valid for **light sleep** wake on ESP32-C3.
- It is not deep sleep. GPIO10 cannot wake ESP32-C3 from deep sleep; that
  requires an RTC-domain GPIO (GPIO0..GPIO5) or external reset hardware.
- Wi-Fi is off while sleeping. The tracker cannot receive UDP, discovery or
  broadcast wake commands until physical motion wakes it.
- The red board power LED and regulator quiescent current are hardware loads;
  this firmware can only turn off the GPIO-controlled status LED.
- The timeout is paused while command-driven static/runtime tests are active,
  so test runs cannot unexpectedly put the tracker to sleep.
- The optional `sleep` serial command is compiled only when
  `TRACKER_ENABLE_MOTION_LIGHT_SLEEP=1`. It queues the same light-sleep path
  and never enters sleep from inside the serial parser callback.

## Bench test

1. Build with `TRACKER_ENABLE_MOTION_LIGHT_SLEEP=1` and open Serial Monitor.
2. Start the tracker without a reachable SlimeVR Server, or issue `sleep` to
   test the exact same path immediately.
3. Wait at least 60 seconds for automatic sleep. The last pre-sleep line is
   `# motion_light_sleep=enter`; Serial then intentionally stops.
4. Move the tracker firmly. It should reconnect Serial and print
   `# motion_light_sleep=wake_resume_ok` before returning to Wi-Fi discovery.
5. Repeat with the tracker stationary on a table. It must not wake without an
   acceleration event. Tune the threshold only from measured behaviour.

Run host tests with:

```text
python3 tools/run_standalone_tests.py --clean
```

The native suite covers the no-server timeout/reset semantics and the LSM6DSV
motion-wake register sequence. It cannot validate ESP32 light-sleep current or
real GPIO wake; those require a flashed board.

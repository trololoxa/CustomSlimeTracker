# Wi-Fi power and thermal optimization status

This document covers wave 5: Wi-Fi power/thermal optimization. These changes
must not alter IMU ODR, LSM6DSV performance mode, FIFO processing, AHRS math or
local quaternion quality.

## Accepted defaults

The committed product defaults remain conservative:

```text
TRACKER_WIFI_POWER_SAVE_MODE = TRACKER_WIFI_POWER_SAVE_NONE
TRACKER_WIFI_TX_POWER        = WIFI_POWER_8_5dBm
```

Why these remain the defaults:

- `WIFI_PS_NONE` was clean on the home-router baseline.
- `MIN_MODEM` and `MAX_MODEM` were stable in the tested environment, but did not
  demonstrate a clear steady-state temperature reduction.
- 5 dBm TX produced rotation send failures in one long run.
- 2 dBm TX was stable in one home-router run, but did not demonstrate a clear
  thermal benefit and may not be robust on weaker AP/RSSI setups.

Therefore the committed matrix stays small and stable:

```text
BOARD_LOLIN_C3_MINI_DEBUG
BOARD_LOLIN_C3_MINI_PRODUCTION
BOARD_LOLIN_C3_MINI_SLIM
```

## Manual A/B overrides

Future Wi-Fi experiments should be local one-off builds, not committed project
environments. Use a private PlatformIO config or a one-off command with the
needed `-D...` values.

Power-save values:

```text
TRACKER_WIFI_POWER_SAVE_NONE      // baseline, WiFi.setSleep(false), WIFI_PS_NONE
TRACKER_WIFI_POWER_SAVE_MIN_MODEM // first modem-sleep candidate
TRACKER_WIFI_POWER_SAVE_MAX_MODEM // aggressive modem-sleep candidate
```

TX power can be overridden either with the Arduino enum token or numeric
quarter-dBm:

```text
-DTRACKER_WIFI_TX_POWER=WIFI_POWER_8_5dBm
-DTRACKER_WIFI_TX_POWER_QUARTER_DBM=20  // 5.00 dBm
-DTRACKER_WIFI_TX_POWER_QUARTER_DBM=8   // 2.00 dBm
```

Do not adopt a Wi-Fi power candidate unless a stable AP/router test shows zero
persistent transport failures and an actual steady-state temperature or current
benefit.

## Runtime fields to compare

`test runtime` prints:

```text
wifi_power_save_start
wifi_power_save_end
wifi_tx_power_start_quarter_dbm
wifi_tx_power_start_dbm
wifi_tx_power_end_quarter_dbm
wifi_tx_power_end_dbm
wifi_rssi_end_dbm
slime_send_failures_delta
slime_rotation_send_failures_delta
slime_control_send_failures_delta
slime_telemetry_send_failures_delta
slime_discovery_send_failures_delta
slime_udp_reopen_requests_delta
slime_rotation_rate_hz_observed
temp_start_c
temp_end_c
temp_delta_c
temp_slope_c_per_min
temp_recent_slope_c_per_min
```

A candidate is rejected if it introduces Wi-Fi disconnects, UDP send failures,
unknown packets, server silence resets, FIFO errors, tracking recovery events or
visible SlimeVR stutter.

## Interpreting temperature

Compare steady-state temperature, not only `temp_delta_c`. Back-to-back runs can
start at different sensor temperatures; a smaller delta from a hotter start does
not prove a cooler configuration.

Under unchanged ambient/case/airflow conditions, reduced heat generation should
reduce the final equilibrium temperature:

```text
T_final = T_ambient + P_heat * R_thermal
```

If different Wi-Fi settings all converge to about the same final temperature,
then either the tested Wi-Fi setting is not a dominant heat source, or Debug/test
runtime overhead is dominating the measurement.

## Battery/work counters during Wi-Fi A/B

`battery_work_count` is a loop-classification counter, not the source of truth
for ADC sampling frequency. Use the runtime-test battery section as the source of
truth:

```text
battery_samples_delta
battery_read_failures_delta
battery_last_sample_age_ms_end
```

For the default Debug interval of 10 seconds, a 300 second test should normally
produce about 30 battery samples. A 1200 second test should normally produce
about 120 samples, plus startup samples.

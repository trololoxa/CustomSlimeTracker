# Battery ADC filtering

The RC1 battery monitor uses a high-value divider:

```text
BAT+ -> 180 kOhm -> GPIO4 / ADC1_CH4 -> 180 kOhm -> GND
```

The divider current is low, but the ADC source impedance is high and there is no
hardware capacitor on the ADC node. A single ADC conversion is therefore expected
to be noisy and may include settling error. The firmware intentionally reads the
battery rarely and spends a small burst of ADC conversions only when a new battery
sample is due.

Default policy:

```text
Debug sample interval:       10 s
Production sample interval:  30 s
Slim:                        battery runtime disabled
ADC burst size:              64 conversions
Discarded settle reads:      4 conversions
Burst estimator:             sorted trimmed mean
Runtime smoothing:           EMA alpha 0.12
Impossible step rejection:   TRACKER_BATTERY_MAX_FILTER_STEP_V
```

The first conversions in the burst are discarded. The remaining valid readings
are sorted; for larger bursts the firmware discards roughly 12.5% from each tail
and averages the stable center. The resulting millivolt value is converted
through the divider ratio and then fed into the runtime EMA.

Useful CLI/runtime checks:

```text
battery status
test runtime 300
```

`battery status` prints the sample interval, oversample count, discard count, EMA
alpha, raw ADC millivolts, filtered voltage and battery percentage. Runtime tests
print `battery_samples_delta`; this should match the configured sparse interval
(for example about 30 samples in a 300 second Debug test with a 10 second
interval).

If the reported voltage is consistently wrong but stable, calibrate the scale and
offset rather than reducing filtering quality:

```cpp
-DTRACKER_BATTERY_VOLTAGE_SCALE=...
-DTRACKER_BATTERY_VOLTAGE_OFFSET=...
```

For a 1:1 180 kOhm / 180 kOhm divider, a 4.20 V battery should read about
2.10 V at GPIO4 before the firmware multiplies it back by 2.

#pragma once

#include <Arduino.h>
#include <cstdint>

#include "connection/lsm6dsv_driver.hpp"
#include "runtime/static_test_types.hpp"
#include "sensor/imu_quality.hpp"

namespace tracker {

// Dedicated gyro temperature calibration capture used by guided setup.
// It records the same temperature-bin data needed by the production temp fit,
// but it is intentionally separate from StaticTestRunner so `setup calibration`
// does not depend on the developer-only `test static` command path.
class GyroTempCalibrationCapture {
public:
    void start(uint32_t nowMs, uint32_t maxDurationMs);
    void stop(uint32_t nowMs);
    void reset();

    bool active() const;
    bool completed() const;
    uint32_t elapsedMs(uint32_t nowMs) const;

    void updateSample(const Lsm6dsv::Sample& calibrated,
                      const ImuQualityResult& quality,
                      uint32_t nowMs);

    const StaticRuntimeTest& capture() const;
    StaticRuntimeTest& capture();

    uint32_t usableTempBins(uint32_t minSamplesPerBin = 512) const;
    float tempRangeC() const;

private:
    StaticRuntimeTest capture_;
    bool completed_ = false;
};

} // namespace tracker

#pragma once

#include <Arduino.h>

#include "runtime/static_test_types.hpp"
#include "runtime/runtime_bias_types.hpp"
#include "sensor/calibration.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"

namespace tracker {

struct GyroTempStaticFitDeps {
    const StaticRuntimeTest* lastCompletedStaticTest = nullptr;
    bool lastCompletedStaticTestValid = false;
    GyroTempCompensator* gyroTempComp = nullptr;
    ImuCalibration* imuCal = nullptr;
    RuntimeGyroBiasEstimator* runtimeBias = nullptr;
    TrackerConfig* config = nullptr;
    TrackerConfigStore* configStore = nullptr;
    void (*onModelApplied)(bool biasValidityChanged, void* user) = nullptr;
    void* onModelAppliedUser = nullptr;
};

enum class GyroTempStaticFitMode : uint8_t {
    PreviewOnly,
    ApplyRam,
    ApplyAndSave,
};

bool fitGyroTempFromCompletedStaticTestEx(GyroTempStaticFitDeps& deps,
                                          const StaticRuntimeTest& test,
                                          GyroTempStaticFitMode mode,
                                          Stream& out);

bool fitGyroTempFromCompletedStaticTest(GyroTempStaticFitDeps& deps,
                                          const StaticRuntimeTest& test,
                                          bool persist,
                                          Stream& out);

bool fitGyroTempFromLastStatic(GyroTempStaticFitDeps& deps, bool persist, Stream& out);

} // namespace tracker

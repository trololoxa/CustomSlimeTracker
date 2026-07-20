#pragma once

#include <cstdint>

#include "runtime/output_runtime.hpp"
#include "sensor/ahrs_6dof.hpp"

namespace tracker {

using OrientationDependentResetFn = void (*)(const char* reason,
                                              uint64_t timestampUs,
                                              bool rebaseAhrsTimebase);

struct OrientationRuntimeResetDeps {
    Ahrs6Dof* ahrs = nullptr;
    PreparedOutputRuntime* preparedOutput = nullptr;
    OrientationDependentResetFn resetDependentState = nullptr;
};

// Deliberate configuration/calibration reset. Unlike FIFO recovery, this
// invalidates the quaternion itself because the calibration or coordinate
// frame may have changed. The next valid accel sample reacquires roll/pitch.
bool resetOrientationRuntime(const OrientationRuntimeResetDeps& deps,
                             const char* reason,
                             uint64_t timestampUs);

} // namespace tracker

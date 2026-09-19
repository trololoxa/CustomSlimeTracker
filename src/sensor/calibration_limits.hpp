#pragma once

#include "core/math.hpp"

namespace tracker::calibration_limits {

// Shared physical admission constants used by both calibration producers and
// persistent semantic validation. Keeping one owner prevents the store from
// accepting a model the corresponding solver would never produce.
inline constexpr float ACCEL_MAX_ABS_BIAS_G = 0.30f;
inline constexpr float ACCEL_MIN_SCALE = 0.70f;
inline constexpr float ACCEL_MAX_SCALE = 1.30f;
inline constexpr float GYRO_STARTUP_MAX_NORM_RAD_S = 3.0f * MATH_DEG_TO_RAD;
inline constexpr float MAG_MIN_AXIS_RADIUS = 20.0f;
inline constexpr float MAG_MAX_AXIS_RATIO = 6.0f;

} // namespace tracker::calibration_limits

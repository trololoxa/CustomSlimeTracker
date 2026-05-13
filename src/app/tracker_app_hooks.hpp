#pragma once

// App hooks: dependency builders and callback glue that connect command/runtime
// modules to the hardware and runtime contexts. This is the composition edge of
// the firmware; domain logic should live in runtime/, serial/, config/ or sensor/.
//
// The implementation is split into small include-only sections because these
// hooks intentionally bind together global hardware/runtime singletons. Keeping
// the split at the composition edge improves readability without adding runtime
// indirection or changing ownership.

#include <Arduino.h>

#include "app/tracker_hardware_context.hpp"
#include "app/tracker_runtime_context.hpp"
#include "runtime/runtime_gyro_bias_controller.hpp"
#include "runtime/machine_log_runtime.hpp"
#include "runtime/imu_sample_pipeline.hpp"
#include "runtime/mag_status_reporter.hpp"
#include "runtime/runtime_status_reporter.hpp"
#include "runtime/gyro_temp_static_fit.hpp"
#include "app/tracker_command_wiring.hpp"
#include "app/tracker_bootstrap.hpp"

using namespace tracker;

#include "app/hooks/tracker_app_common_hooks.hpp"
#include "app/hooks/tracker_app_mag_hooks.hpp"
#include "app/hooks/tracker_app_command_hooks.hpp"
#include "app/hooks/tracker_app_runtime_hooks.hpp"

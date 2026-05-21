#pragma once

// Compatibility umbrella for project-wide compile-time settings.
//
// New code should include the specific header from src/build_config/ when it
// only needs one category. This file remains the stable include used by the
// existing firmware while the optimization/profile split is rolled out.

#include <cstdint>
#include <cstddef>

#include "build_config/build_profiles.hpp"
#include "build_config/feature_flags.hpp"
#include "build_config/runtime_tuning.hpp"
#include "build_config/network_tuning.hpp"
#include "build_config/profile_contract.hpp"
#include "build_config/board_pins.hpp"
#include "build_config/tracking_tuning.hpp"
#include "build_config/legacy_defines.hpp"

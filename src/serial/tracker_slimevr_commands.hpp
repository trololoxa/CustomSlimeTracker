#pragma once

#include "serial/tracker_serial_context.hpp"

namespace tracker {

enum class TrackerSlimeVRRuntimeApplyMode : uint8_t {
    PreserveSession = 0,
    RestartSession = 1,
};

bool trackerSerialApplySlimeVRRuntimeConfig(
    TrackerSerialCommandContext& ctx,
    TrackerSlimeVRRuntimeApplyMode mode
);

bool trackerSerialDispatchSlimeVRCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

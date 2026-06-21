#pragma once

#include "defines.h"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

#if TRACKER_HAS_MOTION_LIGHT_SLEEP
void trackerSerialDispatchMotionLightSleepCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);
#endif

} // namespace tracker

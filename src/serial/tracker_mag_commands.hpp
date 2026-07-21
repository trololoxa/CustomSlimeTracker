#pragma once

#include "serial/tracker_serial_context.hpp"

namespace tracker {

bool rearmMagIfNeeded(TrackerSerialCommandContext& ctx);
void trackerSerialDispatchMagCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

#pragma once

#include "serial/tracker_serial_context.hpp"

namespace tracker {

void rearmMagIfNeeded(TrackerSerialCommandContext& ctx);
void trackerSerialDispatchMagCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

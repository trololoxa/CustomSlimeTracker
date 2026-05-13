#pragma once

#include "serial/tracker_serial_context.hpp"

namespace tracker {

void trackerSerialDispatchAhrsCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

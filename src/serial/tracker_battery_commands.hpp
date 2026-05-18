#pragma once

#include "serial/tracker_serial_context.hpp"

namespace tracker {

void trackerSerialDispatchBatteryCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

#pragma once

#include "serial/tracker_serial_context.hpp"

namespace tracker {

bool trackerSerialDispatchSlimeVRCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

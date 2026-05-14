#pragma once

#include <Arduino.h>

#include "serial/tracker_serial_context.hpp"

namespace tracker {

void trackerSerialPrintNetworkStatus(TrackerSerialCommandContext& ctx);
void trackerSerialPrintNetworkConfig(TrackerSerialCommandContext& ctx);
bool trackerSerialDispatchNetworkCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

#pragma once

#include <Arduino.h>

#include "serial/tracker_serial_context.hpp"

namespace tracker {

void trackerSerialPrintHelp(Stream& out, TrackerCommandOrigin origin);
void trackerSerialPrintStatus(TrackerSerialCommandContext& ctx);
void trackerSerialPrintSetupStatus(TrackerSerialCommandContext& ctx);
void trackerSerialPrintHealth(TrackerSerialCommandContext& ctx);
void trackerSerialFactoryReset(TrackerSerialCommandContext& ctx);
bool trackerSerialDispatchSystemCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

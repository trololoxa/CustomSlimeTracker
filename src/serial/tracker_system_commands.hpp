#pragma once

#include <Arduino.h>

#include "serial/tracker_serial_context.hpp"
#include "config/factory_reset_coordinator.hpp"

namespace tracker {

void trackerSerialPrintHelp(Stream& out, TrackerCommandOrigin origin);
void trackerSerialPrintStatus(TrackerSerialCommandContext& ctx);
void trackerSerialPrintSetupStatus(TrackerSerialCommandContext& ctx);
void trackerSerialPrintHealth(TrackerSerialCommandContext& ctx);
bool trackerSerialFactoryReset(TrackerSerialCommandContext& ctx, FactoryResetScope scope);
bool trackerSerialDispatchSystemCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

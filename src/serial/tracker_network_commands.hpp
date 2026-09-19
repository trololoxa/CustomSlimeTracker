#pragma once

#include <Arduino.h>

#include "serial/tracker_serial_context.hpp"

namespace tracker {

// A synchronous platform scan interrupts FIFO service. Queue the shared,
// verified sensor recovery transaction before returning to normal runtime.
void trackerRecoverSensorStreamAfterBlockingWifiScan(
    TrackerSerialCommandContext& ctx,
    const char* reason);
void trackerSerialPrintNetworkStatus(TrackerSerialCommandContext& ctx);
void trackerSerialPrintNetworkConfig(TrackerSerialCommandContext& ctx);
bool trackerSerialDispatchNetworkCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

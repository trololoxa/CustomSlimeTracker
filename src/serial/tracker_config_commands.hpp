#pragma once

#include <Arduino.h>

#include "config/tracker_config.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

void trackerSerialApplyConfigToRuntime(TrackerSerialCommandContext& ctx);
void trackerSerialCaptureRuntimeToConfig(TrackerSerialCommandContext& ctx, TrackerConfig& target);
void trackerSerialCaptureRuntimeToConfig(TrackerSerialCommandContext& ctx);
void trackerSerialPrintConfigNvsInfo(Stream& out, TrackerConfigStore& store);
void trackerSerialDispatchConfigCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

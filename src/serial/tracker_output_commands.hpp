#pragma once

#include "serial/tracker_serial_context.hpp"

namespace tracker {

const char* trackerSerialStreamModeName(TrackerStreamMode mode);
const char* trackerSerialLogModeName(TrackerLogMode mode);
bool trackerSerialParseStreamMode(const char* s, TrackerStreamMode& mode);
bool trackerSerialParseLogMode(const char* s, TrackerLogMode& mode);
void trackerSerialDispatchStreamCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);
void trackerSerialDispatchLogCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);
void trackerSerialDispatchOutputCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

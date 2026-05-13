#pragma once

#include <Arduino.h>

#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/imu_quality.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

const char* trackerSerialOdrName(Lsm6dsv::Odr odr);
bool trackerSerialParseRuntimeOdr(const char* token, Lsm6dsv::Odr& outOdr);
void trackerSerialPrintFifoStats(Stream& out, const Lsm6dsvFifoReader::DrainStats& fs);
void trackerSerialPrintQualityStats(Stream& out, const ImuQualityCounters& qc);
void trackerSerialPrintImuRuntimeStatus(TrackerSerialCommandContext& ctx, Stream& out);
void trackerSerialDispatchImuCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);
void trackerSerialDispatchFifoCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);
void trackerSerialDispatchQualityCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

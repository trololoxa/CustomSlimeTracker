#pragma once

#include <Arduino.h>
#include <cstdint>

#include "connection/lsm6dsv_driver.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

void trackerSerialPrintFifoTuning(Stream& out, const TrackerSerialCommandContext& ctx);

bool trackerSerialCommitSpiFrequency(TrackerSerialCommandContext& ctx,
                                     Stream& out,
                                     uint32_t hz,
                                     bool save);

bool trackerSerialCommitFifoWatermark(TrackerSerialCommandContext& ctx,
                                      Stream& out,
                                      uint8_t watermarkWords,
                                      bool save);

bool trackerSerialCommitFifoDrain(TrackerSerialCommandContext& ctx,
                                  Stream& out,
                                  uint16_t maxWordsPerDrain,
                                  uint8_t maxDrainRoundsPerEvent,
                                  bool save);

bool trackerSerialCommitImuRate(TrackerSerialCommandContext& ctx,
                                Stream& out,
                                Lsm6dsv::Odr odr,
                                bool save);

// Production/basic CLI subset. Full CLI keeps its richer fifo status/stats/reset
// surface, but both profiles share the same transactional tuning implementation.
bool trackerSerialDispatchBasicFifoCommand(TrackerSerialCommandContext& ctx,
                                           int argc,
                                           char** argv);

} // namespace tracker

#pragma once

#include "serial/tracker_serial_context.hpp"
#include "config/tracker_config_storage.hpp"

namespace tracker {

void trackerSerialDispatchSetupCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

// Shared by manual/setup/config-save calibration acceptance. A partial first-
// run model proves fresh finite sensor progress; a complete model additionally
// proves prepared quaternion/linear acceleration output. Transport is separate.
bool trackerSerialVerifyCalibrationCandidate(TrackerSerialCommandContext& ctx,
                                             const TrackerConfig& candidate, bool promptUser);
bool trackerSerialCommitCalibrationCandidate(TrackerSerialCommandContext& ctx,
                                             const TrackerConfig& candidate,
                                             TrackerCalibrationProvenance provenance,
                                             TrackerPreparedConfigCommit* prepared = nullptr,
                                             uint8_t workspaceMask = 0u);

} // namespace tracker

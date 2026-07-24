#pragma once

#include "serial/tracker_serial_context.hpp"

namespace tracker {

enum TrackerCalibrationWorkspaceMask : uint8_t {
    TRACKER_CAL_WORKSPACE_NONE = 0u,
    TRACKER_CAL_WORKSPACE_ACCEL = 1u << 0,
    TRACKER_CAL_WORKSPACE_GYRO_TEMP = 1u << 1,
    TRACKER_CAL_WORKSPACE_MAG = 1u << 2,
    TRACKER_CAL_WORKSPACE_ALL = TRACKER_CAL_WORKSPACE_ACCEL |
        TRACKER_CAL_WORKSPACE_GYRO_TEMP | TRACKER_CAL_WORKSPACE_MAG,
};

void trackerSerialResetCalibrationWorkspaces(
    TrackerSerialCommandContext& ctx,
    uint8_t mask = TRACKER_CAL_WORKSPACE_ALL
);

void trackerSerialDispatchCalibrationCommand(TrackerSerialCommandContext& ctx, int argc, char** argv);

} // namespace tracker

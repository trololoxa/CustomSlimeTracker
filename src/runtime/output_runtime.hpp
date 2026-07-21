#pragma once

#include <Arduino.h>
#include <cstdint>

#include "defines.h"
#include "connection/lsm6dsv_driver.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/imu_quality.hpp"
#include "config/tracker_config_runtime.hpp"
#include "runtime/tracker_runtime_types.hpp"
#include "serial/tracker_serial_stream.hpp"

namespace tracker {

void outputPrintU64Dec(Stream& out, uint64_t v);

class PreparedOutputRuntime {
public:
    bool enabled(const TrackerConfig& config) const;
    void reset();
    void update(const TrackerConfig& config,
                uint32_t runtimeSamples,
                uint64_t timestampUs,
                const Ahrs6Dof& ahrs,
                const ImuQualityResult& quality,
                const Vec3& accelDeviceG);
    bool copy(TrackerPreparedOutputSnapshot& out) const;

private:
    TrackerPreparedOutputSnapshot snapshot_;
    volatile uint32_t seqLock_ = 0;
    uint64_t lastPublishedTimestampUs_ = 0;
};

void emitSerialStreamIfNeeded(TrackerSerialStreamState& streamState,
                              Stream& out,
                              const Lsm6dsv::RawSample& raw,
                              const Lsm6dsv::Sample& scaled,
                              const Lsm6dsv::Sample& calibrated,
                              const Ahrs6Dof& ahrs,
                              const ImuQualityResult& quality,
                              uint32_t nowUs);

bool maybePrintBootHeartbeat(Stream& out,
                             const TrackerSerialStreamState& streamState,
                             bool staticTestActive,
                             uint32_t nowMs,
                             uint32_t& lastHeartbeatMs,
                             uint32_t runtimeSamples,
                             uint32_t fifoIntCount,
                             float latestTempC,
                             uint32_t magSamples);

} // namespace tracker

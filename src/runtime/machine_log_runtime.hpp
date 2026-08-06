#pragma once

#include <Arduino.h>
#include <cstdint>

#include "defines.h"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "network/wifi_manager.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/calibration.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"
#include "sensor/mag_heading.hpp"
#include "sensor/mag_field_reliability.hpp"
#include "sensor/mag_runtime.hpp"
#include "sensor/mag_yaw_correction.hpp"
#include "runtime/runtime_bias_types.hpp"
#include "runtime/tracker_runtime_types.hpp"
#include "config/tracker_config_runtime.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

enum class MachineLogDeferredRecordType : uint8_t {
    Imu,
    Mag,
    State,
    BiasUpdate,
    Network,
};

struct MachineLogDeferredImuRecord {
    uint64_t timestampUs;
    uint32_t dtUs;
    float quaternionW;
    float quaternionX;
    float quaternionY;
    float quaternionZ;
    uint32_t qualityFlags;
    uint32_t estimatedDroppedBefore;
    float confidence;
    char trackingState[20];
    float accelTrust;
    float accelNormG;
    float accelVarianceG2;
    float gyroTrust;
    float gyroDps;
    bool trackingRecovering;
    bool hardwareTimestamp;
    bool fallbackTimestamp;
    bool fifoOverrun;
    bool fifoFull;
    bool fifoUnknown;
    float calibratedAccelX;
    float calibratedAccelY;
    float calibratedAccelZ;
    float calibratedGyroX;
    float calibratedGyroY;
    float calibratedGyroZ;
    float temperatureC;
    bool emitBias;
    float currentBiasX;
    float currentBiasY;
    float currentBiasZ;
    char biasSource[12];
    float biasFitQuality;
    uint32_t biasFlags;
    bool runtimeBiasEnabled;
    uint32_t runtimeBiasUpdates;
};

struct MachineLogDeferredMagRecord {
    uint64_t timestampUs;
    uint64_t yawTimestampUs;
    uint32_t magSequence;
    uint32_t ageMs;
    uint32_t rejectFlagsForUse;
    uint32_t reliabilityFlags;
    uint32_t yawRejectFlags;
    uint32_t yawCooldownRemainingMs;
    uint32_t yawFieldStableMs;
    uint16_t rawFlags;
    uint8_t reliabilityState;
    uint8_t yawMode;
    bool headingValid;
    bool trustedForUse;
    bool reliabilityTrustedForYaw;
    bool yawValid;
    bool yawGateOpen;
    bool yawApplyAllowed;
    bool yawApplied;
    bool yawReacquirePending;
    bool yawReacquireActive;
    float rawX;
    float rawY;
    float rawZ;
    float calibratedX;
    float calibratedY;
    float calibratedZ;
    float bodyX;
    float bodyY;
    float bodyZ;
    float rawNorm;
    float calibratedNorm;
    float bodyNorm;
    float horizontalNorm;
    float headingYawDeg;
    float headingInnovationDeg;
    float dipDeg;
    float reliabilityNormError;
    float reliabilityDipErrorDeg;
    float reliabilityHeadingErrorDeg;
    float yawErrorDeg;
    float yawCorrectionStepDeg;
    float yawCombinedTrust;
    float yawMagneticHeadingRateDegS;
};

struct MachineLogDeferredStateRecord {
    uint64_t timestampUs;
    uint32_t flags;
    float confidence;
    char state[20];
    char reason[40];
};

struct MachineLogDeferredBiasUpdateRecord {
    uint64_t timestampUs;
    float temperatureC;
    float residualX;
    float residualY;
    float residualZ;
    float stdX;
    float stdY;
    float stdZ;
    float deltaX;
    float deltaY;
    float deltaZ;
    float trimX;
    float trimY;
    float trimZ;
    uint32_t flags;
};

struct MachineLogDeferredNetworkRecord {
    uint64_t timestampUs;
    uint32_t wifiDisconnects;
    uint32_t wifiConnectTimeouts;
    int32_t wifiRssiDbm;
    uint32_t rotationSent;
    uint32_t rotationSendDue;
    uint32_t rotationMissedDeadlines;
    uint32_t rotationLateEvents;
    uint32_t sendFailures;
    uint32_t rotationSendFailures;
    uint32_t controlSendFailures;
    uint32_t telemetrySendFailures;
    uint32_t discoverySendFailures;
    uint32_t txPressureFailures;
    uint32_t txOtherFailures;
    uint32_t udpRebindSuccesses;
    uint32_t udpRebindFailures;
    uint32_t udpFullReopenEscalations;
    uint32_t consecutiveSendFailures;
    int32_t lastUdpSendError;
    uint32_t lastSuccessfulMotionTxAgeMs;
    uint8_t slimeState;
    uint8_t txPressureState;
    uint8_t txRecoveryReason;
    bool wifiConnected;
    bool udpReady;
    bool serverFound;
};

union MachineLogDeferredPayload {
    MachineLogDeferredImuRecord imu;
    MachineLogDeferredMagRecord mag;
    MachineLogDeferredStateRecord state;
    MachineLogDeferredBiasUpdateRecord biasUpdate;
    MachineLogDeferredNetworkRecord network;

    MachineLogDeferredPayload() : imu{} {}
};

struct MachineLogDeferredRecord {
    MachineLogDeferredRecordType type = MachineLogDeferredRecordType::Imu;
    TrackerLogMode mode = TrackerLogMode::Off;
    uint32_t sequence = 0;
    uint32_t queuedAtUs = 0;
    MachineLogDeferredPayload payload;
};

static_assert(sizeof(MachineLogDeferredRecord) <= 224u,
              "Deferred LOGVER3 record exceeded the audited RAM budget");

struct MachineLogDeferredStatus {
    uint32_t queued = 0;
    uint32_t highWater = 0;
    uint32_t enqueued = 0;
    uint32_t serialized = 0;
    uint32_t serviceCalls = 0;
    uint32_t maxRecordAgeUs = 0;
};

class MachineLogDeferredRuntime {
public:
    void begin(TrackerSerialLogState* state,
               MachineLogCounters* counters,
               uint32_t* lastBiasEmitUs,
               uint32_t biasPeriodUs);
    void reset(bool countPendingAsDropped);

    bool enqueueImu(const Lsm6dsv::RawSample& raw,
                    const Lsm6dsv::Sample& calibrated,
                    const ImuQualityResult& quality,
                    const Ahrs6Dof& ahrs,
                    const char* trackingState,
                    bool trackingRecovering,
                    const GyroTempCompensator& gyroTempComp,
                    const ImuCalibration& imuCal,
                    const RuntimeGyroBiasEstimator& runtimeBias,
                    const Vec3& currentBiasDps,
                    uint32_t gyroBiasFlags);
    bool enqueueMag(const MagProcessedSample& mag,
                    const MagHeadingSample& heading,
                    const MagFieldReliabilityOutput& reliability,
                    const MagYawCorrectionOutput& yaw,
                    uint32_t rejectFlagsForUse,
                    bool trustedForUse);
    bool enqueueState(const char* eventState,
                      const char* reason,
                      uint64_t timestampUs,
                      uint32_t flags,
                      float confidence);
    bool enqueueBiasUpdate(uint64_t timestampUs,
                           float temperatureC,
                           const Vec3& residualDps,
                           const Vec3& stdDps,
                           const Vec3& deltaDps,
                           const Vec3& trimDps,
                           uint32_t flags);
    bool enqueueNetwork(uint64_t timestampUs,
                        const TrackerWifiManager& wifi,
                        const SlimeVROutputRuntime& slimevr);

    // Serialize at most maxLines complete CSV lines. A multi-line IMU or MAG
    // bundle remains at the queue head until every line has been admitted, so
    // one background service call cannot consume an entire pose-deadline
    // budget merely because a full LOGVER3 bundle was due.
    bool service(uint8_t maxLines = TRACKER_MACHINE_LOG_LINES_PER_SERVICE);
    bool hasPending() const { return count_ != 0u; }
    MachineLogDeferredStatus status() const;
    void printStatus(Stream& out) const;

private:
    bool ready() const;
    MachineLogDeferredRecord* reserve(MachineLogDeferredRecordType type,
                                      TrackerLogMode mode,
                                      uint32_t sequence);
    bool outputHasRoom() const;
    static uint8_t lineCount(const MachineLogDeferredRecord& record);
    void writeRecordLine(const MachineLogDeferredRecord& record,
                         uint8_t line,
                         Stream& out);
    void writeImuLine(const MachineLogDeferredRecord& record,
                      uint8_t line,
                      Stream& out);
    void writeMagLine(const MachineLogDeferredRecord& record,
                      uint8_t line,
                      Stream& out);
    void writeState(const MachineLogDeferredRecord& record, Stream& out);
    void writeBiasUpdate(const MachineLogDeferredRecord& record, Stream& out);
    void writeNetwork(const MachineLogDeferredRecord& record, Stream& out);
    static void copyText(char* dst, size_t capacity, const char* src);

    TrackerSerialLogState* state_ = nullptr;
    MachineLogCounters* counters_ = nullptr;
    uint32_t* lastBiasEmitUs_ = nullptr;
    uint32_t biasPeriodUs_ = 0u;
    MachineLogDeferredRecord records_[TRACKER_MACHINE_LOG_QUEUE_RECORDS];
    uint8_t head_ = 0u;
    uint8_t tail_ = 0u;
    uint8_t count_ = 0u;
    uint8_t headLine_ = 0u;
    MachineLogDeferredStatus status_;
};

void machineLogPrintU64Dec(Stream& out, uint64_t v);
const char* machineLogModeName(TrackerLogMode mode);
void machineLogResetCounters(MachineLogCounters& counters, uint32_t& lastBiasEmitUs);
void machineLogEmitHeader(Stream& out,
                          const TrackerSerialLogState& state,
                          const TrackerConfig& config);
void machineLogPrintSummary(Stream& out,
                            const TrackerSerialLogState& state,
                            const MachineLogCounters& counters,
                            uint32_t runtimeSamples,
                            uint32_t trackingRecoveryEnterCount,
                            const ImuQualityMonitor& quality,
                            const Lsm6dsvFifoReader& fifo,
                            const MagRuntimeProcessor& magProcessor,
                            const MagYawCorrectionController& magYawCorrection,
                            const Ahrs6Dof& ahrs,
                            const MagProcessedSample& lastMagProcessed,
                            const MagYawCorrectionOutput& lastMagYawCorrection,
                            const RuntimeGyroBiasEstimator& runtimeBias);

} // namespace tracker

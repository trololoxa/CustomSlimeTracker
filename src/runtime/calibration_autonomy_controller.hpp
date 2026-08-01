#pragma once

#include <Arduino.h>
#include <cstdint>

#include "config/tracker_config_store.hpp"
#include "runtime/calibration_autonomy_store.hpp"
#include "runtime/mag_runtime_controller.hpp"
#include "runtime/runtime_bias_types.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "sensor/accel_6pos_calibration.hpp"
#include "sensor/calibration.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"
#include "sensor/mag_axis_alignment.hpp"
#include "sensor/mag_field_reliability.hpp"

namespace tracker {

enum class CalibrationAutonomyState : uint8_t {
    Observing = 0,
    CandidateReady,
    PersistPending,
    PromotionPending,
    PromotionCommitPending,
    PromotedProbation,
    AcceptedCleanup,
    RollbackPending,
    RejectedCooldown,
    SuspendedManualCandidate,
    SuspendedRealtime,
    SuspendedStorage,
};

enum class CalibrationAutonomyRejectReason : uint8_t {
    None = 0,
    CandidateNotBetter,
    CandidateStale,
    RealtimeFault,
    FreshEvidenceRegression,
    MagneticProbationFailed,
    StorageFailure,
    UserDisabled,
    RejectedFingerprint,
};

struct CalibrationAutonomyStats {
    uint32_t samplesObserved = 0;
    uint32_t samplesRejected = 0;
    uint32_t stationaryWindows = 0;
    uint32_t stationaryWindowsRejected = 0;
    uint32_t observationWindowsDeferred = 0;
    uint32_t observationWindowDrops = 0;
    uint32_t independentSessions = 0;
    uint32_t gyroProposals = 0;
    uint32_t temperatureProposals = 0;
    uint32_t accelProposals = 0;
    uint32_t proposalsSuppressed = 0;
    uint32_t candidatesStaged = 0;
    uint32_t candidateStageFailures = 0;
    uint32_t candidateFlushes = 0;
    uint32_t candidateFlushFailures = 0;
    uint32_t promotionsPrepared = 0;
    uint32_t promotionsCommitted = 0;
    uint32_t promotionFailures = 0;
    uint32_t probationWindows = 0;
    uint32_t probationRegressions = 0;
    uint32_t accepts = 0;
    uint32_t rollbacks = 0;
    uint32_t rollbackFailures = 0;
    uint32_t serviceDeferrals = 0;
    uint32_t serviceCalls = 0;
    uint32_t serviceWorked = 0;
    uint32_t lastServiceUs = 0;
    uint32_t maxServiceUs = 0;
    uint32_t lastStorageUs = 0;
    uint32_t maxStorageUs = 0;
};

struct CalibrationAutonomyCallbacks {
    bool (*evaluateRealtimeGate)(MagDeferredServiceGate& gate, void* user) = nullptr;
    void* evaluateRealtimeGateUser = nullptr;
    void (*applyCalibrationConfig)(const TrackerConfig& config, void* user) = nullptr;
    void* applyCalibrationConfigUser = nullptr;
    void (*setWave0022RuntimeEnabled)(bool enabled, void* user) = nullptr;
    void* setWave0022RuntimeEnabledUser = nullptr;
    void (*resetWave0022Evidence)(void* user) = nullptr;
    void* resetWave0022EvidenceUser = nullptr;
    bool (*manualCalibrationTransactionActive)(void* user) = nullptr;
    void* manualCalibrationTransactionActiveUser = nullptr;
};

struct CalibrationAutonomyDeps {
    Stream* out = nullptr;
    TrackerConfig* config = nullptr;
    TrackerConfigStore* configStore = nullptr;
    CalibrationAutonomyStore* autonomyStore = nullptr;
    ImuCalibration* imuCal = nullptr;
    GyroTempCompensator* gyroTempComp = nullptr;
    RuntimeGyroBiasEstimator* runtimeBias = nullptr;
    ImuQualityMonitor* quality = nullptr;
    SlimeVROutputRuntime* slimevr = nullptr;
    MagAxisAlignmentRuntimeState* magAxisState = nullptr;
    const MagFieldReliabilityOutput* magReliability = nullptr;
    CalibrationAutonomyCallbacks callbacks;
};

class CalibrationAutonomyController {
public:
    void begin(const CalibrationAutonomyDeps& deps, uint32_t nowMs);

    void observeImuSample(const Lsm6dsv::Sample& scaledSensorFrame,
                          const ImuQualityResult& quality,
                          uint64_t timestampUs,
                          uint32_t nowMs);

    // Cheap admission checks used by the 960 Hz sample path and the ordinary
    // application loop.  When autonomy is disabled and no transaction is
    // active, callers can avoid even entering the controller implementation.
    bool imuObservationRequired() const;
    bool deferredServiceRequired() const;
    void notifyCalibrationContractChanged();

    // Deferred service only: all candidate/store/journal work is performed here,
    // never in the ~960 Hz sample callback.
    bool service(uint32_t nowMs);

    bool setWave0022Enabled(bool enabled, bool persist, uint32_t nowMs);
    bool setWave0023Enabled(bool enabled, bool persist, uint32_t nowMs);
    bool wave0022Enabled() const { return wave0022Enabled_; }
    bool wave0023Enabled() const { return wave0023Enabled_; }

    bool requestRollback(uint32_t nowMs,
                         CalibrationAutonomyRejectReason reason =
                             CalibrationAutonomyRejectReason::UserDisabled);
    void resetRuntimeEvidence();
    bool clearRejectionMemory();
    bool clearPersistentCalibrationState();
    // Destructive recovery path used only by `cal erase_all confirm`. It does
    // not trust or resolve the journal; the caller must first erase/save a
    // calibrationless authoritative config.
    bool preparePersistentCalibrationErase(const TrackerConfig& cleanConfig);
    bool forceClearPersistentCalibrationStateForErase(uint32_t nowMs);

    // Explicit synchronous ownership hand-off used by setup/manual calibration.
    // It resolves any autonomous transaction, removes only autonomy-owned
    // candidates, freezes background collection and invalidates stale evidence.
    bool beginManualCalibration(uint32_t nowMs);
    void endManualCalibration(bool calibrationChanged, uint32_t nowMs);
    bool manualCalibrationLocked() const { return manualCalibrationDepth_ != 0u; }
    bool blocksMotionLightSleep() const;
    void printStatus(Stream& out, uint32_t nowMs) const;

    CalibrationAutonomyState state() const { return state_; }
    CalibrationAutonomySubsystem subsystem() const { return subsystem_; }
    CalibrationAutonomyRejectReason lastRejectReason() const { return lastRejectReason_; }
    const CalibrationAutonomyStats& stats() const { return stats_; }

private:
    static constexpr uint32_t kWindowSamples = 1024u;
    static constexpr uint32_t kMinSessionSeparationMs = 15000u;
    static constexpr uint32_t kSessionWithoutMotionSeparationMs = 60000u;
    static constexpr uint8_t kMaxSessions = 36u;
    static constexpr uint32_t kProbationMinMs = 60000u;
    static constexpr uint32_t kProbationMaxMs = 60u * 60u * 1000u;
    static constexpr uint32_t kServiceIntervalMs = 250u;
    static constexpr uint32_t kCandidateAbsentProbeIntervalMs = 5000u;
    static constexpr uint32_t kCandidateRetryIntervalMs = 2000u;

    struct WindowAccumulator {
        uint32_t count = 0;
        uint32_t rejectedStreak = 0;
        Vec3 gyroSum = Vec3::zero();
        Vec3 gyroSumSq = Vec3::zero();
        Vec3 accelSum = Vec3::zero();
        Vec3 accelSumSq = Vec3::zero();
        float tempSum = 0.0f;
        float tempMin = 0.0f;
        float tempMax = 0.0f;
        uint64_t firstTimestampUs = 0;
        uint64_t lastTimestampUs = 0;

        void reset();
        void push(const Lsm6dsv::Sample& scaled, uint64_t timestampUs);
        Vec3 gyroMean() const;
        Vec3 gyroStd() const;
        Vec3 accelMean() const;
        Vec3 accelVariance() const;
        float meanTemp() const;
    };

    struct Session {
        bool valid = false;
        uint32_t acceptedMs = 0;
        float tempC = 0.0f;
        Vec3 rawGyroMeanRadS = Vec3::zero();
        Vec3 gyroStdRadS = Vec3::zero();
        Vec3 rawAccelMeanG = Vec3::zero();
        Vec3 accelVarianceG2 = Vec3::zero();
        Accel6PosCalibration::Face face = Accel6PosCalibration::Face::Invalid;
    };

    CalibrationAutonomyDeps deps_;
    bool begun_ = false;
    bool wave0022Enabled_ = true;
    bool wave0023Enabled_ = true;
    CalibrationAutonomyState state_ = CalibrationAutonomyState::Observing;
    CalibrationAutonomySubsystem subsystem_ = CalibrationAutonomySubsystem::None;
    CalibrationAutonomyRejectReason lastRejectReason_ = CalibrationAutonomyRejectReason::None;
    CalibrationAutonomyStats stats_;

    WindowAccumulator window_;
    WindowAccumulator completedWindow_;
    bool completedWindowPending_ = false;
    uint32_t completedWindowNowMs_ = 0u;
    Session sessions_[kMaxSessions] = {};
    Accel6PosCalibration accelProposalCalibration_;
    Session accelBest_[6] = {};
    uint8_t sessionCount_ = 0;
    bool motionSeenSinceSession_ = true;
    uint32_t lastSessionMs_ = 0;
    uint32_t lastServiceMs_ = 0;
    uint32_t candidateProbeNotBeforeMs_ = 0;
    uint32_t stateChangedMs_ = 0;
    uint32_t proposalCreatedMs_ = 0;
    uint32_t probationStartedMs_ = 0;
    uint32_t probationAcceptedWindows_ = 0;
    uint8_t probationAccelFaceMask_ = 0;
    float probationCandidateResidualSum_ = 0.0f;
    float probationPreviousResidualSum_ = 0.0f;
    bool proposalPending_ = false;
    uint8_t manualCalibrationDepth_ = 0u;
    uint32_t proposalActiveRevision_ = 0;
    TrackerSensorSignature proposalSignature_{};

    TrackerConfig proposalConfig_;
    TrackerCalibrationCandidateMetadata proposalMetadata_;
    TrackerCalibrationCandidateRecord candidateRecord_;
    TrackerPreparedConfigPromotion prepared_;
    TrackerConfig candidateConfig_;
    TrackerConfig previousConfig_;
    TrackerConfig promotedConfig_;
    TrackerConfig rollbackConfig_;
    CalibrationAutonomyJournalRecord journal_;
    CalibrationAutonomyRejectionRecord rejection_;

    ImuQualityCounters probationQualityBaseline_{};
    SlimeVROutputHealthCounters probationSlimeBaseline_{};

    bool manualTransactionActive() const;
    bool reconcileStorageTransaction(uint32_t nowMs);
    bool realtimeGateAllows(MagDeferredServiceGate& gate) const;
    void transition(CalibrationAutonomyState state, uint32_t nowMs);
    void noteMotion(const Lsm6dsv::Sample& scaled, const ImuQualityResult& quality);
    bool sampleEligible(const Lsm6dsv::Sample& scaled,
                        const ImuQualityResult& quality) const;
    void finalizeWindow(const WindowAccumulator& window, uint32_t nowMs);
    bool windowLooksStationary(const WindowAccumulator& window, Session& session) const;
    void appendSession(const Session& session);
    void clearSessions();

    void maybeBuildProposal(uint32_t nowMs);
    bool buildGyroTemperatureProposal(uint32_t nowMs);
    bool buildGyroBiasProposal(uint32_t nowMs);
    bool buildAccelProposal(uint32_t nowMs);
    bool stagePendingProposal(uint32_t nowMs);

    CalibrationAutonomySubsystem classifyCandidate(
        const TrackerCalibrationCandidateRecord& candidate) const;
    bool candidateOwnedByAutonomy(const TrackerCalibrationCandidateRecord& candidate) const;
    uint32_t candidateFingerprint(const TrackerCalibrationCandidateRecord& candidate) const;
    bool rejectionSuppresses(uint32_t fingerprint) const;
    bool serviceCandidateLifecycle(uint32_t nowMs);
    bool beginPromotion(uint32_t nowMs);
    bool advancePromotion(uint32_t nowMs);
    bool recoverJournalAtBoot(uint32_t nowMs);
    bool recoverLegacyJournalV1AtBoot(
        const CalibrationAutonomyJournalRecordV1& legacy, uint32_t nowMs);
    bool recoverPendingEraseAtBoot(
        const CalibrationAutonomyEraseRecoveryRecord& recovery, uint32_t nowMs);
    bool enterProbationAfterPromotion(uint32_t nowMs);
    bool evaluateProbationWindow(const Session& session, uint32_t nowMs);
    bool probationHealthFailed() const;
    bool probationCanAccept(uint32_t nowMs) const;
    bool acceptPromotion(uint32_t nowMs);
    bool rollbackPromotion(uint32_t nowMs, CalibrationAutonomyRejectReason reason);
    bool completeRollbackSynchronously(uint32_t nowMs, CalibrationAutonomyRejectReason reason);
    bool writeJournal(CalibrationAutonomyJournalState state, uint32_t nowMs);
    bool recordRejection(uint32_t nowMs, CalibrationAutonomyRejectReason reason);
    void snapshotProbationHealth();

    static float gyroModelResidualDps(const TrackerConfigBlob& payload,
                                      const Session& session);
    static float accelModelResidualG(const TrackerConfigBlob& payload,
                                     const Session& session);
    static bool componentNonRegression(const Vec3& candidateResidual,
                                       const Vec3& activeResidual,
                                       float tolerance);
    static const char* stateName(CalibrationAutonomyState state);
    static const char* rejectReasonName(CalibrationAutonomyRejectReason reason);
};

} // namespace tracker

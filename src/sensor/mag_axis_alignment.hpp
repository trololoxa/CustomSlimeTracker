#pragma once

#include <cstdint>

#include "core/math.hpp"
#include "sensor/mag_runtime.hpp"

namespace tracker {

struct MagAxisAlignmentInterval {
    Vec3 gyroSensorRadS = Vec3::zero();
    Vec3 mag0Raw = Vec3::zero();
    Vec3 mag1Raw = Vec3::zero();
    float dtS = 0.0f;
    // Temporal coverage bucket.  Even/odd windows form deterministic
    // training/validation partitions so a candidate is not fitted and proven
    // on the exact same observations.
    uint16_t windowId = 0;
};

struct MagAxisAlignmentSolvePolicy {
    uint16_t minIntervals = 20;
    uint16_t minTrainingIntervals = 12;
    uint16_t minValidationIntervals = 8;
    uint8_t minExcitedAxes = 2;
    uint32_t minIndependentWindows = 4;
    uint32_t minTrainingWindows = 2;
    uint32_t minValidationWindows = 2;
    // Solver errors are expressed in degrees.  The total score combines
    // direction disagreement and finite rotation-magnitude disagreement.
    float maxScore = 4.0f;
    float maxMeanDirectionError = 3.0f;
    // Separation is normalized by observable angular step so confidence is
    // comparable at ordinary 30-120 deg/s and at faster motion.
    float minNormalizedSeparation = 0.35f;
    float minMeanObservableStepDeg = 0.20f;
    float minTotalObservableRotationDeg = 22.0f;
    float maxRefinementDeg = 12.0f;
    float minActiveAbsoluteImprovement = 0.02f;
    float minActiveRelativeImprovement = 0.12f;
    float minActiveNormalizedImprovement = 0.01f;
    float maxValidationGeneralizationGap = 1.25f;
    float maxTrainingValidationRotationDifferenceDeg = 1.25f;
};

struct MagAxisAlignmentResult {
    bool valid = false;
    bool refined = false;
    bool activeCompared = false;
    bool improvesActive = false;
    bool validationPassed = false;
    bool validationWinnerMatchesTraining = false;

    Mat3 magToImu = Mat3::identity();
    Mat3 coarseMagToImu = Mat3::identity();

    float score = 999.0f;
    float trainingScore = 999.0f;
    float validationScore = 999.0f;
    float coarseScore = 999.0f;
    float secondBestScore = 999.0f;
    float trainingSecondBestScore = 999.0f;
    float validationSecondBestScore = 999.0f;
    float coarseSecondBestScore = 999.0f;
    float activeScore = 999.0f;
    float activeTrainingScore = 999.0f;

    float meanDirectionError = 999.0f;
    float meanMagnitudeError = 999.0f;
    float activeMeanDirectionError = 999.0f;
    float activeMeanMagnitudeError = 999.0f;
    float trainingMeanDirectionError = 999.0f;
    float validationMeanDirectionError = 999.0f;
    float validationMeanMagnitudeError = 999.0f;

    float refinementAngleDeg = 0.0f;
    float qualityScore = 0.0f;
    float normalizedSeparation = 0.0f;
    float meanObservableStepDeg = 0.0f;
    float totalObservableRotationDeg = 0.0f;
    float trainingValidationRotationDifferenceDeg = 999.0f;
    uint16_t refinementEvaluations = 0;
    uint16_t usedIntervals = 0;
    uint16_t trainingUsedIntervals = 0;
    uint16_t validationUsedIntervals = 0;
    uint16_t activeUsedIntervals = 0;
    uint8_t excitedAxes = 0;
    uint32_t independentWindows = 0;
    uint32_t trainingWindows = 0;
    uint32_t validationWindows = 0;
};

struct MagAxisAlignmentCollectorStats {
    uint32_t magSamplesSeen = 0;
    uint32_t intervalsAccepted = 0;
    uint32_t intervalsRejectedUntrusted = 0;
    uint32_t intervalsRejectedTiming = 0;
    uint32_t intervalsRejectedGyroSkew = 0;
    uint32_t intervalsRejectedMotion = 0;
    uint32_t intervalsSkippedCadence = 0;
    uint32_t intervalsRejectedCapacity = 0;
    uint32_t solveAttempts = 0;
    uint32_t solveSuccesses = 0;
};

bool solveMagAxisAlignmentDataset(const MagAxisAlignmentInterval* intervals,
                                  uint16_t intervalCount,
                                  const Vec3& hardIron,
                                  const Mat3& softIron,
                                  uint8_t excitedAxes,
                                  uint32_t independentWindows,
                                  const Mat3* activeAlignment,
                                  const MagAxisAlignmentSolvePolicy& policy,
                                  MagAxisAlignmentResult& out);

float magAxisAlignmentQualityScore(const MagAxisAlignmentResult& result);
float magAxisRotationDifferenceDeg(const Mat3& a, const Mat3& b);
bool magAxisMatricesEquivalent(const Mat3& a, const Mat3& b, float toleranceDeg = 0.75f);

class MagAxisAlignmentCollector {
public:
    static constexpr uint16_t kMaxIntervals = 64;
    static constexpr uint16_t kMinSolveIntervals = 20;
    static constexpr uint16_t kTargetIntervals = 32;
    static constexpr uint32_t kMinAcceptedSpacingMs = 75;

    void reset();

    bool observe(const Vec3& gyroSensorRadS,
                 uint64_t gyroTimestampUs,
                 const MagProcessedSample& mag,
                 bool fieldReliable);

    bool solve(const Vec3& hardIron,
               const Mat3& softIron,
               MagAxisAlignmentResult& out,
               const Mat3* activeAlignment = nullptr);

    uint16_t intervalCount() const { return intervalCount_; }
    uint32_t independentWindows() const { return independentWindows_; }
    uint8_t excitedAxes() const;
    bool readyToSolve() const;
    const MagAxisAlignmentCollectorStats& stats() const { return stats_; }

    static Mat3 signedPermutation(uint8_t ax0, float s0,
                                  uint8_t ax1, float s1,
                                  uint8_t ax2, float s2);
    static bool properRotation(const Mat3& m);

private:
    MagAxisAlignmentInterval intervals_[kMaxIntervals];
    uint16_t intervalCount_ = 0;
    bool havePreviousMag_ = false;
    Vec3 previousMagRaw_ = Vec3::zero();
    uint64_t previousMagUs_ = 0;
    bool havePreviousGyro_ = false;
    Vec3 previousGyroSensorRadS_ = Vec3::zero();
    float axisExcitationRad_[3] = {};
    uint32_t independentWindows_ = 0;
    uint16_t currentWindowId_ = 0;
    uint32_t lastWindowMs_ = 0;
    uint32_t lastAcceptedMs_ = 0;
    MagAxisAlignmentCollectorStats stats_;
};

enum class MagAxisAlignmentDeferredAction : uint8_t {
    None = 0,
    Solve = 1,
    Stage = 2,
    CheckCandidateSlot = 3,
};

struct MagAxisAlignmentRuntimeState {
    bool candidateStaged = false;
    bool activeAlignmentConfirmed = false;
    bool blockedByExistingCandidate = false;
    bool solvePending = false;
    bool stagePending = false;
    bool lastSolveValid = false;
    MagAxisAlignmentDeferredAction pendingAction = MagAxisAlignmentDeferredAction::None;

    uint32_t stageAttempts = 0;
    uint32_t stageSuccesses = 0;
    uint32_t stageFailures = 0;
    uint32_t solveServiceCalls = 0;
    uint32_t storageServiceCalls = 0;
    uint32_t serviceDeferrals = 0;
    uint32_t serviceDeferralSoftwareFifo = 0;
    uint32_t serviceDeferralHardwareStatus = 0;
    uint32_t serviceDeferralHardwareBusy = 0;
    uint32_t serviceDeferralOutputDeadline = 0;
    uint32_t lastSolveAttemptMs = 0;
    uint32_t lastStorageCheckMs = 0;
    uint32_t lastStageMs = 0;

    uint32_t lastSolveUs = 0;
    uint32_t maxSolveUs = 0;
    uint32_t lastStorageUs = 0;
    uint32_t maxStorageUs = 0;
    uint16_t lastDeferredFifoUnreadWords = 0;
    uint16_t maxDeferredFifoUnreadWords = 0;
    uint32_t lastDeferredRotationSlackMs = 0xFFFFFFFFUL;
    uint32_t lastDeferredRejectFlags = 0;

    MagAxisAlignmentResult lastResult;

    void reset() { *this = MagAxisAlignmentRuntimeState{}; }
};

} // namespace tracker

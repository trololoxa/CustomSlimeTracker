#pragma once

#include <cstdint>
#include <cmath>

#include "core/deterministic_reservoir.hpp"
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

enum class MagAxisIntervalReservoirAction : uint8_t {
    Added = 0,
    Replaced,
    Skipped,
};

// Guided collection can observe thousands of valid gyro/mag intervals. A
// first-N buffer freezes after a few seconds and ignores later axes. This
// bounded reservoir is stratified by dominant gyro axis and train/validation
// window parity, so evidence from every observed axis/partition remains
// eligible for the final solve while memory stays fixed.
template <uint16_t Capacity>
class MagAxisIntervalReservoir {
public:
    static_assert(Capacity >= 6u, "mag-axis reservoir needs all six strata");
    static_assert(Capacity <= 255u, "mag-axis reservoir slot indexes are uint8_t");
    static constexpr uint8_t kBucketCount = 6u;
    static constexpr uint16_t kMaxBucketCapacity =
        static_cast<uint16_t>((Capacity + kBucketCount - 1u) / kBucketCount);

    void reset() {
        for (auto& interval : intervals_) interval = MagAxisAlignmentInterval{};
        for (auto& bucket : bucketSlots_) {
            for (uint8_t& slot : bucket) slot = 0u;
        }
        for (uint16_t& count : bucketCounts_) count = 0u;
        for (uint32_t& seen : bucketSeen_) seen = 0u;
        for (float& excitation : axisExcitationRad_) excitation = 0.0f;
        for (float& excitation : bucketExcitationRad_) excitation = 0.0f;
        size_ = 0u;
        seen_ = 0u;
        replacements_ = 0u;
        skipped_ = 0u;
        independentWindows_ = 0u;
    }

    MagAxisIntervalReservoirAction consider(const MagAxisAlignmentInterval& interval) {
        const uint8_t bucket = bucketFor(interval);
        const uint16_t capacity = bucketCapacity(bucket);
        const uint32_t seenBefore = bucketSeen_[bucket];
        const uint32_t seenCount = seenBefore + 1u;
        bucketSeen_[bucket] = seenCount;
        seen_++;

        if (bucketCounts_[bucket] < capacity) {
            const bool newWindow = !containsWindow(interval.windowId, size_);
            const uint16_t slot = size_++;
            intervals_[slot] = interval;
            bucketSlots_[bucket][bucketCounts_[bucket]++] = static_cast<uint8_t>(slot);
            addExcitation(interval);
            if (newWindow) independentWindows_++;
            return MagAxisIntervalReservoirAction::Added;
        }

        const uint32_t candidate = deterministicReservoirCandidate(
            seenBefore,
            seenCount,
            0xA511E9B3u ^ (static_cast<uint32_t>(bucket) * 0x9E3779B9u));
        if (candidate < capacity) {
            const uint16_t slot = bucketSlots_[bucket][static_cast<uint16_t>(candidate)];
            const MagAxisAlignmentInterval previous = intervals_[slot];
            const bool previousWindowRemains = containsWindowExcept(previous.windowId, slot);
            const bool replacementWindowExists = containsWindowExcept(interval.windowId, slot);
            removeExcitation(previous);
            intervals_[slot] = interval;
            addExcitation(interval);
            if (previous.windowId != interval.windowId) {
                if (!previousWindowRemains && independentWindows_ > 0u) independentWindows_--;
                if (!replacementWindowExists) independentWindows_++;
            }
            replacements_++;
            return MagAxisIntervalReservoirAction::Replaced;
        }

        skipped_++;
        return MagAxisIntervalReservoirAction::Skipped;
    }

    const MagAxisAlignmentInterval* data() const { return intervals_; }
    uint16_t size() const { return size_; }
    uint32_t seen() const { return seen_; }
    uint32_t replacements() const { return replacements_; }
    uint32_t skipped() const { return skipped_; }
    bool active() const { return seen_ > size_; }
    uint16_t bucketCount(uint8_t bucket) const {
        return bucket < kBucketCount ? bucketCounts_[bucket] : 0u;
    }
    uint32_t bucketSeen(uint8_t bucket) const {
        return bucket < kBucketCount ? bucketSeen_[bucket] : 0u;
    }

    uint8_t excitedAxes(float minimumExcitationRad = 0.12f) const {
        uint8_t count = 0;
        for (float v : axisExcitationRad_) {
            if (v >= minimumExcitationRad) count++;
        }
        return count;
    }

    uint32_t independentWindows() const { return independentWindows_; }

    uint8_t partitionConfirmedAxes(float minimumExcitationRad = 0.06f) const {
        uint8_t count = 0u;
        for (uint8_t axis = 0u; axis < 3u; ++axis) {
            const uint8_t trainingBucket = static_cast<uint8_t>(axis * 2u);
            const uint8_t validationBucket = static_cast<uint8_t>(trainingBucket + 1u);
            if (bucketExcitationRad_[trainingBucket] >= minimumExcitationRad &&
                bucketExcitationRad_[validationBucket] >= minimumExcitationRad) {
                count++;
            }
        }
        return count;
    }

private:
    bool containsWindow(uint16_t windowId, uint16_t count) const {
        for (uint16_t i = 0; i < count; ++i) {
            if (intervals_[i].windowId == windowId) return true;
        }
        return false;
    }

    bool containsWindowExcept(uint16_t windowId, uint16_t excludedSlot) const {
        for (uint16_t i = 0; i < size_; ++i) {
            if (i != excludedSlot && intervals_[i].windowId == windowId) return true;
        }
        return false;
    }

    void addExcitation(const MagAxisAlignmentInterval& interval) {
        axisExcitationRad_[0] += std::fabs(interval.gyroSensorRadS.x) * interval.dtS;
        axisExcitationRad_[1] += std::fabs(interval.gyroSensorRadS.y) * interval.dtS;
        axisExcitationRad_[2] += std::fabs(interval.gyroSensorRadS.z) * interval.dtS;
        const uint8_t bucket = bucketFor(interval);
        const float dominant = std::fmax(
            std::fabs(interval.gyroSensorRadS.x),
            std::fmax(std::fabs(interval.gyroSensorRadS.y), std::fabs(interval.gyroSensorRadS.z)));
        bucketExcitationRad_[bucket] += dominant * interval.dtS;
    }

    void removeExcitation(const MagAxisAlignmentInterval& interval) {
        axisExcitationRad_[0] -= std::fabs(interval.gyroSensorRadS.x) * interval.dtS;
        axisExcitationRad_[1] -= std::fabs(interval.gyroSensorRadS.y) * interval.dtS;
        axisExcitationRad_[2] -= std::fabs(interval.gyroSensorRadS.z) * interval.dtS;
        const uint8_t bucket = bucketFor(interval);
        const float dominant = std::fmax(
            std::fabs(interval.gyroSensorRadS.x),
            std::fmax(std::fabs(interval.gyroSensorRadS.y), std::fabs(interval.gyroSensorRadS.z)));
        bucketExcitationRad_[bucket] -= dominant * interval.dtS;
        for (float& v : axisExcitationRad_) {
            if (v < 0.0f) v = 0.0f;
        }
        if (bucketExcitationRad_[bucket] < 0.0f) bucketExcitationRad_[bucket] = 0.0f;
    }

    static uint8_t bucketFor(const MagAxisAlignmentInterval& interval) {
        const float ax = std::fabs(interval.gyroSensorRadS.x);
        const float ay = std::fabs(interval.gyroSensorRadS.y);
        const float az = std::fabs(interval.gyroSensorRadS.z);
        uint8_t axis = 0u;
        float best = ax;
        if (ay > best) { axis = 1u; best = ay; }
        if (az > best) axis = 2u;
        return static_cast<uint8_t>(axis * 2u + (interval.windowId & 1u));
    }

    static constexpr uint16_t bucketCapacity(uint8_t bucket) {
        const uint16_t base = Capacity / kBucketCount;
        const uint16_t remainder = Capacity % kBucketCount;
        return static_cast<uint16_t>(base + (bucket < remainder ? 1u : 0u));
    }

    MagAxisAlignmentInterval intervals_[Capacity] = {};
    uint8_t bucketSlots_[kBucketCount][kMaxBucketCapacity] = {};
    uint16_t bucketCounts_[kBucketCount] = {};
    uint32_t bucketSeen_[kBucketCount] = {};
    uint16_t size_ = 0;
    uint32_t seen_ = 0;
    uint32_t replacements_ = 0;
    uint32_t skipped_ = 0;
    float axisExcitationRad_[3] = {};
    float bucketExcitationRad_[kBucketCount] = {};
    uint32_t independentWindows_ = 0;
};

enum class MagAxisAlignmentFailureReason : uint8_t {
    None = 0,
    InvalidCalibration,
    InsufficientIntervals,
    InsufficientAxes,
    InsufficientWindows,
    InsufficientPartitionWindows,
    CandidateBuildFailed,
    TrainingValidationDisagreement,
    InsufficientUsableIntervals,
    ValidationGeneralizationGap,
    ScoreTooHigh,
    DirectionErrorTooHigh,
    SeparationTooLow,
    ObservableStepTooLow,
    TotalRotationTooLow,
    InvalidRotation,
};

const char* magAxisAlignmentFailureReasonName(MagAxisAlignmentFailureReason reason);

// Bounded finite-rotation intervals keep the constant-rate SO(3) model
// observable without allowing long, highly non-uniform gestures to dominate.
constexpr float MAG_AXIS_MIN_INTERVAL_S = 0.004f;
constexpr float MAG_AXIS_MAX_INTERVAL_S = 0.200f;
constexpr float MAG_AXIS_MIN_GYRO_RATE_RAD_S = 3.0f * MATH_DEG_TO_RAD;
constexpr float MAG_AXIS_MAX_GYRO_RATE_RAD_S = 540.0f * MATH_DEG_TO_RAD;
constexpr float MAG_AXIS_MIN_OBSERVABLE_STEP_RAD = 0.0015f;
constexpr float MAG_AXIS_MAX_OBSERVABLE_STEP_RAD = 30.0f * MATH_DEG_TO_RAD;
constexpr uint64_t MAG_AXIS_MAX_GYRO_MAG_SKEW_US = 5000u;

enum class MagAxisIntervalBuildFailure : uint8_t {
    None = 0,
    InvalidInput,
    InvalidTiming,
    MotionOutOfRange,
};

bool buildMagAxisAlignmentInterval(const Vec3& gyro0SensorRadS,
                                   const Vec3& gyro1SensorRadS,
                                   const Vec3& mag0Raw,
                                   const Vec3& mag1Raw,
                                   float dtS,
                                   uint16_t windowId,
                                   MagAxisAlignmentInterval& out,
                                   MagAxisIntervalBuildFailure* failure = nullptr);

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
    MagAxisAlignmentFailureReason failureReason = MagAxisAlignmentFailureReason::None;
    bool refined = false;
    bool activeCompared = false;
    bool improvesActive = false;
    bool validationPassed = false;
    bool validationWinnerMatchesTraining = false;
    bool coarseWinnerMatchesTraining = false;
    bool continuousRefinementAgreement = false;
    bool coarseConsensusFallbackUsed = false;

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
    uint8_t partitionConfirmedAxes = 0;
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
    uint32_t intervalsReservoirReplaced = 0;
    uint32_t intervalsReservoirSkipped = 0;
    uint32_t solveAttempts = 0;
    uint32_t solveSuccesses = 0;
};

bool solveMagAxisAlignmentDataset(const MagAxisAlignmentInterval* intervals,
                                  uint16_t intervalCount,
                                  const Vec3& hardIron,
                                  const Mat3& softIron,
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
    static constexpr uint64_t kMinAcceptedSpacingUs = 75000u;

    void reset();

    bool observe(const Vec3& gyroSensorRadS,
                 uint64_t gyroTimestampUs,
                 const MagProcessedSample& mag,
                 bool fieldReliable);

    bool solve(const Vec3& hardIron,
               const Mat3& softIron,
               MagAxisAlignmentResult& out,
               const Mat3* activeAlignment = nullptr);

    uint16_t intervalCount() const { return intervalReservoir_.size(); }
    uint32_t independentWindows() const { return intervalReservoir_.independentWindows(); }
    uint8_t excitedAxes() const;
    uint8_t partitionConfirmedAxes() const { return intervalReservoir_.partitionConfirmedAxes(); }
    bool readyToSolve() const;
    const MagAxisAlignmentCollectorStats& stats() const { return stats_; }

    static Mat3 signedPermutation(uint8_t ax0, float s0,
                                  uint8_t ax1, float s1,
                                  uint8_t ax2, float s2);
    static bool properRotation(const Mat3& m);

private:
    MagAxisIntervalReservoir<kMaxIntervals> intervalReservoir_;
    bool havePreviousMag_ = false;
    Vec3 previousMagRaw_ = Vec3::zero();
    Vec3 previousMagCalibrated_ = Vec3::zero();
    uint64_t previousMagUs_ = 0;
    bool havePreviousGyro_ = false;
    Vec3 previousGyroSensorRadS_ = Vec3::zero();
    uint32_t windowsSeen_ = 0;
    uint16_t currentWindowId_ = 0;
    uint64_t lastWindowUs_ = 0;
    uint64_t lastAcceptedUs_ = 0;
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
    uint32_t evidenceQueued = 0;
    uint32_t evidenceProcessed = 0;
    uint32_t evidenceDropped = 0;
    uint32_t evidenceStaleDropped = 0;
    uint32_t evidenceServiceDeferrals = 0;
    uint8_t evidenceQueueHighWater = 0;
    uint32_t lastSolveAttemptMs = 0;
    uint32_t lastStorageCheckMs = 0;
    uint32_t lastStageMs = 0;
    uint32_t lastIndependentSessionMs = 0;
    uint32_t nextCollectionAllowedMs = 0;
    uint8_t confirmedIndependentSessions = 0;
    uint8_t sessionAgreementFailures = 0;
    uint16_t reservedSessions = 0;

    uint32_t lastSolveUs = 0;
    uint32_t maxSolveUs = 0;
    uint32_t lastStorageUs = 0;
    uint32_t maxStorageUs = 0;
    uint16_t lastDeferredFifoUnreadWords = 0;
    uint16_t maxDeferredFifoUnreadWords = 0;
    uint32_t lastDeferredRotationSlackMs = 0xFFFFFFFFUL;
    uint32_t lastDeferredRejectFlags = 0;

    MagAxisAlignmentResult lastResult;
    MagAxisAlignmentResult confirmedResult;

    void reset() { *this = MagAxisAlignmentRuntimeState{}; }
};

} // namespace tracker

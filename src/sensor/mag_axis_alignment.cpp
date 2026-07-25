#include "sensor/mag_axis_alignment.hpp"

#include <algorithm>
#include <cmath>

#include "sensor/frame_transform.hpp"

namespace tracker {

namespace {

struct CandidateScore {
    Mat3 matrix = Mat3::identity();
    Mat3 coarse = Mat3::identity();
    float score = 999.0f;
    float coarseScore = 999.0f;
    float directionError = 999.0f;
    float magnitudeError = 999.0f;
    float refinementDeg = 0.0f;
    uint16_t used = 0;
    uint16_t evaluations = 0;
};

enum class DatasetSubset : uint8_t {
    All = 0,
    Training = 1,
    Validation = 2,
};

struct DatasetMetrics {
    float score = 999.0f;
    float directionError = 999.0f;
    float magnitudeError = 999.0f;
    float meanObservableStepDeg = 0.0f;
    float totalObservableRotationDeg = 0.0f;
    uint16_t used = 0;
    uint32_t windows = 0;
};

bool intervalInSubset(const MagAxisAlignmentInterval& in, DatasetSubset subset) {
    if (subset == DatasetSubset::All) return true;
    const bool validation = (in.windowId & 1u) != 0u;
    return subset == DatasetSubset::Validation ? validation : !validation;
}

uint32_t countSubsetWindows(const MagAxisAlignmentInterval* intervals,
                            uint16_t intervalCount,
                            DatasetSubset subset) {
    if (!intervals || intervalCount == 0u) return 0u;
    uint16_t lastId = 0u;
    bool haveLast = false;
    uint32_t count = 0u;
    for (uint16_t i = 0; i < intervalCount; ++i) {
        if (!intervalInSubset(intervals[i], subset)) continue;
        const uint16_t id = intervals[i].windowId;
        if (!haveLast || id != lastId) {
            count++;
            lastId = id;
            haveLast = true;
        }
    }
    return count;
}

DatasetMetrics scoreCandidate(const MagAxisAlignmentInterval* intervals,
                              uint16_t intervalCount,
                              const Vec3& hardIron,
                              const Mat3& softIron,
                              const Mat3& candidate,
                              uint16_t minIntervals,
                              DatasetSubset subset) {
    DatasetMetrics metrics;
    float weightedErrorSum = 0.0f;
    float weightSum = 0.0f;
    float directionSum = 0.0f;
    float magnitudeSum = 0.0f;
    float observableStepSumDeg = 0.0f;

    if (!intervals || intervalCount == 0u || !candidate.isFinite() ||
        !isProperRotationMatrix(candidate, 0.03f, 0.03f, 0.08f)) {
        return metrics;
    }

    for (uint16_t i = 0; i < intervalCount; ++i) {
        const auto& in = intervals[i];
        if (!intervalInSubset(in, subset)) continue;
        if (!in.gyroSensorRadS.isFinite() || !in.mag0Raw.isFinite() ||
            !in.mag1Raw.isFinite() || in.dtS <= 0.0f || !tracker::isFinite(in.dtS)) {
            continue;
        }

        const Vec3 mag0Cal = softIron * (in.mag0Raw - hardIron);
        const Vec3 mag1Cal = softIron * (in.mag1Raw - hardIron);
        Vec3 m0 = candidate * mag0Cal;
        Vec3 m1 = candidate * mag1Cal;
        if (!m0.normalizeInPlace() || !m1.normalizeInPlace()) continue;

        // Compare the measured finite rotation with the exact constant-rate
        // SO(3) prediction over this interval.  The previous dm/dt objective
        // was only first-order accurate and created a pose/rate-dependent bias
        // on the bounded finite intervals retained by the collector.
        const Vec3 rotationVector = in.gyroSensorRadS * (-in.dtS);
        Vec3 predictedM1 = Quat::fromRotationVector(rotationVector).rotate(m0);
        if (!predictedM1.normalizeInPlace()) continue;

        const float agreement = clampf(dot(m1, predictedM1), -1.0f, 1.0f);
        const float dirErr = std::acos(agreement) * MATH_RAD_TO_DEG;
        const float observedStep = std::acos(clampf(dot(m0, m1), -1.0f, 1.0f));
        const float predictedStep = std::acos(clampf(dot(m0, predictedM1), -1.0f, 1.0f));
        if (observedStep < 0.0015f || predictedStep < 0.0015f ||
            !tracker::isFinite(observedStep) || !tracker::isFinite(predictedStep)) {
            continue;
        }
        const float magErr = std::fabs(observedStep - predictedStep) * MATH_RAD_TO_DEG;
        const float predictedRate = predictedStep / in.dtS;
        const float weight = clampf(predictedRate, 0.05f, 4.0f);
        weightedErrorSum += (dirErr + 0.25f * magErr) * weight;
        weightSum += weight;
        directionSum += dirErr;
        magnitudeSum += magErr;
        observableStepSumDeg += predictedStep * MATH_RAD_TO_DEG;
        metrics.used++;
    }

    if (metrics.used < minIntervals || weightSum <= 0.0f) {
        return metrics;
    }
    metrics.directionError = directionSum / static_cast<float>(metrics.used);
    metrics.magnitudeError = magnitudeSum / static_cast<float>(metrics.used);
    metrics.score = weightedErrorSum / weightSum;
    metrics.totalObservableRotationDeg = observableStepSumDeg;
    metrics.meanObservableStepDeg = observableStepSumDeg / static_cast<float>(metrics.used);
    return metrics;
}

float matrixTrace(const Mat3& m) {
    return m.m[0][0] + m.m[1][1] + m.m[2][2];
}

float relativeRotationAngleDeg(const Mat3& a, const Mat3& b) {
    if (!a.isFinite() || !b.isFinite()) return 999.0f;
    const Mat3 relative = a.transposed() * b;
    const float c = clampf((matrixTrace(relative) - 1.0f) * 0.5f, -1.0f, 1.0f);
    return std::acos(c) * MATH_RAD_TO_DEG;
}

Mat3 incrementalRotation(uint8_t axis, float angleDeg) {
    Vec3 rv = Vec3::zero();
    const float angleRad = angleDeg * MATH_DEG_TO_RAD;
    if (axis == 0u) rv.x = angleRad;
    else if (axis == 1u) rv.y = angleRad;
    else rv.z = angleRad;
    return Quat::fromRotationVector(rv).toRotationMatrix();
}

CandidateScore refineCandidate(const MagAxisAlignmentInterval* intervals,
                               uint16_t intervalCount,
                               const Vec3& hardIron,
                               const Mat3& softIron,
                               uint16_t minIntervals,
                               const Mat3& seed,
                               float seedScore,
                               float seedDirection,
                               float seedMagnitude,
                               uint16_t seedUsed,
                               float maxRefinementDeg,
                               DatasetSubset subset) {
    CandidateScore out;
    out.matrix = seed;
    out.coarse = seed;
    out.score = seedScore;
    out.coarseScore = seedScore;
    out.directionError = seedDirection;
    out.magnitudeError = seedMagnitude;
    out.used = seedUsed;

    static constexpr float kStepDeg[] = {4.0f, 2.0f, 1.0f, 0.5f, 0.25f, 0.125f, 0.0625f, 0.03125f};
    for (float stepDeg : kStepDeg) {
        for (uint8_t pass = 0; pass < 2u; ++pass) {
            bool improved = false;
            for (uint8_t axis = 0; axis < 3u; ++axis) {
                for (int sign = -1; sign <= 1; sign += 2) {
                    const Mat3 trial = incrementalRotation(axis, stepDeg * static_cast<float>(sign)) *
                                       out.matrix;
                    if (relativeRotationAngleDeg(seed, trial) > maxRefinementDeg + 0.001f) continue;
                    const DatasetMetrics metrics = scoreCandidate(
                        intervals, intervalCount, hardIron, softIron,
                        trial, minIntervals, subset);
                    out.evaluations++;
                    if (metrics.score + 1.0e-6f < out.score) {
                        out.matrix = trial;
                        out.score = metrics.score;
                        out.directionError = metrics.directionError;
                        out.magnitudeError = metrics.magnitudeError;
                        out.used = metrics.used;
                        improved = true;
                    }
                }
            }
            if (!improved) break;
        }
    }
    out.refinementDeg = relativeRotationAngleDeg(seed, out.matrix);
    return out;
}

void insertRanked(CandidateScore (&ranked)[4], const CandidateScore& candidate) {
    for (uint8_t i = 0; i < 4u; ++i) {
        if (candidate.score < ranked[i].score) {
            for (uint8_t j = 3u; j > i; --j) ranked[j] = ranked[j - 1u];
            ranked[i] = candidate;
            return;
        }
    }
}

bool buildRefinedCandidates(const MagAxisAlignmentInterval* intervals,
                            uint16_t intervalCount,
                            const Vec3& hardIron,
                            const Mat3& softIron,
                            uint16_t minIntervals,
                            float maxRefinementDeg,
                            DatasetSubset subset,
                            CandidateScore (&out)[4]) {
    static constexpr uint8_t perms[6][3] = {
        {0, 1, 2}, {0, 2, 1}, {1, 0, 2},
        {1, 2, 0}, {2, 0, 1}, {2, 1, 0},
    };

    CandidateScore ranked[4];
    for (const auto& p : perms) {
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                for (int sz = -1; sz <= 1; sz += 2) {
                    CandidateScore c;
                    c.matrix = MagAxisAlignmentCollector::signedPermutation(
                        p[0], static_cast<float>(sx),
                        p[1], static_cast<float>(sy),
                        p[2], static_cast<float>(sz));
                    c.coarse = c.matrix;
                    if (!MagAxisAlignmentCollector::properRotation(c.matrix)) continue;
                    const DatasetMetrics metrics = scoreCandidate(
                        intervals, intervalCount, hardIron, softIron,
                        c.matrix, minIntervals, subset);
                    c.score = metrics.score;
                    c.directionError = metrics.directionError;
                    c.magnitudeError = metrics.magnitudeError;
                    c.used = metrics.used;
                    c.coarseScore = c.score;
                    insertRanked(ranked, c);
                }
            }
        }
    }
    if (ranked[0].score >= 998.0f || ranked[1].score >= 998.0f) return false;

    for (uint8_t i = 0; i < 4u; ++i) {
        out[i] = refineCandidate(
            intervals, intervalCount, hardIron, softIron, minIntervals,
            ranked[i].matrix, ranked[i].score,
            ranked[i].directionError, ranked[i].magnitudeError,
            ranked[i].used, maxRefinementDeg, subset);
    }
    std::sort(out, out + 4,
              [](const CandidateScore& a, const CandidateScore& b) {
                  return a.score < b.score;
              });
    return out[0].score < 998.0f && out[1].score < 998.0f;
}

Mat3 rotationMidpoint(const Mat3& a, const Mat3& b) {
    const Mat3 relative = a.transposed() * b;
    const float angle = relativeRotationAngleDeg(a, b) * MATH_DEG_TO_RAD;
    if (!tracker::isFinite(angle) || angle <= 1.0e-7f) return a;
    const float sinAngle = std::sin(angle);
    if (std::fabs(sinAngle) <= 1.0e-6f) return a;
    Vec3 axis(
        relative.m[2][1] - relative.m[1][2],
        relative.m[0][2] - relative.m[2][0],
        relative.m[1][0] - relative.m[0][1]);
    axis *= 0.5f / sinAngle;
    if (!axis.normalizeInPlace()) return a;
    return a * Quat::fromRotationVector(axis * (0.5f * angle)).toRotationMatrix();
}

float clamp01Local(float v) {
    return clampf(v, 0.0f, 1.0f);
}

} // namespace

void MagAxisAlignmentCollector::reset() {
    *this = MagAxisAlignmentCollector{};
}

uint8_t MagAxisAlignmentCollector::excitedAxes() const {
    uint8_t count = 0;
    for (float v : axisExcitationRad_) {
        if (v >= 0.12f) count++;
    }
    return count;
}

bool MagAxisAlignmentCollector::readyToSolve() const {
    return intervalCount_ >= kTargetIntervals && excitedAxes() >= 2 && independentWindows_ >= 4;
}

bool MagAxisAlignmentCollector::observe(const Vec3& gyroSensorRadS,
                                        uint64_t gyroTimestampUs,
                                        const MagProcessedSample& mag,
                                        bool fieldReliable) {
    if (mag.seq == 0u || !mag.raw.isFinite() || mag.rawNorm <= MATH_EPSILON) return false;
    stats_.magSamplesSeen++;

    if (!fieldReliable) {
        stats_.intervalsRejectedUntrusted++;
        havePreviousMag_ = false;
        havePreviousGyro_ = false;
        return false;
    }

    if (gyroTimestampUs == 0u || mag.t_us == 0u ||
        (gyroTimestampUs > mag.t_us ? gyroTimestampUs - mag.t_us : mag.t_us - gyroTimestampUs) > 5000u) {
        stats_.intervalsRejectedGyroSkew++;
        havePreviousMag_ = false;
        havePreviousGyro_ = false;
        return false;
    }

    bool accepted = false;
    if (havePreviousMag_ && havePreviousGyro_ && gyroSensorRadS.isFinite()) {
        // The two coherent endpoint gyros provide a bounded trapezoidal
        // estimate of interval angular velocity without adding a 960 Hz
        // learner hook to the IMU hot path.
        const Vec3 intervalGyroSensorRadS =
            (previousGyroSensorRadS_ + gyroSensorRadS) * 0.5f;
        float dtS = 0.0f;
        if (mag.t_us > previousMagUs_ && previousMagUs_ != 0u) {
            dtS = static_cast<float>(mag.t_us - previousMagUs_) * 1.0e-6f;
        }
        if (dtS < 0.004f || dtS > 0.200f || !tracker::isFinite(dtS)) {
            stats_.intervalsRejectedTiming++;
        } else {
            const float gyroNormDps = intervalGyroSensorRadS.norm() * MATH_RAD_TO_DEG;
            const Vec3 m0 = previousMagRaw_.normalized();
            const Vec3 m1 = mag.raw.normalized();
            const float angle = std::acos(clampf(dot(m0, m1), -1.0f, 1.0f));
            if (angle < 0.0015f || gyroNormDps < 3.0f || gyroNormDps > 540.0f) {
                stats_.intervalsRejectedMotion++;
            } else if (lastAcceptedMs_ != 0u &&
                       mag.receivedMs - lastAcceptedMs_ < kMinAcceptedSpacingMs) {
                stats_.intervalsSkippedCadence++;
            } else if (intervalCount_ >= kMaxIntervals) {
                stats_.intervalsRejectedCapacity++;
            } else {
                if (lastWindowMs_ == 0u || mag.receivedMs - lastWindowMs_ >= 750u) {
                    currentWindowId_ = static_cast<uint16_t>(independentWindows_ & 0xFFFFu);
                    independentWindows_++;
                    lastWindowMs_ = mag.receivedMs;
                }
                intervals_[intervalCount_++] = MagAxisAlignmentInterval{
                    intervalGyroSensorRadS, previousMagRaw_, mag.raw, dtS,
                    currentWindowId_};
                stats_.intervalsAccepted++;
                lastAcceptedMs_ = mag.receivedMs;
                accepted = true;
                axisExcitationRad_[0] += std::fabs(intervalGyroSensorRadS.x) * dtS;
                axisExcitationRad_[1] += std::fabs(intervalGyroSensorRadS.y) * dtS;
                axisExcitationRad_[2] += std::fabs(intervalGyroSensorRadS.z) * dtS;
            }
        }
    }

    previousMagRaw_ = mag.raw;
    previousMagUs_ = mag.t_us;
    havePreviousMag_ = true;
    previousGyroSensorRadS_ = gyroSensorRadS;
    havePreviousGyro_ = gyroSensorRadS.isFinite();
    return accepted;
}

Mat3 MagAxisAlignmentCollector::signedPermutation(uint8_t ax0, float s0,
                                                  uint8_t ax1, float s1,
                                                  uint8_t ax2, float s2) {
    Mat3 m = Mat3::zero();
    m.m[0][ax0] = s0;
    m.m[1][ax1] = s1;
    m.m[2][ax2] = s2;
    return m;
}

bool MagAxisAlignmentCollector::properRotation(const Mat3& m) {
    return isProperRotationMatrix(m, 0.02f, 0.02f, 0.05f);
}

bool solveMagAxisAlignmentDataset(const MagAxisAlignmentInterval* intervals,
                                  uint16_t intervalCount,
                                  const Vec3& hardIron,
                                  const Mat3& softIron,
                                  uint8_t excitedAxes,
                                  uint32_t independentWindows,
                                  const Mat3* activeAlignment,
                                  const MagAxisAlignmentSolvePolicy& policy,
                                  MagAxisAlignmentResult& out) {
    out = MagAxisAlignmentResult{};
    out.excitedAxes = excitedAxes;
    out.independentWindows = independentWindows;
    if (!intervals || intervalCount < policy.minIntervals ||
        excitedAxes < policy.minExcitedAxes ||
        independentWindows < policy.minIndependentWindows ||
        !hardIron.isFinite() || !softIron.isFinite()) {
        return false;
    }

    out.trainingWindows = countSubsetWindows(
        intervals, intervalCount, DatasetSubset::Training);
    out.validationWindows = countSubsetWindows(
        intervals, intervalCount, DatasetSubset::Validation);
    if (out.trainingWindows < policy.minTrainingWindows ||
        out.validationWindows < policy.minValidationWindows) {
        return false;
    }

    // Training and validation solve the rotation independently.  Validation is
    // not merely asked to score hypotheses already optimized on training data:
    // both partitions must recover the same coarse signed permutation and
    // continuous SO(3) rotation before a candidate is considered proven.
    CandidateScore training[4];
    CandidateScore validation[4];
    if (!buildRefinedCandidates(
            intervals, intervalCount, hardIron, softIron,
            policy.minTrainingIntervals, policy.maxRefinementDeg,
            DatasetSubset::Training, training) ||
        !buildRefinedCandidates(
            intervals, intervalCount, hardIron, softIron,
            policy.minValidationIntervals, policy.maxRefinementDeg,
            DatasetSubset::Validation, validation)) {
        return false;
    }

    const CandidateScore& trainingBest = training[0];
    const CandidateScore& validationBest = validation[0];
    const bool coarseWinnerMatches = magAxisMatricesEquivalent(
        trainingBest.coarse, validationBest.coarse, 0.10f);
    out.trainingValidationRotationDifferenceDeg = relativeRotationAngleDeg(
        trainingBest.matrix, validationBest.matrix);
    const bool continuousWinnerMatches =
        out.trainingValidationRotationDifferenceDeg <=
        policy.maxTrainingValidationRotationDifferenceDeg;
    out.validationWinnerMatchesTraining = coarseWinnerMatches && continuousWinnerMatches;

    const Mat3 consensus = out.validationWinnerMatchesTraining
        ? rotationMidpoint(trainingBest.matrix, validationBest.matrix)
        : trainingBest.matrix;
    const DatasetMetrics trainingMetrics = scoreCandidate(
        intervals, intervalCount, hardIron, softIron,
        consensus, policy.minTrainingIntervals, DatasetSubset::Training);
    const DatasetMetrics validationMetrics = scoreCandidate(
        intervals, intervalCount, hardIron, softIron,
        consensus, policy.minValidationIntervals, DatasetSubset::Validation);
    const DatasetMetrics allMetrics = scoreCandidate(
        intervals, intervalCount, hardIron, softIron,
        consensus, policy.minIntervals, DatasetSubset::All);

    out.magToImu = consensus;
    out.coarseMagToImu = trainingBest.coarse;
    out.trainingScore = trainingMetrics.score;
    out.validationScore = validationMetrics.score;
    out.score = validationMetrics.score;
    out.coarseScore = trainingBest.coarseScore;
    out.coarseSecondBestScore = training[1].coarseScore;
    out.trainingSecondBestScore = training[1].score;
    out.validationSecondBestScore = validation[1].score;
    out.secondBestScore = validation[1].score;
    out.trainingMeanDirectionError = trainingMetrics.directionError;
    out.validationMeanDirectionError = validationMetrics.directionError;
    out.meanDirectionError = validationMetrics.directionError;
    out.validationMeanMagnitudeError = validationMetrics.magnitudeError;
    out.meanMagnitudeError = validationMetrics.magnitudeError;
    out.trainingUsedIntervals = trainingMetrics.used;
    out.validationUsedIntervals = validationMetrics.used;
    out.usedIntervals = allMetrics.used;
    out.meanObservableStepDeg = allMetrics.meanObservableStepDeg;
    out.totalObservableRotationDeg = allMetrics.totalObservableRotationDeg;
    out.refinementAngleDeg = relativeRotationAngleDeg(out.coarseMagToImu, consensus);
    out.refined = out.refinementAngleDeg > 0.05f;
    for (const auto& c : training) out.refinementEvaluations += c.evaluations;
    for (const auto& c : validation) out.refinementEvaluations += c.evaluations;

    const DatasetMetrics trainingWinnerMetrics = scoreCandidate(
        intervals, intervalCount, hardIron, softIron,
        trainingBest.matrix, policy.minTrainingIntervals, DatasetSubset::Training);
    const DatasetMetrics validationWinnerMetrics = scoreCandidate(
        intervals, intervalCount, hardIron, softIron,
        validationBest.matrix, policy.minValidationIntervals, DatasetSubset::Validation);
    const float trainingSeparation =
        (training[1].score - training[0].score) /
        std::max(trainingWinnerMetrics.meanObservableStepDeg, 0.05f);
    const float validationSeparation =
        (validation[1].score - validation[0].score) /
        std::max(validationWinnerMetrics.meanObservableStepDeg, 0.05f);
    out.normalizedSeparation = std::min(trainingSeparation, validationSeparation);

    const float generalizationGap = std::fabs(
        out.validationScore - out.trainingScore);
    out.validationPassed = out.validationWinnerMatchesTraining &&
        out.trainingUsedIntervals >= policy.minTrainingIntervals &&
        out.validationUsedIntervals >= policy.minValidationIntervals &&
        out.trainingWindows >= policy.minTrainingWindows &&
        out.validationWindows >= policy.minValidationWindows &&
        generalizationGap <= policy.maxValidationGeneralizationGap;

    if (activeAlignment && MagAxisAlignmentCollector::properRotation(*activeAlignment)) {
        out.activeCompared = true;
        const DatasetMetrics activeTraining = scoreCandidate(
            intervals, intervalCount, hardIron, softIron,
            *activeAlignment, policy.minTrainingIntervals,
            DatasetSubset::Training);
        const DatasetMetrics activeValidation = scoreCandidate(
            intervals, intervalCount, hardIron, softIron,
            *activeAlignment, policy.minValidationIntervals,
            DatasetSubset::Validation);
        out.activeTrainingScore = activeTraining.score;
        out.activeScore = activeValidation.score;
        out.activeMeanDirectionError = activeValidation.directionError;
        out.activeMeanMagnitudeError = activeValidation.magnitudeError;
        out.activeUsedIntervals = activeValidation.used;
        if (tracker::isFinite(out.activeScore) && out.activeScore < 998.0f) {
            const float trainingAbsoluteImprovement =
                out.activeTrainingScore - out.trainingScore;
            const float validationAbsoluteImprovement =
                out.activeScore - out.validationScore;
            const float trainingRelativeImprovement = trainingAbsoluteImprovement /
                std::max(out.activeTrainingScore, 0.01f);
            const float validationRelativeImprovement = validationAbsoluteImprovement /
                std::max(out.activeScore, 0.01f);
            const float trainingNormalizedImprovement = trainingAbsoluteImprovement /
                std::max(trainingMetrics.meanObservableStepDeg, 0.05f);
            const float validationNormalizedImprovement = validationAbsoluteImprovement /
                std::max(validationMetrics.meanObservableStepDeg, 0.05f);
            // Both independent partitions must prove the configured improvement
            // margin.  A lucky held-out fluctuation cannot compensate for weak
            // or contradictory training evidence, and vice versa.
            const bool trainingImproves =
                (trainingAbsoluteImprovement >= policy.minActiveAbsoluteImprovement ||
                 trainingNormalizedImprovement >= policy.minActiveNormalizedImprovement) &&
                trainingRelativeImprovement >= policy.minActiveRelativeImprovement;
            const bool validationImproves =
                (validationAbsoluteImprovement >= policy.minActiveAbsoluteImprovement ||
                 validationNormalizedImprovement >= policy.minActiveNormalizedImprovement) &&
                validationRelativeImprovement >= policy.minActiveRelativeImprovement;
            out.improvesActive = out.validationPassed &&
                trainingImproves && validationImproves;
        }
    } else {
        out.improvesActive = out.validationPassed;
    }

    out.valid = out.usedIntervals >= policy.minIntervals &&
                out.excitedAxes >= policy.minExcitedAxes &&
                out.independentWindows >= policy.minIndependentWindows &&
                out.validationPassed &&
                out.validationScore < policy.maxScore &&
                out.validationMeanDirectionError < policy.maxMeanDirectionError &&
                out.normalizedSeparation >= policy.minNormalizedSeparation &&
                out.meanObservableStepDeg >= policy.minMeanObservableStepDeg &&
                out.totalObservableRotationDeg >= policy.minTotalObservableRotationDeg &&
                MagAxisAlignmentCollector::properRotation(out.magToImu);
    out.qualityScore = magAxisAlignmentQualityScore(out);
    return out.valid;
}

bool MagAxisAlignmentCollector::solve(const Vec3& hardIron,
                                      const Mat3& softIron,
                                      MagAxisAlignmentResult& out,
                                      const Mat3* activeAlignment) {
    stats_.solveAttempts++;
    MagAxisAlignmentSolvePolicy policy;
    policy.minIntervals = kMinSolveIntervals;
    policy.minTrainingIntervals = 12;
    policy.minValidationIntervals = 8;
    policy.minExcitedAxes = 2;
    policy.minIndependentWindows = 4;
    policy.minTrainingWindows = 2;
    policy.minValidationWindows = 2;
    const bool ok = solveMagAxisAlignmentDataset(
        intervals_, intervalCount_, hardIron, softIron,
        excitedAxes(), independentWindows_, activeAlignment, policy, out);
    if (ok) stats_.solveSuccesses++;
    return ok;
}

float magAxisAlignmentQualityScore(const MagAxisAlignmentResult& result) {
    if (!result.valid || !result.validationPassed ||
        !tracker::isFinite(result.validationScore) ||
        !tracker::isFinite(result.validationSecondBestScore) ||
        !tracker::isFinite(result.validationMeanDirectionError) ||
        !tracker::isFinite(result.normalizedSeparation)) {
        return 0.0f;
    }
    const float fit = clamp01Local((4.0f - result.validationScore) / 4.0f);
    const float direction = clamp01Local(
        (3.0f - result.validationMeanDirectionError) / 3.0f);
    const float separation = clamp01Local(
        (result.normalizedSeparation - 0.35f) / 1.25f);
    const float excitation = clamp01Local(
        (result.totalObservableRotationDeg - 22.0f) / 78.0f);
    const float agreement = clamp01Local(
        (1.25f - result.trainingValidationRotationDifferenceDeg) / 1.25f);
    const float intervalCoverage = clamp01Local(static_cast<float>(result.usedIntervals) / 48.0f);
    const float axisCoverage = clamp01Local(static_cast<float>(result.excitedAxes) / 3.0f);
    const float windowCoverage = clamp01Local(static_cast<float>(result.independentWindows) / 6.0f);
    const float coverage = intervalCoverage * (0.5f * axisCoverage + 0.5f * windowCoverage);
    return clamp01Local(0.25f * fit + 0.15f * direction +
                        0.20f * separation + 0.15f * excitation +
                        0.10f * agreement + 0.15f * coverage);
}

float magAxisRotationDifferenceDeg(const Mat3& a, const Mat3& b) {
    if (!MagAxisAlignmentCollector::properRotation(a) ||
        !MagAxisAlignmentCollector::properRotation(b)) {
        return 999.0f;
    }
    return relativeRotationAngleDeg(a, b);
}

bool magAxisMatricesEquivalent(const Mat3& a, const Mat3& b, float toleranceDeg) {
    return magAxisRotationDifferenceDeg(a, b) <= toleranceDeg;
}

} // namespace tracker

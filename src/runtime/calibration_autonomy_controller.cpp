#include "runtime/calibration_autonomy_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "config/tracker_config_storage.hpp"

namespace tracker {

namespace {

constexpr float kRadToDps = MATH_RAD_TO_DEG;
constexpr uint32_t kBadQualityMask =
    imu_quality_flags::TIMESTAMP_ZERO |
    imu_quality_flags::TIMESTAMP_NON_MONOTONIC |
    imu_quality_flags::TIMESTAMP_LARGE_GAP |
    imu_quality_flags::TIMESTAMP_QUEUE_OVERFLOW |
    imu_quality_flags::TIMESTAMP_META_MISMATCH |
    imu_quality_flags::TIMESTAMP_BACKWARDS |
    imu_quality_flags::FIFO_OVERRUN |
    imu_quality_flags::FIFO_FULL |
    imu_quality_flags::FIFO_UNKNOWN_TAG |
    imu_quality_flags::FIFO_ORPHAN_WORDS |
    imu_quality_flags::FIFO_TAG_COUNTER_JUMP |
    imu_quality_flags::FIFO_GYRO_TAG_COUNTER_JUMP |
    imu_quality_flags::FIFO_ACCEL_TAG_COUNTER_JUMP |
    imu_quality_flags::FIFO_RECOVERY_REQUESTED |
    imu_quality_flags::GYRO_SATURATED |
    imu_quality_flags::ACCEL_SATURATED |
    imu_quality_flags::SAMPLE_DROPPED_BEFORE |
    imu_quality_flags::ACCEL_COMPONENT_MISSING |
    imu_quality_flags::GYRO_COMPONENT_MISSING |
    imu_quality_flags::FIFO_PAIR_DEGRADED |
    imu_quality_flags::FIFO_COMPLETED_QUEUE_OVERFLOW;

float clamp01(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

float maxAbs(const Vec3& value) {
    return std::max(std::fabs(value.x), std::max(std::fabs(value.y), std::fabs(value.z)));
}

Vec3 absVec(const Vec3& value) {
    return Vec3(std::fabs(value.x), std::fabs(value.y), std::fabs(value.z));
}

Vec3 sqrtVec(const Vec3& value) {
    return Vec3(std::sqrt(std::max(0.0f, value.x)),
                std::sqrt(std::max(0.0f, value.y)),
                std::sqrt(std::max(0.0f, value.z)));
}

Vec3 gyroBiasAt(const TrackerConfigBlob& payload, float tempC) {
    if (!payload.gyroCal.biasValid) return Vec3::zero();
    Vec3 bias = payload.gyroCal.biasRadS;
    if (payload.gyroCal.tempCompValid && payload.gyroCal.tempCompEnabled) {
        bias += payload.gyroCal.tempSlopeRadSPerC *
                (tempC - payload.gyroCal.referenceTempC);
    }
    return bias;
}

Vec3 accelApply(const TrackerConfigBlob& payload, const Vec3& rawAccelG) {
    if (!payload.accelCal.valid) return rawAccelG;
    return payload.accelCal.scale * (rawAccelG - payload.accelCal.biasG);
}

float faceResidual(const Vec3& calibrated,
                   Accel6PosCalibration::Face face) {
    const Vec3 expected = Accel6PosCalibration::expectedVector(face);
    return (calibrated - expected).norm();
}

bool finiteFloat(float value) {
    return std::isfinite(value);
}

} // namespace

void CalibrationAutonomyController::WindowAccumulator::reset() {
    *this = WindowAccumulator{};
}

void CalibrationAutonomyController::WindowAccumulator::push(
    const Lsm6dsv::Sample& scaled, uint64_t timestampUs) {
    if (count == 0u) {
        tempMin = scaled.temp_c;
        tempMax = scaled.temp_c;
        firstTimestampUs = timestampUs;
    } else {
        tempMin = std::min(tempMin, scaled.temp_c);
        tempMax = std::max(tempMax, scaled.temp_c);
    }
    lastTimestampUs = timestampUs;
    ++count;
    gyroSum += scaled.gyro_rad_s;
    gyroSumSq += hadamard(scaled.gyro_rad_s, scaled.gyro_rad_s);
    accelSum += scaled.accel_g;
    accelSumSq += hadamard(scaled.accel_g, scaled.accel_g);
    tempSum += scaled.temp_c;
    rejectedStreak = 0u;
}

Vec3 CalibrationAutonomyController::WindowAccumulator::gyroMean() const {
    return count == 0u ? Vec3::zero() : gyroSum / static_cast<float>(count);
}

Vec3 CalibrationAutonomyController::WindowAccumulator::gyroStd() const {
    if (count < 2u) return Vec3::zero();
    const Vec3 mean = gyroMean();
    return sqrtVec(gyroSumSq / static_cast<float>(count) - hadamard(mean, mean));
}

Vec3 CalibrationAutonomyController::WindowAccumulator::accelMean() const {
    return count == 0u ? Vec3::zero() : accelSum / static_cast<float>(count);
}

Vec3 CalibrationAutonomyController::WindowAccumulator::accelVariance() const {
    if (count < 2u) return Vec3::zero();
    const Vec3 mean = accelMean();
    const Vec3 variance = accelSumSq / static_cast<float>(count) - hadamard(mean, mean);
    return Vec3(std::max(0.0f, variance.x),
                std::max(0.0f, variance.y),
                std::max(0.0f, variance.z));
}

float CalibrationAutonomyController::WindowAccumulator::meanTemp() const {
    return count == 0u ? 0.0f : tempSum / static_cast<float>(count);
}

bool CalibrationAutonomyController::manualTransactionActive() const {
    if (manualCalibrationDepth_ != 0u) return true;
    return deps_.callbacks.manualCalibrationTransactionActive &&
           deps_.callbacks.manualCalibrationTransactionActive(
               deps_.callbacks.manualCalibrationTransactionActiveUser);
}

bool CalibrationAutonomyController::imuObservationRequired() const {
    if (!begun_ || manualCalibrationDepth_ != 0u) return false;

    bool stateNeedsSamples = false;
    if (state_ == CalibrationAutonomyState::PromotedProbation) {
        stateNeedsSamples = subsystem_ == CalibrationAutonomySubsystem::GyroBias ||
            subsystem_ == CalibrationAutonomySubsystem::GyroTemperature ||
            subsystem_ == CalibrationAutonomySubsystem::Accelerometer;
    } else if (wave0023Enabled_ && !proposalPending_) {
        stateNeedsSamples = state_ == CalibrationAutonomyState::Observing ||
            state_ == CalibrationAutonomyState::RejectedCooldown;
    }
    if (!stateNeedsSamples) return false;
    return !manualTransactionActive();
}

bool CalibrationAutonomyController::deferredServiceRequired() const {
    if (!begun_) return false;

    if (CalibrationAutonomyStore::valid(journal_) || prepared_.valid ||
        proposalPending_ || completedWindowPending_) {
        return true;
    }

    switch (state_) {
        case CalibrationAutonomyState::CandidateReady:
        case CalibrationAutonomyState::PersistPending:
        case CalibrationAutonomyState::PromotionPending:
        case CalibrationAutonomyState::PromotionCommitPending:
        case CalibrationAutonomyState::PromotedProbation:
        case CalibrationAutonomyState::AcceptedCleanup:
        case CalibrationAutonomyState::RollbackPending:
            return true;
        case CalibrationAutonomyState::SuspendedStorage:
        case CalibrationAutonomyState::SuspendedManualCandidate:
            return false;
        default:
            break;
    }

    return wave0022Enabled_ || wave0023Enabled_;
}

bool CalibrationAutonomyController::realtimeGateAllows(MagDeferredServiceGate& gate) const {
    gate = MagDeferredServiceGate{};
    if (!deps_.callbacks.evaluateRealtimeGate) {
        gate.allowed = true;
        return true;
    }
    return deps_.callbacks.evaluateRealtimeGate(
        gate, deps_.callbacks.evaluateRealtimeGateUser);
}

void CalibrationAutonomyController::transition(CalibrationAutonomyState state,
                                                uint32_t nowMs) {
    state_ = state;
    stateChangedMs_ = nowMs;
}

void CalibrationAutonomyController::begin(const CalibrationAutonomyDeps& deps,
                                          uint32_t nowMs) {
    deps_ = deps;
    begun_ = deps_.config && deps_.configStore && deps_.autonomyStore;
    resetRuntimeEvidence();
    transition(CalibrationAutonomyState::Observing, nowMs);

    CalibrationAutonomyEraseRecoveryRecord eraseRecovery{};
    if (deps_.autonomyStore && deps_.autonomyStore->loadEraseRecovery(eraseRecovery)) {
        if (!recoverPendingEraseAtBoot(eraseRecovery, nowMs)) {
            return;
        }
    } else if (deps_.autonomyStore &&
               !deps_.autonomyStore->lastErrorIsNotFound()) {
        wave0022Enabled_ = false;
        wave0023Enabled_ = false;
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return;
    }

    CalibrationAutonomyPreferencesRecord prefs{};
    if (deps_.autonomyStore && deps_.autonomyStore->loadPreferences(prefs)) {
        wave0022Enabled_ = prefs.wave0022Enabled != 0u;
        wave0023Enabled_ = prefs.wave0023Enabled != 0u;
    } else if (deps_.autonomyStore && !deps_.autonomyStore->lastErrorIsNotFound()) {
        // Corrupt preferences must never turn autonomous calibration back on.
        // Rewrite a valid disabled record when NVS is writable; transient NVS
        // failures remain fail-closed in SuspendedStorage.
        wave0022Enabled_ = false;
        wave0023Enabled_ = false;
        if (!deps_.autonomyStore->savePreferences(false, false)) {
            transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        }
    }
    if (deps_.callbacks.setWave0022RuntimeEnabled) {
        deps_.callbacks.setWave0022RuntimeEnabled(
            wave0022Enabled_, deps_.callbacks.setWave0022RuntimeEnabledUser);
    }
    if (deps_.autonomyStore) {
        (void)deps_.autonomyStore->loadRejection(rejection_);
    }
    if (begun_ && state_ != CalibrationAutonomyState::SuspendedStorage) {
        (void)recoverJournalAtBoot(nowMs);
    }
}

void CalibrationAutonomyController::resetRuntimeEvidence() {
    window_.reset();
    completedWindow_.reset();
    completedWindowPending_ = false;
    completedWindowNowMs_ = 0u;
    clearSessions();
    motionSeenSinceSession_ = true;
    lastSessionMs_ = 0u;
    proposalPending_ = false;
    proposalActiveRevision_ = 0u;
    proposalSignature_ = TrackerSensorSignature{};
    probationAcceptedWindows_ = 0u;
    probationAccelFaceMask_ = 0u;
    probationCandidateResidualSum_ = 0.0f;
    probationPreviousResidualSum_ = 0.0f;
}

void CalibrationAutonomyController::notifyCalibrationContractChanged() {
    // All config/calibration mutation paths call this explicitly. Keeping the
    // epoch hand-off out of observeImuSample avoids FNV/signature work in the
    // approximately 960 Hz path, including while the tracker is moving.
    resetRuntimeEvidence();
    if (deps_.callbacks.resetWave0022Evidence) {
        deps_.callbacks.resetWave0022Evidence(
            deps_.callbacks.resetWave0022EvidenceUser);
    }
}

void CalibrationAutonomyController::clearSessions() {
    for (auto& session : sessions_) session = Session{};
    sessionCount_ = 0u;
}

void CalibrationAutonomyController::noteMotion(
    const Lsm6dsv::Sample& scaled, const ImuQualityResult& quality) {
    const float gyroDps = scaled.gyro_rad_s.norm() * kRadToDps;
    const float accelError = std::fabs(scaled.accel_g.norm() - 1.0f);
    if (gyroDps > 5.0f || accelError > 0.15f ||
        (quality.flags & kBadQualityMask) != 0u) {
        motionSeenSinceSession_ = true;
    }
}

bool CalibrationAutonomyController::sampleEligible(
    const Lsm6dsv::Sample& scaled, const ImuQualityResult& quality) const {
    if (!wave0023Enabled_ || manualTransactionActive() ||
        state_ == CalibrationAutonomyState::PromotionPending ||
        state_ == CalibrationAutonomyState::AcceptedCleanup ||
        state_ == CalibrationAutonomyState::RollbackPending) {
        return false;
    }
    if (!scaled.gyro_rad_s.isFinite() || !scaled.accel_g.isFinite() ||
        !finiteFloat(scaled.temp_c)) {
        return false;
    }
    if ((quality.flags & kBadQualityMask) != 0u ||
        quality.overallConfidence < 0.80f ||
        quality.accelConfidence < 0.75f) {
        return false;
    }
    const float accelNorm = scaled.accel_g.norm();
    if (std::fabs(accelNorm - 1.0f) > 0.10f) return false;

    const Vec3 baseBias = deps_.config
        ? gyroBiasAt(deps_.config->data, scaled.temp_c)
        : Vec3::zero();
    const float residualGyroDps = (scaled.gyro_rad_s - baseBias).norm() * kRadToDps;
    return residualGyroDps <= 1.5f;
}

void CalibrationAutonomyController::observeImuSample(
    const Lsm6dsv::Sample& scaledSensorFrame,
    const ImuQualityResult& quality,
    uint64_t timestampUs,
    uint32_t nowMs) {
    if (!imuObservationRequired()) return;
    ++stats_.samplesObserved;
    noteMotion(scaledSensorFrame, quality);

    if (!sampleEligible(scaledSensorFrame, quality)) {
        ++stats_.samplesRejected;
        ++window_.rejectedStreak;
        if (window_.rejectedStreak >= 32u && window_.count != 0u) {
            ++stats_.stationaryWindowsRejected;
            window_.reset();
        }
        return;
    }

    window_.push(scaledSensorFrame, timestampUs);
    if (window_.count >= kWindowSamples) {
        if (completedWindowPending_) {
            // Preserve the older complete evidence window. A service stall may
            // discard a newer background-calibration window, but must never
            // block or slow tracking to retain it.
            ++stats_.observationWindowDrops;
        } else {
            completedWindow_ = window_;
            completedWindowPending_ = true;
            completedWindowNowMs_ = nowMs;
            ++stats_.observationWindowsDeferred;
        }
        window_.reset();
    }
}

bool CalibrationAutonomyController::windowLooksStationary(
    const WindowAccumulator& window, Session& session) const {
    if (window.count < kWindowSamples) return false;
    const Vec3 gyroStd = window.gyroStd();
    const Vec3 accelVariance = window.accelVariance();
    const Vec3 accelStd = sqrtVec(accelVariance);
    const Vec3 rawGyroMean = window.gyroMean();
    const float tempSpan = window.tempMax - window.tempMin;
    const Vec3 baseBias = deps_.config
        ? gyroBiasAt(deps_.config->data, window.meanTemp())
        : Vec3::zero();
    const Vec3 gyroResidualDps = (rawGyroMean - baseBias) * kRadToDps;

    if (maxAbs(gyroStd * kRadToDps) > 0.18f ||
        gyroResidualDps.norm() > 0.35f ||
        maxAbs(accelStd) > 0.020f ||
        tempSpan > 0.20f) {
        return false;
    }

    session.valid = true;
    session.tempC = window.meanTemp();
    session.rawGyroMeanRadS = rawGyroMean;
    session.gyroStdRadS = gyroStd;
    session.rawAccelMeanG = window.accelMean();
    session.accelVarianceG2 = accelVariance;
    const auto face = Accel6PosCalibration::detectFace(session.rawAccelMeanG);
    session.face = face.valid ? face.face : Accel6PosCalibration::Face::Invalid;
    return true;
}

void CalibrationAutonomyController::finalizeWindow(
    const WindowAccumulator& window, uint32_t nowMs) {
    Session session{};
    if (!windowLooksStationary(window, session)) {
        ++stats_.stationaryWindowsRejected;
        return;
    }
    ++stats_.stationaryWindows;

    const bool separatedByTime = lastSessionMs_ == 0u ||
        static_cast<uint32_t>(nowMs - lastSessionMs_) >=
            kSessionWithoutMotionSeparationMs;
    const bool separatedByMotion = motionSeenSinceSession_ &&
        (lastSessionMs_ == 0u ||
         static_cast<uint32_t>(nowMs - lastSessionMs_) >= kMinSessionSeparationMs);
    if (lastSessionMs_ != 0u && !separatedByTime && !separatedByMotion) {
        ++stats_.stationaryWindowsRejected;
        return;
    }

    session.acceptedMs = nowMs;
    appendSession(session);
    ++stats_.independentSessions;
    lastSessionMs_ = nowMs;
    motionSeenSinceSession_ = false;

    if (state_ == CalibrationAutonomyState::PromotedProbation) {
        (void)evaluateProbationWindow(session, nowMs);
    } else if (wave0023Enabled_ && !proposalPending_) {
        maybeBuildProposal(nowMs);
    }
}

void CalibrationAutonomyController::appendSession(const Session& session) {
    if (sessionCount_ < kMaxSessions) {
        sessions_[sessionCount_++] = session;
        return;
    }
    for (uint8_t i = 1u; i < kMaxSessions; ++i) sessions_[i - 1u] = sessions_[i];
    sessions_[kMaxSessions - 1u] = session;
}

bool CalibrationAutonomyController::componentNonRegression(
    const Vec3& candidateResidual, const Vec3& activeResidual, float tolerance) {
    return std::fabs(candidateResidual.x) <= std::fabs(activeResidual.x) + tolerance &&
           std::fabs(candidateResidual.y) <= std::fabs(activeResidual.y) + tolerance &&
           std::fabs(candidateResidual.z) <= std::fabs(activeResidual.z) + tolerance;
}

void CalibrationAutonomyController::maybeBuildProposal(uint32_t nowMs) {
    if (!wave0023Enabled_ || proposalPending_ || manualTransactionActive()) return;
    if (buildGyroTemperatureProposal(nowMs)) return;
    if (buildGyroBiasProposal(nowMs)) return;
    (void)buildAccelProposal(nowMs);
}

bool CalibrationAutonomyController::buildGyroBiasProposal(uint32_t nowMs) {
    if (!deps_.config || sessionCount_ < 3u) return false;
    const uint8_t start = sessionCount_ - 3u;
    float minTemp = sessions_[start].tempC;
    float maxTemp = minTemp;
    Vec3 candidateBiasSum = Vec3::zero();
    Vec3 activeResidualSum = Vec3::zero();
    Vec3 candidateResidualSum = Vec3::zero();
    float activeSq = 0.0f;
    float candidateSq = 0.0f;

    for (uint8_t i = start; i < sessionCount_; ++i) {
        minTemp = std::min(minTemp, sessions_[i].tempC);
        maxTemp = std::max(maxTemp, sessions_[i].tempC);
        Vec3 biasAtReference = sessions_[i].rawGyroMeanRadS;
        if (deps_.config->data.gyroCal.tempCompValid &&
            deps_.config->data.gyroCal.tempCompEnabled) {
            biasAtReference -= deps_.config->data.gyroCal.tempSlopeRadSPerC *
                (sessions_[i].tempC - deps_.config->data.gyroCal.referenceTempC);
        }
        candidateBiasSum += biasAtReference;
    }
    if (maxTemp - minTemp > 1.0f) return false;
    const Vec3 candidateBias = candidateBiasSum / 3.0f;

    for (uint8_t i = start; i < sessionCount_; ++i) {
        const Vec3 activeResidual = sessions_[i].rawGyroMeanRadS -
            gyroBiasAt(deps_.config->data, sessions_[i].tempC);
        Vec3 candidateAtTemp = candidateBias;
        if (deps_.config->data.gyroCal.tempCompValid &&
            deps_.config->data.gyroCal.tempCompEnabled) {
            candidateAtTemp += deps_.config->data.gyroCal.tempSlopeRadSPerC *
                (sessions_[i].tempC - deps_.config->data.gyroCal.referenceTempC);
        }
        const Vec3 candidateResidual = sessions_[i].rawGyroMeanRadS - candidateAtTemp;
        activeResidualSum += absVec(activeResidual * kRadToDps);
        candidateResidualSum += absVec(candidateResidual * kRadToDps);
        activeSq += activeResidual.normSq();
        candidateSq += candidateResidual.normSq();
    }
    activeResidualSum /= 3.0f;
    candidateResidualSum /= 3.0f;
    const float activeRmsDps = std::sqrt(activeSq / 3.0f) * kRadToDps;
    const float candidateRmsDps = std::sqrt(candidateSq / 3.0f) * kRadToDps;

    if (!componentNonRegression(candidateResidualSum, activeResidualSum, 0.005f) ||
        candidateRmsDps + 0.020f >= activeRmsDps ||
        candidateRmsDps > 0.10f ||
        maxAbs((candidateBias - deps_.config->data.gyroCal.biasRadS) * kRadToDps) > 1.5f) {
        return false;
    }

    proposalConfig_ = *deps_.config;
    proposalConfig_.data.gyroCal.biasValid = true;
    proposalConfig_.data.gyroCal.biasRadS = candidateBias;
    proposalConfig_.data.gyroCalMeta.biasCalibrationUptimeMs = nowMs;
    proposalConfig_.sanitize();
    proposalConfig_.updateCrc();

    proposalMetadata_ = TrackerCalibrationCandidateMetadata{};
    proposalMetadata_.provenance = TrackerCalibrationProvenance::Background;
    proposalMetadata_.sampleCount = 3u * kWindowSamples;
    proposalMetadata_.independentWindowCount = 3u;
    proposalMetadata_.quality = trackerCalibrationQualityFromConfig(proposalConfig_);
    proposalMetadata_.quality.gyroResidualDps = candidateRmsDps;
    proposalMetadata_.quality.gyroScore = clamp01(1.0f - candidateRmsDps / 0.35f);
    proposalMetadata_.quality.qualityFlags |=
        tracker_calibration_quality_flags::AUTONOMY_0023 |
        tracker_calibration_quality_flags::GYRO_MEASURED |
        tracker_calibration_quality_flags::SOURCE_MEASURED;
    trackerCalibrationQualityRecomputeOverall(proposalConfig_, proposalMetadata_.quality);
    trackerCalibrationQualitySetProvenance(
        proposalMetadata_.quality, TrackerCalibrationProvenance::Background);
    proposalPending_ = true;
    proposalActiveRevision_ = trackerCalibrationPayloadRevision(*deps_.config);
    proposalSignature_ = trackerMakeSensorSignature(*deps_.config);
    subsystem_ = CalibrationAutonomySubsystem::GyroBias;
    proposalCreatedMs_ = nowMs;
    transition(CalibrationAutonomyState::CandidateReady, nowMs);
    ++stats_.gyroProposals;
    return true;
}

bool CalibrationAutonomyController::buildGyroTemperatureProposal(uint32_t nowMs) {
    if (!deps_.config || sessionCount_ < 10u ||
        !deps_.config->data.gyroCal.tempCompEnabled) {
        return false;
    }

    struct Fit {
        bool valid = false;
        uint8_t count = 0u;
        float minTemp = 0.0f;
        float maxTemp = 0.0f;
        float referenceTemp = 0.0f;
        Vec3 referenceBias = Vec3::zero();
        Vec3 slope = Vec3::zero();
    };

    auto fitPartition = [&](uint8_t parity, Fit& out) {
        float sumT = 0.0f;
        Vec3 sumY = Vec3::zero();
        bool first = true;
        for (uint8_t i = parity; i < sessionCount_; i = static_cast<uint8_t>(i + 2u)) {
            const Session& session = sessions_[i];
            if (!session.valid) continue;
            if (first) {
                out.minTemp = session.tempC;
                out.maxTemp = session.tempC;
                first = false;
            } else {
                out.minTemp = std::min(out.minTemp, session.tempC);
                out.maxTemp = std::max(out.maxTemp, session.tempC);
            }
            sumT += session.tempC;
            sumY += session.rawGyroMeanRadS;
            ++out.count;
        }
        if (out.count < 4u || first || out.maxTemp - out.minTemp < 3.0f) return;
        out.referenceTemp = sumT / static_cast<float>(out.count);
        out.referenceBias = sumY / static_cast<float>(out.count);
        float denom = 0.0f;
        Vec3 numer = Vec3::zero();
        for (uint8_t i = parity; i < sessionCount_; i = static_cast<uint8_t>(i + 2u)) {
            const Session& session = sessions_[i];
            if (!session.valid) continue;
            const float dt = session.tempC - out.referenceTemp;
            denom += dt * dt;
            numer += (session.rawGyroMeanRadS - out.referenceBias) * dt;
        }
        if (denom < 1.0f) return;
        out.slope = numer / denom;
        out.valid = out.slope.isFinite() && maxAbs(out.slope * kRadToDps) <= 0.10f;
    };

    Fit train{};
    Fit validationCoverage{};
    fitPartition(0u, train);
    fitPartition(1u, validationCoverage);
    if (!train.valid || !validationCoverage.valid) return false;

    float activeTrainSq = 0.0f;
    float candidateTrainSq = 0.0f;
    float activeValidationSq = 0.0f;
    float candidateValidationSq = 0.0f;
    uint8_t trainCount = 0u;
    uint8_t validationCount = 0u;
    Vec3 activeTrainAxis = Vec3::zero();
    Vec3 candidateTrainAxis = Vec3::zero();
    Vec3 activeValidationAxis = Vec3::zero();
    Vec3 candidateValidationAxis = Vec3::zero();

    for (uint8_t i = 0u; i < sessionCount_; ++i) {
        const Session& session = sessions_[i];
        const Vec3 activeResidual = session.rawGyroMeanRadS -
            gyroBiasAt(deps_.config->data, session.tempC);
        const Vec3 candidateBias = train.referenceBias +
            train.slope * (session.tempC - train.referenceTemp);
        const Vec3 candidateResidual = session.rawGyroMeanRadS - candidateBias;
        if ((i & 1u) == 0u) {
            activeTrainSq += activeResidual.normSq();
            candidateTrainSq += candidateResidual.normSq();
            activeTrainAxis += absVec(activeResidual * kRadToDps);
            candidateTrainAxis += absVec(candidateResidual * kRadToDps);
            ++trainCount;
        } else {
            activeValidationSq += activeResidual.normSq();
            candidateValidationSq += candidateResidual.normSq();
            activeValidationAxis += absVec(activeResidual * kRadToDps);
            candidateValidationAxis += absVec(candidateResidual * kRadToDps);
            ++validationCount;
        }
    }
    if (trainCount < 4u || validationCount < 4u) return false;
    activeTrainAxis /= static_cast<float>(trainCount);
    candidateTrainAxis /= static_cast<float>(trainCount);
    activeValidationAxis /= static_cast<float>(validationCount);
    candidateValidationAxis /= static_cast<float>(validationCount);
    if (!componentNonRegression(candidateTrainAxis, activeTrainAxis, 0.006f) ||
        !componentNonRegression(candidateValidationAxis, activeValidationAxis, 0.006f)) {
        return false;
    }

    const float activeTrain = std::sqrt(activeTrainSq / trainCount) * kRadToDps;
    const float candidateTrain = std::sqrt(candidateTrainSq / trainCount) * kRadToDps;
    const float activeValidation =
        std::sqrt(activeValidationSq / validationCount) * kRadToDps;
    const float candidateValidation =
        std::sqrt(candidateValidationSq / validationCount) * kRadToDps;
    if (candidateTrain + 0.015f >= activeTrain ||
        candidateValidation + 0.015f >= activeValidation ||
        candidateValidation > 0.12f) {
        return false;
    }

    proposalConfig_ = *deps_.config;
    proposalConfig_.data.gyroCal.biasValid = true;
    proposalConfig_.data.gyroCal.biasRadS = train.referenceBias;
    proposalConfig_.data.gyroCal.tempCompValid = true;
    proposalConfig_.data.gyroCal.referenceTempC = train.referenceTemp;
    proposalConfig_.data.gyroCal.tempSlopeRadSPerC = train.slope;
    // Preserve the user's runtime policy. This builder only runs when enabled,
    // and never silently toggles a disabled compensation policy.
    proposalConfig_.data.gyroCal.tempCompEnabled =
        deps_.config->data.gyroCal.tempCompEnabled;
    proposalConfig_.data.gyroTempQuality.tempRangeMinC =
        std::min(train.minTemp, validationCoverage.minTemp);
    proposalConfig_.data.gyroTempQuality.tempRangeMaxC =
        std::max(train.maxTemp, validationCoverage.maxTemp);
    proposalConfig_.data.gyroTempQuality.residualBeforeDps = activeValidation;
    proposalConfig_.data.gyroTempQuality.residualAfterDps = candidateValidation;
    proposalConfig_.data.gyroTempQuality.fitQuality =
        clamp01(1.0f - candidateValidation / 0.20f);
    proposalConfig_.data.gyroCalMeta.tempModelUpdatedUptimeMs = nowMs;
    proposalConfig_.data.gyroCalMeta.tempModelSampleCount =
        static_cast<uint32_t>(sessionCount_) * kWindowSamples;
    proposalConfig_.sanitize();
    proposalConfig_.updateCrc();
    if (!proposalConfig_.data.gyroCal.tempCompValid ||
        !proposalConfig_.data.gyroCal.tempCompEnabled) {
        return false;
    }

    proposalMetadata_ = TrackerCalibrationCandidateMetadata{};
    proposalMetadata_.provenance = TrackerCalibrationProvenance::Background;
    proposalMetadata_.sampleCount =
        static_cast<uint32_t>(sessionCount_) * kWindowSamples;
    proposalMetadata_.independentWindowCount = sessionCount_;
    proposalMetadata_.quality = trackerCalibrationQualityFromConfig(proposalConfig_);
    proposalMetadata_.quality.gyroResidualDps = candidateValidation;
    proposalMetadata_.quality.gyroScore =
        clamp01(1.0f - candidateValidation / 0.30f);
    proposalMetadata_.quality.qualityFlags |=
        tracker_calibration_quality_flags::AUTONOMY_0023 |
        tracker_calibration_quality_flags::GYRO_MEASURED |
        tracker_calibration_quality_flags::SOURCE_MEASURED;
    trackerCalibrationQualityRecomputeOverall(proposalConfig_, proposalMetadata_.quality);
    trackerCalibrationQualitySetProvenance(
        proposalMetadata_.quality, TrackerCalibrationProvenance::Background);
    proposalPending_ = true;
    proposalActiveRevision_ = trackerCalibrationPayloadRevision(*deps_.config);
    proposalSignature_ = trackerMakeSensorSignature(*deps_.config);
    subsystem_ = CalibrationAutonomySubsystem::GyroTemperature;
    proposalCreatedMs_ = nowMs;
    transition(CalibrationAutonomyState::CandidateReady, nowMs);
    ++stats_.temperatureProposals;
    return true;
}

bool CalibrationAutonomyController::buildAccelProposal(uint32_t nowMs) {
    if (!deps_.config || sessionCount_ < 12u) return false;
    accelProposalCalibration_.reset();

    // Keep only compact session indices on the task stack. A previous version
    // copied six complete Session objects into a local validation array; that
    // compiled to 848 bytes on Windows/MSYS2 GCC despite staying below the
    // limit on Linux. Indices preserve the same independent held-out windows
    // without moving hundreds of bytes from stack to permanent static RAM.
    uint8_t bestIndex[6] = {};
    uint8_t secondIndex[6] = {};
    float bestVariance[6] = {};
    float secondVariance[6] = {};
    uint8_t bestMask = 0u;
    uint8_t secondMask = 0u;
    for (uint8_t face = 0u; face < 6u; ++face) {
        bestVariance[face] = 1.0e9f;
        secondVariance[face] = 1.0e9f;
    }

    for (uint8_t i = 0u; i < sessionCount_; ++i) {
        if (sessions_[i].face == Accel6PosCalibration::Face::Invalid) continue;
        const uint8_t face = static_cast<uint8_t>(sessions_[i].face);
        if (face >= 6u) continue;
        const uint8_t faceBit = static_cast<uint8_t>(1u << face);
        const float variance = sessions_[i].accelVarianceG2.x +
                               sessions_[i].accelVarianceG2.y +
                               sessions_[i].accelVarianceG2.z;
        if ((bestMask & faceBit) == 0u || variance < bestVariance[face]) {
            if ((bestMask & faceBit) != 0u) {
                secondIndex[face] = bestIndex[face];
                secondVariance[face] = bestVariance[face];
                secondMask = static_cast<uint8_t>(secondMask | faceBit);
            }
            bestIndex[face] = i;
            bestVariance[face] = variance;
            bestMask = static_cast<uint8_t>(bestMask | faceBit);
        } else if ((secondMask & faceBit) == 0u ||
                   variance < secondVariance[face]) {
            secondIndex[face] = i;
            secondVariance[face] = variance;
            secondMask = static_cast<uint8_t>(secondMask | faceBit);
        }
    }
    if (bestMask != 0x3Fu || secondMask != 0x3Fu) return false;
    for (uint8_t face = 0u; face < 6u; ++face) {
        accelBest_[face] = sessions_[bestIndex[face]];
        if (!accelProposalCalibration_.setFace(
                static_cast<Accel6PosCalibration::Face>(face),
                accelBest_[face].rawAccelMeanG,
                kWindowSamples,
                accelBest_[face].accelVarianceG2)) {
            return false;
        }
    }
    if (!accelProposalCalibration_.compute()) return false;
    const auto& result = accelProposalCalibration_.result();
    if (!result.valid || result.qualityScore < 0.80f) return false;

    float activeSum = 0.0f;
    float candidateSum = 0.0f;
    float activeWorst = 0.0f;
    float candidateWorst = 0.0f;
    for (uint8_t face = 0u; face < 6u; ++face) {
        const auto f = static_cast<Accel6PosCalibration::Face>(face);
        // The second independent window is never used by the fit.
        const Vec3 raw = sessions_[secondIndex[face]].rawAccelMeanG;
        const float activeResidual = faceResidual(accelApply(deps_.config->data, raw), f);
        const float candidateResidual = faceResidual(
            result.scaleMatrix * (raw - result.biasG), f);
        if (candidateResidual > 0.080f ||
            (deps_.config->data.accelCal.valid &&
             candidateResidual > activeResidual + 0.004f)) {
            return false;
        }
        activeSum += activeResidual;
        candidateSum += candidateResidual;
        activeWorst = std::max(activeWorst, activeResidual);
        candidateWorst = std::max(candidateWorst, candidateResidual);
    }
    const float activeMean = activeSum / 6.0f;
    const float candidateMean = candidateSum / 6.0f;
    if (deps_.config->data.accelCal.valid &&
        (candidateMean + 0.004f >= activeMean ||
         candidateWorst > activeWorst + 0.002f)) {
        return false;
    }

    proposalConfig_ = *deps_.config;
    proposalConfig_.data.accelCal.valid = true;
    proposalConfig_.data.accelCal.biasG = result.biasG;
    proposalConfig_.data.accelCal.scale = result.scaleMatrix;
    proposalConfig_.data.accelCalQuality.calibrationUptimeMs = nowMs;
    proposalConfig_.data.accelCalQuality.qualityFlags = result.qualityFlags;
    proposalConfig_.data.accelCalQuality.qualityScore = result.qualityScore;
    proposalConfig_.data.accelCalQuality.maxFaceNormErrorG = candidateWorst;
    proposalConfig_.sanitize();
    proposalConfig_.updateCrc();

    proposalMetadata_ = TrackerCalibrationCandidateMetadata{};
    proposalMetadata_.provenance = TrackerCalibrationProvenance::Background;
    proposalMetadata_.sampleCount = 12u * kWindowSamples;
    proposalMetadata_.independentWindowCount = 12u;
    proposalMetadata_.quality = trackerCalibrationQualityFromConfig(proposalConfig_);
    proposalMetadata_.quality.accelResidualG = candidateWorst;
    proposalMetadata_.quality.accelScore = result.qualityScore;
    proposalMetadata_.quality.qualityFlags |=
        tracker_calibration_quality_flags::AUTONOMY_0023 |
        tracker_calibration_quality_flags::ACCEL_MEASURED |
        tracker_calibration_quality_flags::SOURCE_MEASURED;
    trackerCalibrationQualityRecomputeOverall(proposalConfig_, proposalMetadata_.quality);
    trackerCalibrationQualitySetProvenance(
        proposalMetadata_.quality, TrackerCalibrationProvenance::Background);
    proposalPending_ = true;
    proposalActiveRevision_ = trackerCalibrationPayloadRevision(*deps_.config);
    proposalSignature_ = trackerMakeSensorSignature(*deps_.config);
    subsystem_ = CalibrationAutonomySubsystem::Accelerometer;
    proposalCreatedMs_ = nowMs;
    transition(CalibrationAutonomyState::CandidateReady, nowMs);
    ++stats_.accelProposals;
    return true;
}

bool CalibrationAutonomyController::stagePendingProposal(uint32_t nowMs) {
    if (!proposalPending_ || !deps_.configStore || !deps_.config) return false;
    const uint32_t currentRevision = trackerCalibrationPayloadRevision(*deps_.config);
    const TrackerSensorSignature currentSignature = trackerMakeSensorSignature(*deps_.config);
    if (currentRevision != proposalActiveRevision_ ||
        !trackerSensorSignaturesEqual(currentSignature, proposalSignature_)) {
        ++stats_.proposalsSuppressed;
        lastRejectReason_ = CalibrationAutonomyRejectReason::CandidateStale;
        resetRuntimeEvidence();
        transition(CalibrationAutonomyState::Observing, nowMs);
        return false;
    }
    proposalMetadata_.activeCalibrationRevisionAtCreation = proposalActiveRevision_;
    bool exists = true;
    if (!deps_.configStore->candidateExists(exists)) {
        ++stats_.candidateStageFailures;
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }
    if (exists) {
        ++stats_.proposalsSuppressed;
        transition(CalibrationAutonomyState::SuspendedManualCandidate, nowMs);
        return false;
    }
    const uint32_t startUs = micros();
    const bool staged = deps_.configStore->stageCandidate(
        proposalConfig_, proposalMetadata_, nowMs);
    const uint32_t elapsed = micros() - startUs;
    stats_.lastStorageUs = elapsed;
    stats_.maxStorageUs = std::max(stats_.maxStorageUs, elapsed);
    if (!staged) {
        ++stats_.candidateStageFailures;
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }
    ++stats_.candidatesStaged;
    proposalPending_ = false;
    candidateProbeNotBeforeMs_ = 0u;
    clearSessions();
    transition(CalibrationAutonomyState::PersistPending, nowMs);
    return true;
}

CalibrationAutonomySubsystem CalibrationAutonomyController::classifyCandidate(
    const TrackerCalibrationCandidateRecord& candidate) const {
    if (!deps_.config) return CalibrationAutonomySubsystem::None;
    const TrackerConfigBlob& payload = candidate.payload;
    const bool gyroChanged =
        payload.gyroCal.biasValid != deps_.config->data.gyroCal.biasValid ||
        (payload.gyroCal.biasRadS - deps_.config->data.gyroCal.biasRadS).norm() > 1.0e-8f ||
        payload.gyroCal.tempCompValid != deps_.config->data.gyroCal.tempCompValid ||
        (payload.gyroCal.tempSlopeRadSPerC -
         deps_.config->data.gyroCal.tempSlopeRadSPerC).norm() > 1.0e-9f;
    const bool accelChanged =
        payload.accelCal.valid != deps_.config->data.accelCal.valid ||
        (payload.accelCal.biasG - deps_.config->data.accelCal.biasG).norm() > 1.0e-7f ||
        std::memcmp(&payload.accelCal.scale,
                    &deps_.config->data.accelCal.scale,
                    sizeof(Mat3)) != 0;
    const bool alignmentChanged =
        payload.magCal.axisAlignmentValid !=
            deps_.config->data.magCal.axisAlignmentValid ||
        !magAxisMatricesEquivalent(payload.magCal.magToImu,
                                   deps_.config->data.magCal.magToImu,
                                   0.05f);
    if (alignmentChanged &&
        (candidate.metadata.quality.qualityFlags &
         tracker_calibration_quality_flags::ALIGNMENT_MEASURED) != 0u) {
        return CalibrationAutonomySubsystem::MagToImu;
    }
    if ((candidate.metadata.quality.qualityFlags &
         tracker_calibration_quality_flags::AUTONOMY_0023) == 0u) {
        return CalibrationAutonomySubsystem::None;
    }
    if (accelChanged) return CalibrationAutonomySubsystem::Accelerometer;
    if (gyroChanged) {
        return payload.gyroCal.tempCompValid &&
               (!deps_.config->data.gyroCal.tempCompValid ||
                (payload.gyroCal.tempSlopeRadSPerC -
                 deps_.config->data.gyroCal.tempSlopeRadSPerC).norm() > 1.0e-9f)
            ? CalibrationAutonomySubsystem::GyroTemperature
            : CalibrationAutonomySubsystem::GyroBias;
    }
    return CalibrationAutonomySubsystem::None;
}

bool CalibrationAutonomyController::candidateOwnedByAutonomy(
    const TrackerCalibrationCandidateRecord& candidate) const {
    if (candidate.metadata.provenance != TrackerCalibrationProvenance::Background) return false;
    // After selector commit the candidate payload equals the active payload, so
    // classifyCandidate() legitimately returns None. Ownership must survive
    // promotion and is therefore carried by immutable quality-origin flags.
    const uint32_t flags = candidate.metadata.quality.qualityFlags;
    return (flags & (tracker_calibration_quality_flags::AUTONOMY_0022 |
                     tracker_calibration_quality_flags::AUTONOMY_0023)) != 0u;
}

uint32_t CalibrationAutonomyController::candidateFingerprint(
    const TrackerCalibrationCandidateRecord& candidate) const {
    return trackerCalibrationPayloadRevision(candidate.payload) ^
           candidate.signature.crc32 ^
           (static_cast<uint32_t>(classifyCandidate(candidate)) << 24u);
}

bool CalibrationAutonomyController::rejectionSuppresses(uint32_t fingerprint) const {
    if (!CalibrationAutonomyStore::valid(rejection_) || !deps_.config) return false;
    return rejection_.candidateFingerprint == fingerprint &&
           rejection_.activeCalibrationRevision ==
               trackerCalibrationPayloadRevision(*deps_.config);
}

bool CalibrationAutonomyController::service(uint32_t nowMs) {
    if (!deferredServiceRequired()) return false;

    // Completed 960 Hz observation windows bypass the slow lifecycle cadence,
    // but are finalized here, after FIFO processing, never in the sample hook.
    if (completedWindowPending_) {
        ++stats_.serviceCalls;
        const uint32_t serviceStartUs = micros();
        const WindowAccumulator completed = completedWindow_;
        const uint32_t completedNowMs = completedWindowNowMs_;
        completedWindowPending_ = false;
        completedWindowNowMs_ = 0u;
        completedWindow_.reset();
        finalizeWindow(completed, completedNowMs);
        const uint32_t elapsed = micros() - serviceStartUs;
        stats_.lastServiceUs = elapsed;
        stats_.maxServiceUs = std::max(stats_.maxServiceUs, elapsed);
        ++stats_.serviceWorked;
        return true;
    }

    if (lastServiceMs_ != 0u && nowMs - lastServiceMs_ < kServiceIntervalMs) return false;
    lastServiceMs_ = nowMs;
    ++stats_.serviceCalls;
    const uint32_t serviceStartUs = micros();

    bool worked = false;
    if (state_ == CalibrationAutonomyState::SuspendedStorage) {
        worked = reconcileStorageTransaction(nowMs);
    } else if (manualTransactionActive()) {
        if (CalibrationAutonomyStore::valid(journal_) &&
            (state_ == CalibrationAutonomyState::PromotionPending ||
             state_ == CalibrationAutonomyState::PromotionCommitPending ||
             state_ == CalibrationAutonomyState::PromotedProbation ||
             state_ == CalibrationAutonomyState::AcceptedCleanup ||
             state_ == CalibrationAutonomyState::RollbackPending)) {
            lastRejectReason_ = CalibrationAutonomyRejectReason::UserDisabled;
            transition(CalibrationAutonomyState::RollbackPending, nowMs);
            worked = true;
        } else {
            transition(CalibrationAutonomyState::SuspendedManualCandidate, nowMs);
        }
    } else if (proposalPending_) {
        MagDeferredServiceGate gate;
        if (realtimeGateAllows(gate)) worked = stagePendingProposal(nowMs);
        else {
            ++stats_.serviceDeferrals;
            transition(CalibrationAutonomyState::SuspendedRealtime, nowMs);
        }
    } else if (state_ == CalibrationAutonomyState::PromotionPending) {
        MagDeferredServiceGate gate;
        if (realtimeGateAllows(gate)) worked = advancePromotion(nowMs);
        else ++stats_.serviceDeferrals;
    } else if (state_ == CalibrationAutonomyState::PromotionCommitPending) {
        MagDeferredServiceGate gate;
        if (realtimeGateAllows(gate)) worked = enterProbationAfterPromotion(nowMs);
        else ++stats_.serviceDeferrals;
    } else if (state_ == CalibrationAutonomyState::RollbackPending) {
        MagDeferredServiceGate gate;
        if (realtimeGateAllows(gate)) worked = rollbackPromotion(nowMs, lastRejectReason_);
        else ++stats_.serviceDeferrals;
    } else if (state_ == CalibrationAutonomyState::AcceptedCleanup) {
        MagDeferredServiceGate gate;
        if (realtimeGateAllows(gate)) worked = acceptPromotion(nowMs);
        else ++stats_.serviceDeferrals;
    } else if (state_ == CalibrationAutonomyState::PromotedProbation) {
        if (probationHealthFailed()) {
            // Network/FIFO/magnetic disturbances are not evidence that the new
            // calibration is worse. Pause and discard the contaminated interval.
            snapshotProbationHealth();
            probationStartedMs_ = nowMs;
            window_.reset();
            ++stats_.serviceDeferrals;
        } else if (nowMs - probationStartedMs_ > kProbationMaxMs) {
            lastRejectReason_ = CalibrationAutonomyRejectReason::RealtimeFault;
            transition(CalibrationAutonomyState::RollbackPending, nowMs);
            worked = true;
        } else if (probationCanAccept(nowMs)) {
            transition(CalibrationAutonomyState::AcceptedCleanup, nowMs);
            worked = true;
        }
    } else if (wave0023Enabled_ || wave0022Enabled_) {
        MagDeferredServiceGate gate;
        if (realtimeGateAllows(gate)) worked = serviceCandidateLifecycle(nowMs);
        else if (state_ == CalibrationAutonomyState::PersistPending ||
                 state_ == CalibrationAutonomyState::CandidateReady) {
            ++stats_.serviceDeferrals;
            transition(CalibrationAutonomyState::SuspendedRealtime, nowMs);
        }
    }

    const uint32_t elapsed = micros() - serviceStartUs;
    stats_.lastServiceUs = elapsed;
    stats_.maxServiceUs = std::max(stats_.maxServiceUs, elapsed);
    if (worked) ++stats_.serviceWorked;
    return worked;
}

bool CalibrationAutonomyController::serviceCandidateLifecycle(uint32_t nowMs) {
    if (!deps_.configStore) return false;
    if (candidateProbeNotBeforeMs_ != 0u && nowMs < candidateProbeNotBeforeMs_) {
        return false;
    }
    if (!deps_.configStore->loadCandidate(candidateRecord_)) {
        if (deps_.configStore->lastError() == TrackerConfigError::NotFound) {
            candidateProbeNotBeforeMs_ = nowMs + kCandidateAbsentProbeIntervalMs;
            if (state_ != CalibrationAutonomyState::Observing &&
                state_ != CalibrationAutonomyState::RejectedCooldown) {
                transition(CalibrationAutonomyState::Observing, nowMs);
            }
            return false;
        }
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }
    candidateProbeNotBeforeMs_ = 0u;
    subsystem_ = classifyCandidate(candidateRecord_);
    if (!candidateOwnedByAutonomy(candidateRecord_)) {
        transition(CalibrationAutonomyState::SuspendedManualCandidate, nowMs);
        return false;
    }
    const uint32_t fingerprint = candidateFingerprint(candidateRecord_);
    if (rejectionSuppresses(fingerprint)) {
        ++stats_.proposalsSuppressed;
        lastRejectReason_ = CalibrationAutonomyRejectReason::RejectedFingerprint;
        // The slot is owned by autonomy and has already been rejected for the
        // current active revision. Remove it immediately so a stale background
        // proposal cannot monopolize the shared manual/setup candidate slot.
        (void)deps_.configStore->discardCandidate();
        transition(CalibrationAutonomyState::RejectedCooldown, nowMs);
        return true;
    }

    if (deps_.configStore->candidateDirty()) {
        const uint32_t startUs = micros();
        const bool flushed = deps_.configStore->flushCandidate(nowMs, false);
        const uint32_t elapsed = micros() - startUs;
        stats_.lastStorageUs = elapsed;
        stats_.maxStorageUs = std::max(stats_.maxStorageUs, elapsed);
        if (!flushed) {
            ++stats_.candidateFlushFailures;
            candidateProbeNotBeforeMs_ = nowMs + kCandidateRetryIntervalMs;
            transition(CalibrationAutonomyState::PersistPending, nowMs);
            return false;
        }
        ++stats_.candidateFlushes;
        // Candidate is now durable, but no promotion has been prepared yet.
        // Leave the transaction state ordinary so the next deferred tick can
        // compare and prepare it before PromotionPending is entered.
        transition(CalibrationAutonomyState::CandidateReady, nowMs);
        return true;
    }

    TrackerCalibrationComparisonResult comparison;
    uint32_t flags = 0u;
    if (!deps_.configStore->compareCandidate(candidateRecord_, comparison, flags)) {
        if (comparison == TrackerCalibrationComparisonResult::StaleActiveGeneration) {
            lastRejectReason_ = CalibrationAutonomyRejectReason::CandidateStale;
        } else {
            lastRejectReason_ = CalibrationAutonomyRejectReason::CandidateNotBetter;
        }
        (void)recordRejection(nowMs, lastRejectReason_);
        (void)deps_.configStore->discardCandidate();
        transition(CalibrationAutonomyState::RejectedCooldown, nowMs);
        return true;
    }
    return beginPromotion(nowMs);
}

bool CalibrationAutonomyController::beginPromotion(uint32_t nowMs) {
    if (!deps_.config || !deps_.configStore || !deps_.autonomyStore) return false;
    previousConfig_ = *deps_.config;
    prepared_ = TrackerPreparedConfigPromotion{};
    if (!deps_.configStore->prepareCandidatePromotion(
            prepared_, candidateConfig_, false, deps_.config)) {
        ++stats_.promotionFailures;
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }
    ++stats_.promotionsPrepared;
    // Keep each deferred tick bounded to one persistent transaction stage.
    // The next tick writes the durable journal; a later tick switches selector.
    journal_ = CalibrationAutonomyJournalRecord{};
    transition(CalibrationAutonomyState::PromotionPending, nowMs);
    return true;
}

bool CalibrationAutonomyController::advancePromotion(uint32_t nowMs) {
    if (!deps_.config || !deps_.configStore || !deps_.autonomyStore ||
        !prepared_.valid) {
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }

    if (!CalibrationAutonomyStore::valid(journal_)) {
        journal_ = CalibrationAutonomyJournalRecord{};
        journal_.state = CalibrationAutonomyJournalState::PromotionPending;
        journal_.subsystem = subsystem_;
        journal_.previousSlot = prepared_.previousSelector.activeSlot;
        journal_.previousGeneration = prepared_.previousSelector.activeGeneration;
        journal_.previousCalibrationRevision =
            trackerCalibrationPayloadRevision(previousConfig_);
        journal_.targetCalibrationRevision =
            trackerCalibrationPayloadRevision(candidateConfig_);
        journal_.candidateFingerprint = candidateFingerprint(candidateRecord_);
        journal_.transactionStartedUptimeMs = nowMs;
        journal_.previousPayload = previousConfig_.data;
        if (!deps_.autonomyStore->writeJournal(journal_)) {
            deps_.configStore->abortPreparedPromotion(prepared_);
            ++stats_.promotionFailures;
            transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
            return false;
        }
        return true;
    }

    deps_.configStore->setAutonomyProbationWriteBarrier(true);
    if (!deps_.configStore->commitPreparedPromotion(prepared_, promotedConfig_)) {
        const TrackerConfigError commitError = deps_.configStore->lastError();
        ++stats_.promotionFailures;
        lastRejectReason_ = CalibrationAutonomyRejectReason::StorageFailure;

        if (commitError == TrackerConfigError::CommitUncertain) {
            if (deps_.configStore->load(promotedConfig_)) {
                const bool candidateCommitted =
                    std::memcmp(&promotedConfig_.data,
                                &candidateConfig_.data,
                                sizeof(promotedConfig_.data)) == 0;
                const bool previousStillActive =
                    std::memcmp(&promotedConfig_.data,
                                &previousConfig_.data,
                                sizeof(promotedConfig_.data)) == 0;
                if (deps_.callbacks.applyCalibrationConfig) {
                    deps_.callbacks.applyCalibrationConfig(
                        promotedConfig_, deps_.callbacks.applyCalibrationConfigUser);
                }
                deps_.configStore->confirmAuthoritativeConfigApplied();
                if (candidateCommitted) {
                    prepared_.valid = false;
                    ++stats_.promotionsCommitted;
                    transition(CalibrationAutonomyState::PromotionCommitPending, nowMs);
                    return true;
                }
                if (previousStillActive) {
                    deps_.configStore->abortPreparedPromotion(prepared_);
                    (void)recordRejection(nowMs, lastRejectReason_);
                    (void)deps_.configStore->discardCandidate();
                    (void)deps_.autonomyStore->clearJournal();
                    deps_.configStore->setAutonomyProbationWriteBarrier(false);
                    transition(CalibrationAutonomyState::RejectedCooldown, nowMs);
                    return false;
                }
            }
            transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
            return false;
        }

        deps_.configStore->abortPreparedPromotion(prepared_);
        (void)recordRejection(nowMs, lastRejectReason_);
        (void)deps_.configStore->discardCandidate();
        (void)deps_.autonomyStore->clearJournal();
        deps_.configStore->setAutonomyProbationWriteBarrier(false);
        transition(CalibrationAutonomyState::RejectedCooldown, nowMs);
        return false;
    }

    if (deps_.callbacks.applyCalibrationConfig) {
        deps_.callbacks.applyCalibrationConfig(
            promotedConfig_, deps_.callbacks.applyCalibrationConfigUser);
    }
    // Runtime application is complete; release the store's apply-pending
    // guard while retaining the stronger probation rollback barrier.
    deps_.configStore->confirmAuthoritativeConfigApplied();
    ++stats_.promotionsCommitted;
    transition(CalibrationAutonomyState::PromotionCommitPending, nowMs);
    return true;
}

bool CalibrationAutonomyController::enterProbationAfterPromotion(uint32_t nowMs) {
    probationStartedMs_ = nowMs;
    probationAcceptedWindows_ = 0u;
    probationAccelFaceMask_ = 0u;
    probationCandidateResidualSum_ = 0.0f;
    probationPreviousResidualSum_ = 0.0f;
    snapshotProbationHealth();
    clearSessions();
    motionSeenSinceSession_ = true;
    if (deps_.magAxisState) deps_.magAxisState->activeAlignmentConfirmed = false;
    journal_.state = CalibrationAutonomyJournalState::Probation;
    journal_.probationAcceptedWindows = 0u;
    if (!deps_.autonomyStore->writeJournal(journal_)) {
        lastRejectReason_ = CalibrationAutonomyRejectReason::StorageFailure;
        transition(CalibrationAutonomyState::RollbackPending, nowMs);
        return false;
    }
    deps_.configStore->setAutonomyProbationWriteBarrier(true);
    transition(CalibrationAutonomyState::PromotedProbation, nowMs);
    return true;
}

void CalibrationAutonomyController::snapshotProbationHealth() {
    if (deps_.quality) probationQualityBaseline_ = deps_.quality->counters();
    if (deps_.slimevr) deps_.slimevr->healthCounters(probationSlimeBaseline_);
}

bool CalibrationAutonomyController::probationHealthFailed() const {
    if (deps_.quality) {
        const auto& now = deps_.quality->counters();
        if (now.fifoOverrunEvents != probationQualityBaseline_.fifoOverrunEvents ||
            now.fifoFullEvents != probationQualityBaseline_.fifoFullEvents ||
            now.fifoRecoveryRequests != probationQualityBaseline_.fifoRecoveryRequests ||
            now.timestampBackwards != probationQualityBaseline_.timestampBackwards ||
            now.timestampQueueOverflows != probationQualityBaseline_.timestampQueueOverflows ||
            now.completedSampleQueueOverflows !=
                probationQualityBaseline_.completedSampleQueueOverflows) {
            return true;
        }
    }
    if (deps_.slimevr) {
        SlimeVROutputHealthCounters now;
        deps_.slimevr->healthCounters(now);
        if (now.sendFailures != probationSlimeBaseline_.sendFailures ||
            now.rotationSendFailures != probationSlimeBaseline_.rotationSendFailures ||
            now.serverSilenceResets != probationSlimeBaseline_.serverSilenceResets ||
            now.wifiLostResets != probationSlimeBaseline_.wifiLostResets) {
            return true;
        }
    }
    if (subsystem_ == CalibrationAutonomySubsystem::MagToImu && deps_.magReliability) {
        if (deps_.magReliability->state == MagFieldReliabilityState::Disturbed ||
            deps_.magReliability->state == MagFieldReliabilityState::Suspect) {
            return true;
        }
    }
    return false;
}

float CalibrationAutonomyController::gyroModelResidualDps(
    const TrackerConfigBlob& payload, const Session& session) {
    return (session.rawGyroMeanRadS - gyroBiasAt(payload, session.tempC)).norm() * kRadToDps;
}

float CalibrationAutonomyController::accelModelResidualG(
    const TrackerConfigBlob& payload, const Session& session) {
    if (session.face == Accel6PosCalibration::Face::Invalid) return 999.0f;
    return faceResidual(accelApply(payload, session.rawAccelMeanG), session.face);
}

bool CalibrationAutonomyController::evaluateProbationWindow(
    const Session& session, uint32_t nowMs) {
    if (state_ != CalibrationAutonomyState::PromotedProbation || !deps_.config) return false;
    float candidateResidual = 0.0f;
    float previousResidual = 0.0f;
    if (subsystem_ == CalibrationAutonomySubsystem::GyroBias ||
        subsystem_ == CalibrationAutonomySubsystem::GyroTemperature) {
        candidateResidual = gyroModelResidualDps(deps_.config->data, session);
        previousResidual = gyroModelResidualDps(journal_.previousPayload, session);
        if (candidateResidual > previousResidual + 0.010f || candidateResidual > 0.15f) {
            ++stats_.probationRegressions;
            lastRejectReason_ = CalibrationAutonomyRejectReason::FreshEvidenceRegression;
            transition(CalibrationAutonomyState::RollbackPending, nowMs);
            return false;
        }
    } else if (subsystem_ == CalibrationAutonomySubsystem::Accelerometer) {
        if (session.face == Accel6PosCalibration::Face::Invalid) return false;
        candidateResidual = accelModelResidualG(deps_.config->data, session);
        previousResidual = accelModelResidualG(journal_.previousPayload, session);
        if (candidateResidual > previousResidual + 0.004f || candidateResidual > 0.10f) {
            ++stats_.probationRegressions;
            lastRejectReason_ = CalibrationAutonomyRejectReason::FreshEvidenceRegression;
            transition(CalibrationAutonomyState::RollbackPending, nowMs);
            return false;
        }
        const uint8_t faceIndex = static_cast<uint8_t>(session.face);
        if (faceIndex < 6u) probationAccelFaceMask_ |= static_cast<uint8_t>(1u << faceIndex);
    } else {
        return false;
    }
    probationCandidateResidualSum_ += candidateResidual;
    probationPreviousResidualSum_ += previousResidual;
    ++probationAcceptedWindows_;
    ++stats_.probationWindows;
    journal_.probationAcceptedWindows = probationAcceptedWindows_;
    // Journal progress is checkpointed sparsely: every third independent window.
    if ((probationAcceptedWindows_ % 3u) == 0u && deps_.autonomyStore) {
        (void)deps_.autonomyStore->writeJournal(journal_);
    }
    return true;
}

bool CalibrationAutonomyController::probationCanAccept(uint32_t nowMs) const {
    if (nowMs - probationStartedMs_ < kProbationMinMs) return false;
    if (subsystem_ == CalibrationAutonomySubsystem::MagToImu) {
        return deps_.magAxisState && deps_.magAxisState->activeAlignmentConfirmed &&
               deps_.magReliability && deps_.magReliability->trustedForYaw;
    }
    if (probationAcceptedWindows_ < 3u) return false;
    if (subsystem_ == CalibrationAutonomySubsystem::Accelerometer &&
        probationAccelFaceMask_ != 0x3Fu) {
        return false;
    }
    return probationCandidateResidualSum_ <= probationPreviousResidualSum_ + 0.005f;
}

bool CalibrationAutonomyController::writeJournal(
    CalibrationAutonomyJournalState state, uint32_t nowMs) {
    if (!deps_.autonomyStore) return false;
    journal_.state = state;
    journal_.transactionStartedUptimeMs = nowMs;
    journal_.probationAcceptedWindows = probationAcceptedWindows_;
    return deps_.autonomyStore->writeJournal(journal_);
}

bool CalibrationAutonomyController::acceptPromotion(uint32_t nowMs) {
    if (!deps_.configStore || !deps_.autonomyStore ||
        !CalibrationAutonomyStore::valid(journal_)) return false;

    // Persist the decision first. Subsequent calls are idempotent and perform
    // at most one destructive persistent cleanup stage per service tick.
    if (journal_.state != CalibrationAutonomyJournalState::AcceptPending) {
        if (!writeJournal(CalibrationAutonomyJournalState::AcceptPending, nowMs)) {
            transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
            return false;
        }
        return true;
    }

    candidateRecord_ = TrackerCalibrationCandidateRecord{};
    if (deps_.configStore->loadCandidate(candidateRecord_)) {
        if (!candidateOwnedByAutonomy(candidateRecord_)) {
            // Never remove a manual/setup candidate that appeared while
            // recovering an old autonomous transaction.
            transition(CalibrationAutonomyState::SuspendedManualCandidate, nowMs);
            return false;
        }
        if (!deps_.configStore->discardCandidate()) {
            transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
            return false;
        }
        return true;
    }
    if (deps_.configStore->lastError() != TrackerConfigError::NotFound) {
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }

    if (!deps_.autonomyStore->clearJournal()) {
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }
    journal_ = CalibrationAutonomyJournalRecord{};
    deps_.configStore->setAutonomyProbationWriteBarrier(false);
    ++stats_.accepts;
    lastRejectReason_ = CalibrationAutonomyRejectReason::None;
    subsystem_ = CalibrationAutonomySubsystem::None;
    resetRuntimeEvidence();
    transition(CalibrationAutonomyState::Observing, nowMs);
    return true;
}

bool CalibrationAutonomyController::recordRejection(
    uint32_t nowMs, CalibrationAutonomyRejectReason reason) {
    if (!deps_.autonomyStore || !deps_.config) return false;
    rejection_ = CalibrationAutonomyRejectionRecord{};
    rejection_.subsystem = subsystem_;
    rejection_.reason = static_cast<uint8_t>(reason);
    rejection_.candidateFingerprint = journal_.candidateFingerprint != 0u
        ? journal_.candidateFingerprint
        : candidateFingerprint(candidateRecord_);
    rejection_.activeCalibrationRevision = trackerCalibrationPayloadRevision(*deps_.config);
    rejection_.rejectedUptimeMs = nowMs;
    return deps_.autonomyStore->writeRejection(rejection_);
}

bool CalibrationAutonomyController::rollbackPromotion(
    uint32_t nowMs, CalibrationAutonomyRejectReason reason) {
    if (!deps_.config || !deps_.configStore || !deps_.autonomyStore ||
        !CalibrationAutonomyStore::valid(journal_)) {
        return false;
    }
    deps_.configStore->setAutonomyProbationWriteBarrier(true);

    // Write-ahead rollback intent is its own persistent stage. A reboot after
    // this point deterministically resumes the same rollback path.
    if (journal_.state != CalibrationAutonomyJournalState::RollbackPending) {
        if (!writeJournal(CalibrationAutonomyJournalState::RollbackPending, nowMs)) {
            ++stats_.rollbackFailures;
            transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
            return false;
        }
        return true;
    }

    const uint32_t activeRevision = trackerCalibrationPayloadRevision(*deps_.config);
    if (activeRevision != journal_.previousCalibrationRevision) {
        if (!deps_.configStore->restoreAuthoritativeGeneration(
                journal_.previousSlot, journal_.previousGeneration, rollbackConfig_)) {
            ++stats_.rollbackFailures;
            transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
            return false;
        }
        if (trackerCalibrationPayloadRevision(rollbackConfig_) !=
            journal_.previousCalibrationRevision) {
            ++stats_.rollbackFailures;
            transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
            return false;
        }
        if (deps_.callbacks.applyCalibrationConfig) {
            deps_.callbacks.applyCalibrationConfig(
                rollbackConfig_, deps_.callbacks.applyCalibrationConfigUser);
        }
        deps_.configStore->confirmAuthoritativeConfigApplied();
        return true;
    }

    candidateRecord_ = TrackerCalibrationCandidateRecord{};
    if (deps_.configStore->loadCandidate(candidateRecord_)) {
        if (candidateOwnedByAutonomy(candidateRecord_)) {
            if (!deps_.configStore->discardCandidate()) {
                ++stats_.rollbackFailures;
                transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
                return false;
            }
            return true;
        }
        // A newer manual/setup candidate is not part of this rollback and must
        // remain untouched.
    } else if (deps_.configStore->lastError() != TrackerConfigError::NotFound) {
        ++stats_.rollbackFailures;
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }

    lastRejectReason_ = reason;
    const bool rejectionAlreadyRecorded =
        CalibrationAutonomyStore::valid(rejection_) &&
        rejection_.candidateFingerprint == journal_.candidateFingerprint &&
        rejection_.activeCalibrationRevision == journal_.previousCalibrationRevision;
    if (!rejectionAlreadyRecorded) {
        if (!recordRejection(nowMs, reason)) {
            ++stats_.rollbackFailures;
            transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
            return false;
        }
        return true;
    }

    if (!deps_.autonomyStore->clearJournal()) {
        ++stats_.rollbackFailures;
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }
    journal_ = CalibrationAutonomyJournalRecord{};
    deps_.configStore->setAutonomyProbationWriteBarrier(false);
    ++stats_.rollbacks;
    subsystem_ = CalibrationAutonomySubsystem::None;
    resetRuntimeEvidence();
    transition(CalibrationAutonomyState::RejectedCooldown, nowMs);
    return true;
}

bool CalibrationAutonomyController::completeRollbackSynchronously(
    uint32_t nowMs, CalibrationAutonomyRejectReason reason) {
    // Manual/setup entry is already a blocking explicit operation. Resolve the
    // bounded idempotent phases now so no provisional model or write barrier is
    // left behind when the command starts mutating calibration state.
    for (uint8_t phase = 0u;
         phase < 8u && CalibrationAutonomyStore::valid(journal_);
         ++phase) {
        if (!rollbackPromotion(nowMs, reason)) return false;
    }
    return !CalibrationAutonomyStore::valid(journal_) &&
           state_ != CalibrationAutonomyState::SuspendedStorage;
}

bool CalibrationAutonomyController::recoverJournalAtBoot(uint32_t nowMs) {
    if (!deps_.autonomyStore || !deps_.configStore || !deps_.config) return false;
    if (!deps_.autonomyStore->loadJournal(journal_)) {
        if (deps_.autonomyStore->lastErrorIsNotFound()) {
            deps_.configStore->setAutonomyProbationWriteBarrier(false);
            return false;
        }
        // 0023a expanded the journal with exact slot/generation rollback
        // anchors. A valid v1 write-ahead record is conservatively rolled back
        // using its calibration payload, then removed. Never accept the target
        // merely because the firmware was upgraded between transaction phases.
        CalibrationAutonomyJournalRecordV1 legacy{};
        if (deps_.autonomyStore->loadLegacyJournalV1(legacy)) {
            return recoverLegacyJournalV1AtBoot(legacy, nowMs);
        }
        wave0022Enabled_ = false;
        wave0023Enabled_ = false;
        if (deps_.callbacks.setWave0022RuntimeEnabled) {
            deps_.callbacks.setWave0022RuntimeEnabled(
                false, deps_.callbacks.setWave0022RuntimeEnabledUser);
        }
        deps_.configStore->setAutonomyProbationWriteBarrier(true);
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }

    subsystem_ = journal_.subsystem;
    const uint32_t activeRevision = trackerCalibrationPayloadRevision(*deps_.config);
    deps_.configStore->setAutonomyProbationWriteBarrier(true);

    if (journal_.state == CalibrationAutonomyJournalState::PromotionPending) {
        if (activeRevision == journal_.previousCalibrationRevision) {
            if (!deps_.autonomyStore->clearJournal()) {
                transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
                return false;
            }
            deps_.configStore->setAutonomyProbationWriteBarrier(false);
            transition(CalibrationAutonomyState::Observing, nowMs);
            return true;
        }
        if (activeRevision == journal_.targetCalibrationRevision) {
            probationStartedMs_ = nowMs;
            snapshotProbationHealth();
            journal_.state = CalibrationAutonomyJournalState::Probation;
            if (!deps_.autonomyStore->writeJournal(journal_)) {
                lastRejectReason_ = CalibrationAutonomyRejectReason::StorageFailure;
                transition(CalibrationAutonomyState::RollbackPending, nowMs);
                return false;
            }
            transition(CalibrationAutonomyState::PromotedProbation, nowMs);
            return true;
        }
        lastRejectReason_ = CalibrationAutonomyRejectReason::StorageFailure;
        transition(CalibrationAutonomyState::RollbackPending, nowMs);
        return true;
    }
    if (journal_.state == CalibrationAutonomyJournalState::Probation) {
        if (activeRevision == journal_.targetCalibrationRevision) {
            probationStartedMs_ = nowMs;
            probationAcceptedWindows_ = journal_.probationAcceptedWindows;
            snapshotProbationHealth();
            transition(CalibrationAutonomyState::PromotedProbation, nowMs);
            return true;
        }
        lastRejectReason_ = CalibrationAutonomyRejectReason::StorageFailure;
        transition(CalibrationAutonomyState::RollbackPending, nowMs);
        return true;
    }
    if (journal_.state == CalibrationAutonomyJournalState::AcceptPending) {
        if (activeRevision != journal_.targetCalibrationRevision) {
            lastRejectReason_ = CalibrationAutonomyRejectReason::StorageFailure;
            transition(CalibrationAutonomyState::RollbackPending, nowMs);
            return true;
        }
        // Cleanup is idempotent and split across later service ticks. Do not
        // combine candidate removal and journal removal during boot recovery.
        transition(CalibrationAutonomyState::AcceptedCleanup, nowMs);
        return true;
    }
    if (journal_.state == CalibrationAutonomyJournalState::RollbackPending) {
        lastRejectReason_ = CalibrationAutonomyRejectReason::StorageFailure;
        transition(CalibrationAutonomyState::RollbackPending, nowMs);
        return true;
    }
    transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
    return false;
}

bool CalibrationAutonomyController::recoverPendingEraseAtBoot(
    const CalibrationAutonomyEraseRecoveryRecord& recovery, uint32_t nowMs) {
    if (!deps_.config || !deps_.configStore || !deps_.autonomyStore) return false;

    TrackerConfig clean;
    clean.data = recovery.cleanPayload;
    clean.clearAllCalibrationPreservingPolicy();
    clean.sanitize();
    clean.updateCrc();

    deps_.configStore->setAutonomyProbationWriteBarrier(false);
    if (!deps_.configStore->erase() ||
        !deps_.configStore->save(clean, TrackerCalibrationProvenance::Manual)) {
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }
    *deps_.config = clean;
    if (deps_.callbacks.applyCalibrationConfig) {
        deps_.callbacks.applyCalibrationConfig(
            *deps_.config, deps_.callbacks.applyCalibrationConfigUser);
    }
    deps_.configStore->confirmAuthoritativeConfigApplied();

    wave0022Enabled_ = recovery.wave0022Enabled != 0u;
    wave0023Enabled_ = recovery.wave0023Enabled != 0u;
    if (deps_.callbacks.setWave0022RuntimeEnabled) {
        deps_.callbacks.setWave0022RuntimeEnabled(
            wave0022Enabled_, deps_.callbacks.setWave0022RuntimeEnabledUser);
    }

    if (!deps_.autonomyStore->clearJournal() ||
        !deps_.autonomyStore->clearRejection() ||
        !deps_.autonomyStore->savePreferences(wave0022Enabled_, wave0023Enabled_) ||
        !deps_.autonomyStore->clearEraseRecovery()) {
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }

    journal_ = CalibrationAutonomyJournalRecord{};
    rejection_ = CalibrationAutonomyRejectionRecord{};
    prepared_ = TrackerPreparedConfigPromotion{};
    subsystem_ = CalibrationAutonomySubsystem::None;
    lastRejectReason_ = CalibrationAutonomyRejectReason::None;
    resetRuntimeEvidence();
    transition(CalibrationAutonomyState::Observing, nowMs);
    return true;
}

bool CalibrationAutonomyController::recoverLegacyJournalV1AtBoot(
    const CalibrationAutonomyJournalRecordV1& legacy, uint32_t nowMs) {
    if (!deps_.config || !deps_.configStore || !deps_.autonomyStore) return false;
    deps_.configStore->setAutonomyProbationWriteBarrier(false);

    if (legacy.state != CalibrationAutonomyJournalState::Empty) {
        previousConfig_ = TrackerConfig{};
        previousConfig_.data = legacy.previousPayload;
        previousConfig_.sanitize();
        previousConfig_.updateCrc();

        rollbackConfig_ = *deps_.config;
        trackerApplyCalibrationCandidateToConfig(rollbackConfig_, previousConfig_);
        if (!deps_.configStore->save(
                rollbackConfig_, TrackerCalibrationProvenance::Background)) {
            deps_.configStore->setAutonomyProbationWriteBarrier(true);
            transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
            return false;
        }
        *deps_.config = rollbackConfig_;
        if (deps_.callbacks.applyCalibrationConfig) {
            deps_.callbacks.applyCalibrationConfig(
                *deps_.config, deps_.callbacks.applyCalibrationConfigUser);
        }
        deps_.configStore->confirmAuthoritativeConfigApplied();
    }

    // A v1 candidate belongs to the same obsolete transaction. Remove only a
    // background candidate; manual/setup work remains operator-owned.
    if (deps_.configStore->loadCandidate(candidateRecord_)) {
        if (candidateOwnedByAutonomy(candidateRecord_) &&
            !deps_.configStore->discardCandidate()) {
            transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
            return false;
        }
    } else if (deps_.configStore->lastError() != TrackerConfigError::NotFound) {
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }
    if (!deps_.autonomyStore->clearJournal()) {
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }
    journal_ = CalibrationAutonomyJournalRecord{};
    subsystem_ = CalibrationAutonomySubsystem::None;
    lastRejectReason_ = CalibrationAutonomyRejectReason::StorageFailure;
    resetRuntimeEvidence();
    transition(CalibrationAutonomyState::RejectedCooldown, nowMs);
    return true;
}

bool CalibrationAutonomyController::reconcileStorageTransaction(uint32_t nowMs) {
    if (!deps_.autonomyStore || !deps_.configStore) return false;
    CalibrationAutonomyJournalRecord loaded{};
    if (!deps_.autonomyStore->loadJournal(loaded)) {
        if (deps_.autonomyStore->lastErrorIsNotFound()) {
            deps_.configStore->setAutonomyProbationWriteBarrier(false);
            transition(CalibrationAutonomyState::Observing, nowMs);
            return true;
        }
        CalibrationAutonomyJournalRecordV1 legacy{};
        if (deps_.autonomyStore->loadLegacyJournalV1(legacy)) {
            return recoverLegacyJournalV1AtBoot(legacy, nowMs);
        }
        return false;
    }
    journal_ = loaded;
    subsystem_ = journal_.subsystem;
    if (journal_.state == CalibrationAutonomyJournalState::RollbackPending) {
        return rollbackPromotion(nowMs, CalibrationAutonomyRejectReason::StorageFailure);
    }
    return recoverJournalAtBoot(nowMs);
}

bool CalibrationAutonomyController::beginManualCalibration(uint32_t nowMs) {
    if (!begun_) return false;
    if (manualCalibrationDepth_ != 0u) {
        ++manualCalibrationDepth_;
        return true;
    }
    // Resolve any provisional model before the blocking setup/manual command can
    // mutate RAM or write a checkpoint.
    if (prepared_.valid && !CalibrationAutonomyStore::valid(journal_)) {
        deps_.configStore->abortPreparedPromotion(prepared_);
    }
    if (CalibrationAutonomyStore::valid(journal_)) {
        if (!completeRollbackSynchronously(
                nowMs, CalibrationAutonomyRejectReason::UserDisabled)) {
            return false;
        }
    } else if (state_ == CalibrationAutonomyState::SuspendedStorage) {
        return false;
    }
    if (deps_.configStore && deps_.configStore->loadCandidate(candidateRecord_)) {
        if (candidateOwnedByAutonomy(candidateRecord_)) {
            if (!deps_.configStore->discardCandidate()) return false;
        }
    } else if (deps_.configStore &&
               deps_.configStore->lastError() != TrackerConfigError::NotFound) {
        return false;
    }
    resetRuntimeEvidence();
    if (deps_.callbacks.resetWave0022Evidence) {
        deps_.callbacks.resetWave0022Evidence(
            deps_.callbacks.resetWave0022EvidenceUser);
    }
    manualCalibrationDepth_ = 1u;
    transition(CalibrationAutonomyState::SuspendedManualCandidate, nowMs);
    return true;
}

void CalibrationAutonomyController::endManualCalibration(bool calibrationChanged,
                                                          uint32_t nowMs) {
    if (manualCalibrationDepth_ == 0u) return;
    --manualCalibrationDepth_;
    if (manualCalibrationDepth_ != 0u) return;
    // Even an aborted command may have applied/checkpointed a calibration. Never
    // let evidence collected under the old model survive the ownership hand-off.
    resetRuntimeEvidence();
    if (calibrationChanged && deps_.callbacks.resetWave0022Evidence) {
        deps_.callbacks.resetWave0022Evidence(
            deps_.callbacks.resetWave0022EvidenceUser);
    }
    transition(CalibrationAutonomyState::Observing, nowMs);
}

bool CalibrationAutonomyController::setWave0022Enabled(
    bool enabled, bool persist, uint32_t nowMs) {
    if (!begun_) return false;
    if (!enabled && subsystem_ == CalibrationAutonomySubsystem::MagToImu &&
        CalibrationAutonomyStore::valid(journal_) &&
        (state_ == CalibrationAutonomyState::PromotionPending ||
         state_ == CalibrationAutonomyState::PromotionCommitPending ||
         state_ == CalibrationAutonomyState::PromotedProbation ||
         state_ == CalibrationAutonomyState::AcceptedCleanup ||
         state_ == CalibrationAutonomyState::RollbackPending ||
         state_ == CalibrationAutonomyState::SuspendedStorage)) {
        if (!completeRollbackSynchronously(
                nowMs, CalibrationAutonomyRejectReason::UserDisabled)) {
            return false;
        }
    }
    wave0022Enabled_ = enabled;
    if (deps_.callbacks.setWave0022RuntimeEnabled) {
        deps_.callbacks.setWave0022RuntimeEnabled(
            enabled, deps_.callbacks.setWave0022RuntimeEnabledUser);
    }
    if (!enabled) {
        if (deps_.callbacks.resetWave0022Evidence) {
            deps_.callbacks.resetWave0022Evidence(
                deps_.callbacks.resetWave0022EvidenceUser);
        }
        if (deps_.configStore->loadCandidate(candidateRecord_) &&
            candidateRecord_.metadata.provenance == TrackerCalibrationProvenance::Background &&
            classifyCandidate(candidateRecord_) == CalibrationAutonomySubsystem::MagToImu) {
            (void)deps_.configStore->discardCandidate();
        }
    }
    if (persist && !deps_.autonomyStore->savePreferences(
            wave0022Enabled_, wave0023Enabled_)) {
        return false;
    }
    return true;
}

bool CalibrationAutonomyController::setWave0023Enabled(
    bool enabled, bool persist, uint32_t nowMs) {
    if (!begun_) return false;
    if (!enabled && CalibrationAutonomyStore::valid(journal_) &&
        (state_ == CalibrationAutonomyState::PromotionPending ||
         state_ == CalibrationAutonomyState::PromotionCommitPending ||
         state_ == CalibrationAutonomyState::PromotedProbation ||
         state_ == CalibrationAutonomyState::AcceptedCleanup ||
         state_ == CalibrationAutonomyState::RollbackPending ||
         state_ == CalibrationAutonomyState::SuspendedStorage)) {
        if (!completeRollbackSynchronously(
                nowMs, CalibrationAutonomyRejectReason::UserDisabled)) {
            return false;
        }
    }
    wave0023Enabled_ = enabled;
    if (!enabled) {
        resetRuntimeEvidence();
        if (deps_.configStore->loadCandidate(candidateRecord_) &&
            candidateOwnedByAutonomy(candidateRecord_)) {
            (void)deps_.configStore->discardCandidate();
        }
        transition(CalibrationAutonomyState::Observing, nowMs);
    }
    if (persist && !deps_.autonomyStore->savePreferences(
            wave0022Enabled_, wave0023Enabled_)) {
        return false;
    }
    return true;
}

bool CalibrationAutonomyController::requestRollback(
    uint32_t nowMs, CalibrationAutonomyRejectReason reason) {
    if (!CalibrationAutonomyStore::valid(journal_) ||
        (state_ != CalibrationAutonomyState::PromotionPending &&
         state_ != CalibrationAutonomyState::PromotionCommitPending &&
         state_ != CalibrationAutonomyState::PromotedProbation &&
         state_ != CalibrationAutonomyState::AcceptedCleanup &&
         state_ != CalibrationAutonomyState::RollbackPending &&
         state_ != CalibrationAutonomyState::SuspendedStorage)) {
        return false;
    }
    lastRejectReason_ = reason;
    transition(CalibrationAutonomyState::RollbackPending, nowMs);
    return rollbackPromotion(nowMs, reason);
}

bool CalibrationAutonomyController::clearRejectionMemory() {
    rejection_ = CalibrationAutonomyRejectionRecord{};
    return deps_.autonomyStore && deps_.autonomyStore->clearRejection();
}

bool CalibrationAutonomyController::blocksMotionLightSleep() const {
    // Light sleep preserves RAM and resumes in-place. Keep the tracker awake
    // only while calibration state can still advance, mutate storage or needs
    // fresh probation evidence. A passive fail-closed storage suspension with
    // no valid transaction journal is not work in progress and must not drain
    // the battery indefinitely. The damaged/obsolete record remains untouched
    // and autonomous promotion stays disabled until explicit recovery.
    if (!begun_) return false;
    if (manualCalibrationDepth_ != 0u ||
        CalibrationAutonomyStore::valid(journal_)) return true;
    return proposalPending_ ||
        state_ == CalibrationAutonomyState::CandidateReady ||
        state_ == CalibrationAutonomyState::PersistPending ||
        state_ == CalibrationAutonomyState::PromotionPending ||
        state_ == CalibrationAutonomyState::PromotionCommitPending ||
        state_ == CalibrationAutonomyState::PromotedProbation ||
        state_ == CalibrationAutonomyState::AcceptedCleanup ||
        state_ == CalibrationAutonomyState::RollbackPending;
}

bool CalibrationAutonomyController::clearPersistentCalibrationState() {
    if (!begun_ || manualCalibrationDepth_ == 0u || !deps_.autonomyStore) return false;
    if (!deps_.autonomyStore->clearJournal()) return false;
    if (!deps_.autonomyStore->clearRejection()) return false;
    journal_ = CalibrationAutonomyJournalRecord{};
    rejection_ = CalibrationAutonomyRejectionRecord{};
    subsystem_ = CalibrationAutonomySubsystem::None;
    lastRejectReason_ = CalibrationAutonomyRejectReason::None;
    candidateProbeNotBeforeMs_ = 0u;
    resetRuntimeEvidence();
    return true;
}

bool CalibrationAutonomyController::preparePersistentCalibrationErase(
    const TrackerConfig& cleanConfig) {
    if (!begun_ || !deps_.autonomyStore || !deps_.configStore) return false;
    deps_.configStore->setAutonomyProbationWriteBarrier(false);
    return deps_.autonomyStore->writeEraseRecovery(
        cleanConfig, wave0022Enabled_, wave0023Enabled_);
}

bool CalibrationAutonomyController::forceClearPersistentCalibrationStateForErase(
    uint32_t nowMs) {
    if (!begun_ || !deps_.autonomyStore || !deps_.configStore) return false;
    if (prepared_.valid) deps_.configStore->abortPreparedPromotion(prepared_);
    deps_.configStore->setAutonomyProbationWriteBarrier(false);

    // Do not parse or trust a damaged/obsolete journal here. The caller has
    // already installed a calibrationless authoritative config, so raw removal
    // is the only safe and intended recovery action.
    const bool journalCleared = deps_.autonomyStore->clearJournal();
    const bool rejectionCleared = deps_.autonomyStore->clearRejection();
    const bool preferencesNormalized = deps_.autonomyStore->savePreferences(
        wave0022Enabled_, wave0023Enabled_);
    const bool eraseMarkerCleared = journalCleared && rejectionCleared &&
        preferencesNormalized && deps_.autonomyStore->clearEraseRecovery();
    if (!eraseMarkerCleared) {
        transition(CalibrationAutonomyState::SuspendedStorage, nowMs);
        return false;
    }

    journal_ = CalibrationAutonomyJournalRecord{};
    rejection_ = CalibrationAutonomyRejectionRecord{};
    prepared_ = TrackerPreparedConfigPromotion{};
    subsystem_ = CalibrationAutonomySubsystem::None;
    lastRejectReason_ = CalibrationAutonomyRejectReason::None;
    candidateProbeNotBeforeMs_ = 0u;
    manualCalibrationDepth_ = 0u;
    resetRuntimeEvidence();
    transition(CalibrationAutonomyState::Observing, nowMs);
    return true;
}

const char* CalibrationAutonomyController::stateName(CalibrationAutonomyState state) {
    switch (state) {
        case CalibrationAutonomyState::Observing: return "observing";
        case CalibrationAutonomyState::CandidateReady: return "candidate_ready";
        case CalibrationAutonomyState::PersistPending: return "persist_pending";
        case CalibrationAutonomyState::PromotionPending: return "promotion_pending";
        case CalibrationAutonomyState::PromotionCommitPending: return "promotion_commit_pending";
        case CalibrationAutonomyState::PromotedProbation: return "promoted_probation";
        case CalibrationAutonomyState::AcceptedCleanup: return "accepted_cleanup";
        case CalibrationAutonomyState::RollbackPending: return "rollback_pending";
        case CalibrationAutonomyState::RejectedCooldown: return "rejected_cooldown";
        case CalibrationAutonomyState::SuspendedManualCandidate: return "suspended_manual_candidate";
        case CalibrationAutonomyState::SuspendedRealtime: return "suspended_realtime";
        case CalibrationAutonomyState::SuspendedStorage: return "suspended_storage";
    }
    return "unknown";
}

const char* CalibrationAutonomyController::rejectReasonName(
    CalibrationAutonomyRejectReason reason) {
    switch (reason) {
        case CalibrationAutonomyRejectReason::None: return "none";
        case CalibrationAutonomyRejectReason::CandidateNotBetter: return "candidate_not_better";
        case CalibrationAutonomyRejectReason::CandidateStale: return "candidate_stale";
        case CalibrationAutonomyRejectReason::RealtimeFault: return "realtime_fault";
        case CalibrationAutonomyRejectReason::FreshEvidenceRegression: return "fresh_evidence_regression";
        case CalibrationAutonomyRejectReason::MagneticProbationFailed: return "magnetic_probation_failed";
        case CalibrationAutonomyRejectReason::StorageFailure: return "storage_failure";
        case CalibrationAutonomyRejectReason::UserDisabled: return "user_disabled";
        case CalibrationAutonomyRejectReason::RejectedFingerprint: return "rejected_fingerprint";
    }
    return "unknown";
}

void CalibrationAutonomyController::printStatus(Stream& out, uint32_t nowMs) const {
    out.println("# CALIBRATION AUTONOMY");
    out.print("autonomy_0022_enabled="); out.println(wave0022Enabled_ ? "yes" : "no");
    out.print("autonomy_0023_enabled="); out.println(wave0023Enabled_ ? "yes" : "no");
    out.print("autonomy_imu_hotpath_enabled=");
    out.println(imuObservationRequired() ? "yes" : "no");
    out.print("autonomy_deferred_service_required=");
    out.println(deferredServiceRequired() ? "yes" : "no");
    out.print("autonomy_state="); out.println(stateName(state_));
    out.print("autonomy_storage_error=");
    out.println(deps_.autonomyStore ? deps_.autonomyStore->lastErrorName() : "unavailable");
    out.print("autonomy_motion_sleep_blocked=");
    out.println(blocksMotionLightSleep() ? "yes" : "no");
    out.print("autonomy_motion_sleep_block_reason=");
    if (!begun_) out.println("none");
    else if (manualCalibrationDepth_ != 0u) out.println("manual_calibration");
    else if (CalibrationAutonomyStore::valid(journal_)) out.println("durable_transaction");
    else if (proposalPending_) out.println("proposal_pending");
    else if (state_ == CalibrationAutonomyState::CandidateReady ||
             state_ == CalibrationAutonomyState::PersistPending) {
        out.println("candidate_persistence");
    } else if (state_ == CalibrationAutonomyState::PromotionPending ||
               state_ == CalibrationAutonomyState::PromotionCommitPending) {
        out.println("promotion");
    } else if (state_ == CalibrationAutonomyState::PromotedProbation) {
        out.println("probation");
    } else if (state_ == CalibrationAutonomyState::AcceptedCleanup) {
        out.println("accept_cleanup");
    } else if (state_ == CalibrationAutonomyState::RollbackPending) {
        out.println("rollback");
    } else {
        out.println("none");
    }
    out.print("autonomy_subsystem="); out.println(calibrationAutonomySubsystemName(subsystem_));
    out.print("autonomy_state_age_ms="); out.println(nowMs - stateChangedMs_);
    out.print("autonomy_last_reject_reason="); out.println(rejectReasonName(lastRejectReason_));
    out.print("autonomy_stationary_windows="); out.println(stats_.stationaryWindows);
    out.print("autonomy_observation_windows_deferred="); out.println(stats_.observationWindowsDeferred);
    out.print("autonomy_observation_window_pending="); out.println(completedWindowPending_ ? "yes" : "no");
    out.print("autonomy_observation_window_drops="); out.println(stats_.observationWindowDrops);
    out.print("autonomy_independent_sessions="); out.println(stats_.independentSessions);
    out.print("autonomy_sessions_buffered="); out.println(sessionCount_);
    out.print("autonomy_proposal_pending="); out.println(proposalPending_ ? "yes" : "no");
    out.print("autonomy_probation_windows="); out.println(probationAcceptedWindows_);
    out.print("autonomy_probation_age_ms=");
    out.println(state_ == CalibrationAutonomyState::PromotedProbation
                    ? nowMs - probationStartedMs_ : 0u);
    out.print("autonomy_candidates_staged="); out.println(stats_.candidatesStaged);
    out.print("autonomy_promotions_committed="); out.println(stats_.promotionsCommitted);
    out.print("autonomy_accepts="); out.println(stats_.accepts);
    out.print("autonomy_rollbacks="); out.println(stats_.rollbacks);
    out.print("autonomy_probation_regressions="); out.println(stats_.probationRegressions);
    out.print("autonomy_service_calls="); out.println(stats_.serviceCalls);
    out.print("autonomy_service_worked="); out.println(stats_.serviceWorked);
    out.print("autonomy_service_last_us="); out.println(stats_.lastServiceUs);
    out.print("autonomy_service_max_us="); out.println(stats_.maxServiceUs);
    out.print("autonomy_storage_last_us="); out.println(stats_.lastStorageUs);
    out.print("autonomy_storage_max_us="); out.println(stats_.maxStorageUs);
    out.print("autonomy_service_deferrals="); out.println(stats_.serviceDeferrals);
}

} // namespace tracker

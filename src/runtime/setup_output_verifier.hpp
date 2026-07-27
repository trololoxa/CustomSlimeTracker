#pragma once

#include <cstdint>

#include "runtime/tracker_runtime_types.hpp"

namespace tracker {

struct SetupOutputVerificationConfig {
    uint32_t minimumSnapshots = 80;
    uint32_t minimumInputSamples = 256;
    float maximumQuaternionNormError = 0.010f;
    float maximumQuaternionStepDeg = 15.0f;
    float maximumLinearAccelerationMeanG = 0.080f;
    float maximumLinearAccelerationRmsG = 0.120f;
    float maximumLinearAccelerationPeakG = 0.300f;
    float minimumLinearAccelerationValidRatio = 0.80f;
    float maximumInputGyroMeanNormDps = 0.20f;
    float maximumInputGyroMeanStdErrorAxisDps = 0.050f;
    float maximumInputAccelNormMeanErrorG = 0.050f;
    float maximumInputAccelNormStdG = 0.025f;
};

struct SetupOutputVerificationResult {
    bool valid = false;
    bool outputAvailable = false;
    bool quaternionFinite = true;
    bool quaternionNormPassed = false;
    bool quaternionContinuityPassed = false;
    bool linearAccelerationPassed = false;
    bool stationaryInputPassed = false;
    bool stationaryInputSampleCountPassed = false;
    bool stationaryGyroMeanPassed = false;
    bool stationaryGyroPrecisionPassed = false;
    bool stationaryAccelMeanPassed = false;
    bool stationaryAccelStdPassed = false;
    bool streamHealthPassed = false;

    uint32_t snapshots = 0;
    uint32_t uniqueSnapshots = 0;
    uint32_t linearAccelerationValidSnapshots = 0;
    uint32_t invalidSnapshots = 0;
    uint32_t duplicateSnapshots = 0;
    uint32_t inputSamples = 0;

    float maximumQuaternionNormError = 0.0f;
    float maximumQuaternionStepDeg = 0.0f;
    float linearAccelerationMeanG = 0.0f;
    float linearAccelerationRmsG = 0.0f;
    float linearAccelerationPeakG = 0.0f;
    float linearAccelerationValidRatio = 0.0f;
    Vec3 inputGyroMeanDps = Vec3::zero();
    Vec3 inputGyroMeanStdErrorDps = Vec3::zero();
    float inputAccelNormMeanG = 0.0f;
    float inputAccelNormStdG = 0.0f;
};

// Accumulates already-produced coherent output snapshots. It intentionally
// verifies the final output boundary instead of re-running calibration math.
// This catches frame, AHRS, gravity-removal and snapshot-coherency failures
// using the same quaternion/acceleration pair that the network path consumes.
class SetupOutputVerificationAccumulator {
public:
    void reset();
    void push(const TrackerPreparedOutputSnapshot& snapshot);
    void pushInputSample(const Vec3& calibratedGyroRadS,
                         const Vec3& calibratedAccelG);

    uint32_t uniqueSnapshotCount() const { return uniqueSnapshots_; }
    uint32_t inputSampleCount() const { return inputSamples_; }

    SetupOutputVerificationResult finish(
        const SetupOutputVerificationConfig& config,
        bool streamHealthPassed) const;

private:
    uint32_t snapshots_ = 0;
    uint32_t uniqueSnapshots_ = 0;
    uint32_t linearAccelerationValidSnapshots_ = 0;
    uint32_t invalidSnapshots_ = 0;
    uint32_t duplicateSnapshots_ = 0;
    uint32_t lastSequence_ = 0;
    bool haveSequence_ = false;
    bool haveQuaternion_ = false;
    bool quaternionFinite_ = true;
    Quat previousQuaternion_ = Quat::identity();
    double linearAccelerationNormSum_ = 0.0;
    double linearAccelerationNormSqSum_ = 0.0;
    float maximumLinearAccelerationNorm_ = 0.0f;
    float maximumQuaternionNormError_ = 0.0f;
    float maximumQuaternionStepDeg_ = 0.0f;

    uint32_t inputSamples_ = 0;
    double gyroSum_[3] = {};
    double gyroSqSum_[3] = {};
    double accelNormSum_ = 0.0;
    double accelNormSqSum_ = 0.0;
};

} // namespace tracker

#pragma once

#include <cstdint>

#include "core/math.hpp"
#include "runtime/static_test_types.hpp"

namespace tracker {

struct RuntimeGyroBiasEstimator {
    bool enabled = false;
    bool requireAccelCalibration = true;

    // Temperature compensation range is a trust signal, not a hard kill switch.
    // Inside the calibrated range the estimator runs normally. Just outside
    // the range it may still run with a smaller gain if the tracker is strongly
    // stationary and temperature is stable. Far outside the range is rejected
    // while requireTempCompRange remains enabled.
    bool requireTempCompRange = true;
    bool allowOutOfRangeEstimator = true;
    bool dryRun = false;

    // Conservative defaults: a window is ~4.4 s at 932 Hz. The first
    // stationary window only primes the detector; updates start after the
    // second consecutive stationary window to avoid learning during slow motion.
    uint32_t windowSamplesRequired = 4096;
    uint8_t stationaryWindowsBeforeUpdate = 2;
    float gyroMeanMaxDps = 0.08f;
    float gyroStdNormMaxDps = 0.16f;
    float gyroStdAxisMaxDps = 0.12f;
    float accelNormMeanMaxErrG = 0.015f;
    float accelNormStdMaxG = 0.006f;
    float accelTrustMin = 0.92f;
    float maxWindowTempDeltaC = 0.35f;
    float tempExtrapolationMarginC = 6.0f;
    float outOfRangeGainScale = 0.35f;
    float updateAlpha = 0.05f;
    float maxUpdateStepDps = 0.0015f;
    float maxRuntimeTrimDps = 0.08f;

    Vec3Stats calibratedGyroRadS;
    ScalarStats accelNormG;
    ScalarStats accelTrust;
    ScalarStats tempC;

    uint32_t windows = 0;
    uint32_t stationaryWindows = 0;
    uint32_t primingWindows = 0;
    uint32_t accepted = 0;
    uint32_t rejected = 0;
    uint32_t badTimingRejects = 0;
    uint32_t motionRejects = 0;
    uint32_t accelRejects = 0;
    uint32_t saturationRejects = 0;
    uint32_t tempRejects = 0;
    uint32_t tempCautiousSamples = 0;
    uint32_t tempCautiousWindows = 0;
    uint32_t tempFarRejects = 0;
    uint32_t calibrationRejects = 0;
    uint32_t finiteRejects = 0;
    uint32_t dryRunUpdates = 0;
    uint32_t updates = 0;
    uint8_t consecutiveStationaryWindows = 0;

    Vec3 runtimeTrimRadS = Vec3::zero();
    Vec3 lastResidualDps = Vec3::zero();
    Vec3 lastGyroStdDps = Vec3::zero();
    Vec3 lastAppliedDeltaDps = Vec3::zero();
    float lastAccelNormMeanG = 0.0f;
    float lastAccelNormStdG = 0.0f;
    float lastAccelTrustMean = 0.0f;
    float lastTempC = 0.0f;
    float lastTempSpanC = 0.0f;
    float lastTempDistanceToRangeC = 0.0f;
    float lastUpdateGainScale = 1.0f;
    uint32_t lastDecisionFlags = 0;
    uint32_t lastUpdateMs = 0;

    void resetWindow() {
        calibratedGyroRadS.reset();
        accelNormG.reset();
        accelTrust.reset();
        tempC.reset();
    }

    void resetCounters() {
        windows = stationaryWindows = primingWindows = 0;
        accepted = rejected = 0;
        badTimingRejects = motionRejects = accelRejects = saturationRejects = 0;
        tempRejects = tempCautiousSamples = tempCautiousWindows = tempFarRejects = 0;
        calibrationRejects = finiteRejects = dryRunUpdates = 0;
        updates = 0;
        consecutiveStationaryWindows = 0;
        lastResidualDps = Vec3::zero();
        lastGyroStdDps = Vec3::zero();
        lastAppliedDeltaDps = Vec3::zero();
        lastAccelNormMeanG = 0.0f;
        lastAccelNormStdG = 0.0f;
        lastAccelTrustMean = 0.0f;
        lastTempC = 0.0f;
        lastTempSpanC = 0.0f;
        lastTempDistanceToRangeC = 0.0f;
        lastUpdateGainScale = 1.0f;
        lastDecisionFlags = 0;
        lastUpdateMs = 0;
        resetWindow();
    }

    void resetAll() {
        enabled = false;
        dryRun = false;
        runtimeTrimRadS = Vec3::zero();
        resetCounters();
    }
};

} // namespace tracker

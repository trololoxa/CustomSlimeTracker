#include "test_common.hpp"

#include <Arduino.h>
#include <cmath>

#include "config/tracker_config.hpp"
#include "runtime/mag_runtime_controller.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/mag_axis_alignment.hpp"
#include "sensor/mag_field_reliability.hpp"
#include "sensor/mag_heading.hpp"
#include "sensor/mag_runtime.hpp"
#include "sensor/mag_yaw_correction.hpp"

Stream Serial;
ESPClass ESP;

using namespace tracker;

namespace {

struct DeferredGateState {
    bool allow = true;
    uint32_t calls = 0;
};

bool evaluateDeferredGate(MagDeferredServiceGate& gate, void* user) {
    auto& state = *static_cast<DeferredGateState*>(user);
    ++state.calls;
    gate = MagDeferredServiceGate{};
    gate.allowed = state.allow;
    if (!state.allow) {
        gate.rejectFlags = MAG_DEFERRED_REJECT_SOFTWARE_FIFO_PENDING;
    }
    return state.allow;
}

Lsm6dsvFifoReader::MagRawSample makeMag(uint32_t seq, uint64_t timestampUs, float angleRad) {
    Lsm6dsvFifoReader::MagRawSample sample;
    sample.seq = seq;
    sample.t_us = timestampUs;
    sample.x = static_cast<int16_t>(std::lround(100.0f * std::cos(angleRad)));
    sample.y = static_cast<int16_t>(std::lround(100.0f * std::sin(angleRad)));
    sample.z = 10;
    return sample;
}

void testAxisEvidenceIsDeferredWithoutChangingSequence(TestContext& ctx) {
    TrackerConfig config;
    config.resetDefaults();
    config.data.magCal.driverEnabled = true;
    config.data.magCal.calibrationValid = true;
    config.data.magCal.axisAlignmentValid = false;
    config.data.magCal.hardIron = Vec3::zero();
    config.data.magCal.softIron = Mat3::diagonal(0.01f, 0.01f, 0.01f);
    config.data.magCal.expectedFieldNorm = 1.0f;
    config.data.magCal.minTrustNorm = 0.25f;
    config.data.magCal.maxTrustNorm = 2.5f;
    config.data.magYaw.controllerEnabled = false;
    config.data.magYaw.applyEnabled = false;

    TrackerConfigStore store;
    MagRuntimeState state;
    MagRuntimeProcessor processor;
    MagAxisAlignmentCollector deferredCollector;
    MagAxisAlignmentCollector directCollector;
    MagAxisAlignmentRuntimeState axisState;
    TrackerConfig candidateWorkspace;
    MagHeadingEstimator headingEstimator;
    MagFieldReliabilityMonitor fieldReliability;
    MagYawCorrectionController yawCorrection;
    Ahrs6Dof ahrs;
    ahrs.reset(Quat::identity(), 1000000u);

    MagProcessedSample lastProcessed;
    MagHeadingSample lastHeading;
    MagFieldReliabilityOutput lastReliability;
    MagYawCorrectionOutput lastYaw;
    Lsm6dsv::Sample lastCalibrated;
    lastCalibrated.gyro_rad_s = Vec3(0.0f, 0.0f, 1.0f);
    uint64_t lastImuTimestampUs = 1000000u;
    uint32_t lastImuSequence = 1u;
    bool accelReady = false;
    uint64_t fallbackTimestampUs = 1000000u;
    float outputConfidence = 1.0f;
    MagHeadingReferenceState headingRef;
    MagHeadingAutoReferenceState autoRef;
    Stream out;
    DeferredGateState deferredGate;

    MagRuntimeControllerDeps deps;
    deps.out = &out;
    deps.config = &config;
    deps.configStore = &store;
    deps.ahrs = &ahrs;
    deps.state = &state;
    deps.processor = &processor;
    deps.headingEstimator = &headingEstimator;
    deps.fieldReliability = &fieldReliability;
    deps.axisAlignmentCollector = &deferredCollector;
    deps.axisAlignmentState = &axisState;
    deps.axisAlignmentCandidateWorkspace = &candidateWorkspace;
    deps.headingRef = &headingRef;
    deps.headingAutoRef = &autoRef;
    deps.yawCorrection = &yawCorrection;
    deps.lastProcessed = &lastProcessed;
    deps.lastHeading = &lastHeading;
    deps.lastFieldReliability = &lastReliability;
    deps.lastYawCorrection = &lastYaw;
    deps.lastOutputConfidence = &outputConfidence;
    deps.lastCalibratedSample = &lastCalibrated;
    deps.lastImuTimestampUs = &lastImuTimestampUs;
    deps.lastImuSampleSequence = &lastImuSequence;
    deps.accelCalibrationReady = &accelReady;
    deps.fallbackTimestampUs = &fallbackTimestampUs;
    deps.callbacks.evaluateDeferredServiceGate = evaluateDeferredGate;
    deps.callbacks.evaluateDeferredServiceGateUser = &deferredGate;

    MagRuntimeController controller;
    controller.begin(deps);

    for (uint32_t i = 0; i < 4u; ++i) {
        const uint64_t timestampUs = 1000000u + static_cast<uint64_t>(i) * 100000u;
        lastImuTimestampUs = timestampUs;
        lastImuSequence = i + 1u;
        const auto sample = makeMag(i + 1u, timestampUs, static_cast<float>(i) * 0.10f);

        controller.processRawSample(sample);
        const MagProcessedSample copied = lastProcessed;
        CHECK(ctx, axisState.evidenceQueued == i + 1u);
        CHECK(ctx, axisState.evidenceProcessed == i);
        CHECK(ctx, deferredCollector.stats().magSamplesSeen == i);

        directCollector.observe(copied.gyroSensorRadS,
                                copied.gyroTimestampUs,
                                copied,
                                true);
        if (i == 0u) {
            deferredGate.allow = false;
            CHECK(ctx, !controller.serviceDeferred());
            CHECK(ctx, axisState.evidenceProcessed == 0u);
            CHECK(ctx, axisState.evidenceServiceDeferrals == 1u);
            CHECK(ctx, axisState.serviceDeferralSoftwareFifo == 1u);
            deferredGate.allow = true;
        }
        CHECK(ctx, controller.serviceDeferred());
        CHECK(ctx, axisState.evidenceProcessed == i + 1u);
        CHECK(ctx, deferredCollector.stats().magSamplesSeen == i + 1u);
        CHECK(ctx, deferredCollector.intervalCount() == directCollector.intervalCount());
        CHECK(ctx, deferredCollector.stats().intervalsAccepted ==
                   directCollector.stats().intervalsAccepted);
        CHECK(ctx, deferredCollector.stats().intervalsRejectedTiming ==
                   directCollector.stats().intervalsRejectedTiming);
        CHECK(ctx, deferredCollector.stats().intervalsRejectedMotion ==
                   directCollector.stats().intervalsRejectedMotion);
    }

    CHECK(ctx, axisState.evidenceDropped == 0u);
    CHECK(ctx, axisState.evidenceServiceDeferrals == 1u);
    CHECK(ctx, deferredGate.calls == 5u);
    CHECK(ctx, axisState.evidenceQueueHighWater == 1u);
    CHECK(ctx, !controller.serviceDeferred());

}

void testAxisEvidenceQueueIsBoundedAndFailClosed(TestContext& ctx) {
    TrackerConfig config;
    config.resetDefaults();
    config.data.magCal.driverEnabled = true;
    config.data.magCal.calibrationValid = true;
    config.data.magCal.axisAlignmentValid = false;
    config.data.magCal.softIron = Mat3::diagonal(0.01f, 0.01f, 0.01f);

    TrackerConfigStore store;
    MagRuntimeState state;
    MagRuntimeProcessor processor;
    MagAxisAlignmentCollector collector;
    MagAxisAlignmentRuntimeState axisState;
    TrackerConfig candidateWorkspace;
    MagHeadingEstimator headingEstimator;
    MagFieldReliabilityMonitor fieldReliability;
    MagYawCorrectionController yawCorrection;
    Ahrs6Dof ahrs;
    ahrs.reset(Quat::identity(), 1000000u);
    MagProcessedSample lastProcessed;
    MagHeadingSample lastHeading;
    MagFieldReliabilityOutput lastReliability;
    MagYawCorrectionOutput lastYaw;
    Lsm6dsv::Sample lastCalibrated;
    lastCalibrated.gyro_rad_s = Vec3(0.0f, 0.0f, 1.0f);
    uint64_t lastImuTimestampUs = 1000000u;
    uint32_t lastImuSequence = 1u;
    bool accelReady = false;
    uint64_t fallbackTimestampUs = 1000000u;
    MagHeadingReferenceState headingRef;
    MagHeadingAutoReferenceState autoRef;
    Stream out;

    MagRuntimeControllerDeps deps;
    deps.out = &out;
    deps.config = &config;
    deps.configStore = &store;
    deps.ahrs = &ahrs;
    deps.state = &state;
    deps.processor = &processor;
    deps.headingEstimator = &headingEstimator;
    deps.fieldReliability = &fieldReliability;
    deps.axisAlignmentCollector = &collector;
    deps.axisAlignmentState = &axisState;
    deps.axisAlignmentCandidateWorkspace = &candidateWorkspace;
    deps.headingRef = &headingRef;
    deps.headingAutoRef = &autoRef;
    deps.yawCorrection = &yawCorrection;
    deps.lastProcessed = &lastProcessed;
    deps.lastHeading = &lastHeading;
    deps.lastFieldReliability = &lastReliability;
    deps.lastYawCorrection = &lastYaw;
    deps.lastCalibratedSample = &lastCalibrated;
    deps.lastImuTimestampUs = &lastImuTimestampUs;
    deps.lastImuSampleSequence = &lastImuSequence;
    deps.accelCalibrationReady = &accelReady;
    deps.fallbackTimestampUs = &fallbackTimestampUs;

    MagRuntimeController controller;
    controller.begin(deps);
    for (uint32_t i = 0; i < 11u; ++i) {
        const uint64_t timestampUs = 1000000u + static_cast<uint64_t>(i) * 100000u;
        lastImuTimestampUs = timestampUs;
        lastImuSequence = i + 1u;
        controller.processRawSample(makeMag(i + 1u, timestampUs, static_cast<float>(i) * 0.10f));
    }
    CHECK(ctx, axisState.evidenceQueued == 8u);
    CHECK(ctx, axisState.evidenceDropped == 3u);
    CHECK(ctx, axisState.evidenceQueueHighWater == 8u);

    // A config/model epoch change must not mix queued observations from the
    // old calibrated frame into the new axis-alignment dataset.
    config.data.magCal.expectedFieldNorm += 0.1f;
    config.updateCrc();
    for (uint32_t i = 0; i < 8u; ++i) CHECK(ctx, controller.serviceDeferred());
    CHECK(ctx, axisState.evidenceProcessed == 0u);
    CHECK(ctx, axisState.evidenceStaleDropped == 8u);
    CHECK(ctx, collector.stats().magSamplesSeen == 0u);
    CHECK(ctx, !controller.serviceDeferred());
}

} // namespace

int main() {
    TestContext ctx;
    testAxisEvidenceIsDeferredWithoutChangingSequence(ctx);
    testAxisEvidenceQueueIsBoundedAndFailClosed(ctx);
    return ctx.finish("policy_pre_0024a_mag_deferred");
}

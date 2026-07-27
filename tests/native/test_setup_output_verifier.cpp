#include <cassert>
#include <cmath>

#include "runtime/setup_output_verifier.hpp"

using namespace tracker;

static TrackerPreparedOutputSnapshot makeSnapshot(uint32_t sequence,
                                                  const Quat& q,
                                                  const Vec3& linearG,
                                                  bool accelValid = true) {
    TrackerPreparedOutputSnapshot s;
    s.valid = true;
    s.sequence = sequence;
    s.q = q;
    s.linearAccelerationValid = accelValid;
    s.linearAccelerationDeviceG = linearG;
    return s;
}

static void pushStableInput(SetupOutputVerificationAccumulator& verifier,
                            uint32_t count,
                            float gyroMeanXDps = 0.0f) {
    for (uint32_t i = 0; i < count; ++i) {
        const float nx = (i & 1u) ? 0.58f : -0.58f;
        const float ny = (i & 2u) ? 0.30f : -0.30f;
        const float nz = (i & 4u) ? 0.07f : -0.07f;
        verifier.pushInputSample(
            Vec3(gyroMeanXDps + nx, ny, nz) * MATH_DEG_TO_RAD,
            Vec3(0.0f, 0.0f, 1.0f + ((i & 1u) ? 0.004f : -0.004f)));
    }
}

int main() {
    SetupOutputVerificationConfig cfg;
    cfg.minimumSnapshots = 80;

    {
        SetupOutputVerificationAccumulator verifier;
        for (uint32_t i = 0; i < 100; ++i) {
            const float yaw = 0.0002f * static_cast<float>(i);
            verifier.push(makeSnapshot(
                i + 1,
                Quat::fromEulerXYZ(0.0f, 0.0f, yaw),
                Vec3(0.008f, -0.004f, 0.006f)));
        }
        pushStableInput(verifier, 1024);
        const SetupOutputVerificationResult r = verifier.finish(cfg, true);
        assert(r.valid);
        assert(r.uniqueSnapshots == 100u);
        assert(r.linearAccelerationValidRatio == 1.0f);
        assert(r.maximumQuaternionNormError < 1.0e-5f);
        assert(r.linearAccelerationRmsG < 0.02f);
    }

    {
        SetupOutputVerificationAccumulator verifier;
        for (uint32_t i = 0; i < 100; ++i) {
            verifier.push(makeSnapshot(
                i + 1,
                Quat::identity(),
                Vec3(0.18f, 0.0f, 0.0f)));
        }
        pushStableInput(verifier, 1024);
        const SetupOutputVerificationResult r = verifier.finish(cfg, true);
        assert(!r.valid);
        assert(!r.linearAccelerationPassed);
    }

    {
        SetupOutputVerificationAccumulator verifier;
        for (uint32_t i = 0; i < 100; ++i) {
            const Quat q = (i == 50)
                ? Quat::fromEulerXYZ(0.0f, 0.0f, 45.0f * MATH_DEG_TO_RAD)
                : Quat::identity();
            verifier.push(makeSnapshot(i + 1, q, Vec3::zero()));
        }
        pushStableInput(verifier, 1024);
        const SetupOutputVerificationResult r = verifier.finish(cfg, true);
        assert(!r.valid);
        assert(!r.quaternionContinuityPassed);
    }

    {
        SetupOutputVerificationAccumulator verifier;
        for (uint32_t i = 0; i < 100; ++i) {
            verifier.push(makeSnapshot(i + 1, Quat::identity(), Vec3::zero(), i < 60));
        }
        pushStableInput(verifier, 1024);
        const SetupOutputVerificationResult r = verifier.finish(cfg, true);
        assert(!r.valid);
        assert(!r.linearAccelerationPassed);
    }

    {
        SetupOutputVerificationAccumulator verifier;
        for (uint32_t i = 0; i < 100; ++i) {
            verifier.push(makeSnapshot(i + 1, Quat::identity(), Vec3::zero()));
        }
        pushStableInput(verifier, 1024);
        const SetupOutputVerificationResult r = verifier.finish(cfg, false);
        assert(!r.valid);
        assert(!r.streamHealthPassed);
    }


    {
        SetupOutputVerificationAccumulator verifier;
        for (uint32_t i = 0; i < 100; ++i) {
            verifier.push(makeSnapshot(i + 1, Quat::identity(), Vec3::zero()));
        }
        // Reproduces the hardware log that had healthy gyro/accel statistics
        // but only 207 input observations after a fixed four-second capture.
        // The verifier must identify the exact count gate instead of making the
        // stationarity rejection opaque.
        pushStableInput(verifier, 207);
        const SetupOutputVerificationResult r = verifier.finish(cfg, true);
        assert(!r.valid);
        assert(!r.stationaryInputPassed);
        assert(!r.stationaryInputSampleCountPassed);
        assert(r.stationaryGyroMeanPassed);
        assert(r.stationaryGyroPrecisionPassed);
        assert(r.stationaryAccelMeanPassed);
        assert(r.stationaryAccelStdPassed);
    }

    {
        SetupOutputVerificationAccumulator verifier;
        for (uint32_t i = 0; i < 100; ++i) {
            verifier.push(makeSnapshot(i + 1, Quat::identity(), Vec3::zero()));
        }
        pushStableInput(verifier, 1024, 0.45f);
        const SetupOutputVerificationResult r = verifier.finish(cfg, true);
        assert(!r.valid);
        assert(!r.stationaryInputPassed);
        assert(r.inputGyroMeanDps.x > 0.40f);
    }

    return 0;
}

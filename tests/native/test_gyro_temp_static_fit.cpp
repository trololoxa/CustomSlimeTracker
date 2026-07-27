#include "test_common.hpp"

#include <cstring>
#include <initializer_list>

#include "runtime/gyro_temp_static_fit.hpp"

using namespace tracker;

namespace {

class SinkStream final : public Stream {
public:
    std::size_t write(uint8_t) override { return 1u; }
    std::size_t write(const uint8_t*, std::size_t len) override { return len; }
};

void addBin(StaticRuntimeTest& test,
            float tempC,
            const Vec3& meanDps,
            const Vec3& noiseDps,
            uint32_t samples) {
    const int idx = staticTempBinIndex(tempC);
    if (idx < 0) return;
    auto& bin = test.tempBins[idx];
    for (uint32_t i = 0; i < samples; ++i) {
        const float sx = (i & 1u) ? 1.0f : -1.0f;
        const float sy = (i & 2u) ? 1.0f : -1.0f;
        const float sz = (i & 4u) ? 1.0f : -1.0f;
        const Vec3 dps = meanDps + Vec3(noiseDps.x * sx, noiseDps.y * sy, noiseDps.z * sz);
        bin.push(tempC, dps * MATH_DEG_TO_RAD, 1.0f, true);
        test.tempC.push(tempC);
        test.gyroAfterRadS.push(dps * MATH_DEG_TO_RAD);
        test.samples++;
    }
}

} // namespace

int main() {
    TestContext t;

    StaticRuntimeTest capture;
    capture.reset();
    const Vec3 slopeDpsPerC(0.018f, -0.011f, 0.007f);
    const Vec3 atRefDps(0.015f, -0.010f, 0.006f);
    const Vec3 noiseDps(0.58f, 0.30f, 0.07f);
    constexpr float refC = 31.5f;
    for (float temp : {29.2f, 30.2f, 31.2f, 32.2f, 33.2f, 34.2f}) {
        addBin(capture, temp, atRefDps + slopeDpsPerC * (temp - refC), noiseDps, 2048u);
    }

    GyroTempCompensator comp;
    comp.setModel(Vec3::zero(), refC, Vec3::zero());
    comp.setEnabled(true);
    ImuCalibration imu;
    imu.gyroBiasValid = true;
    RuntimeGyroBiasEstimator bias;
    bias.enabled = true;
    TrackerConfig config;
    config.resetDefaults();
    TrackerConfigStore store("temp_fit_test", "cfg");
    SinkStream sink;

    GyroTempStaticFitDeps deps;
    deps.gyroTempComp = &comp;
    deps.imuCal = &imu;
    deps.runtimeBias = &bias;
    deps.config = &config;
    deps.configStore = &store;

    CHECK(t, fitGyroTempFromCompletedStaticTestEx(
        deps, capture, GyroTempStaticFitMode::ApplyRam, sink));
    CHECK(t, comp.valid());
    CHECK(t, comp.snapshot(refC).enabled);
    CHECK_NEAR(t, comp.slopeRadSPerC().x * MATH_RAD_TO_DEG, slopeDpsPerC.x, 0.004f);
    CHECK_NEAR(t, comp.slopeRadSPerC().y * MATH_RAD_TO_DEG, slopeDpsPerC.y, 0.004f);
    CHECK_NEAR(t, comp.slopeRadSPerC().z * MATH_RAD_TO_DEG, slopeDpsPerC.z, 0.004f);
    CHECK(t, config.data.gyroCal.tempCompValid);
    CHECK(t, config.data.gyroCal.tempCompEnabled);

    // A physically inconsistent middle bin must not produce an accepted model.
    StaticRuntimeTest bad = capture;
    const int badIdx = staticTempBinIndex(32.2f);
    CHECK(t, badIdx >= 0);
    if (badIdx >= 0) {
        bad.tempBins[badIdx].gyroAfterRadS.reset();
        for (uint32_t i = 0; i < 2048u; ++i) {
            bad.tempBins[badIdx].gyroAfterRadS.push(Vec3(0.45f, -0.35f, 0.25f) * MATH_DEG_TO_RAD);
        }
    }
    GyroTempCompensator badComp;
    badComp.setModel(Vec3::zero(), refC, Vec3::zero());
    badComp.setEnabled(true);
    ImuCalibration badImu;
    badImu.gyroBiasValid = true;
    RuntimeGyroBiasEstimator badBias;
    TrackerConfig badConfig;
    badConfig.resetDefaults();
    GyroTempStaticFitDeps badDeps{nullptr, false, &badComp, &badImu, &badBias, &badConfig, &store};
    CHECK(t, !fitGyroTempFromCompletedStaticTestEx(
        badDeps, bad, GyroTempStaticFitMode::ApplyRam, sink));

    return t.finish("gyro_temp_static_fit");
}

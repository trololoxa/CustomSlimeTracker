#include "test_common.hpp"

#include <string>

#include "runtime/runtime_test_runner.hpp"
#include "runtime/static_test_runner.hpp"

using namespace tracker;

namespace {

class CaptureStream final : public Stream {
public:
    std::string output;

    size_t write(uint8_t value) override {
        output.push_back(static_cast<char>(value));
        return 1u;
    }

    size_t write(const uint8_t* data, size_t len) override {
        if (data != nullptr) output.append(reinterpret_cast<const char*>(data), len);
        return data != nullptr ? len : 0u;
    }
};

class NullLsmTransport final : public Lsm6dsvTransport {
public:
    bool read(uint8_t, uint8_t* dst, size_t len) override {
        if (dst == nullptr || len == 0u) return false;
        for (size_t i = 0; i < len; ++i) dst[i] = 0u;
        return true;
    }

    bool write(uint8_t, const uint8_t*, size_t) override { return true; }
    void delayMs(uint32_t) override {}
};

void testStaticCompletion(TestContext& ctx) {
    StaticRuntimeTest active;
    StaticRuntimeTest completed;
    bool completedValid = false;
    uint32_t completedMs = 0u;
    ImuQualityMonitor qualityMonitor;
    NullLsmTransport bus;
    Lsm6dsv lsm{bus};
    Lsm6dsvFifoReader fifo{bus, lsm};
    TrackerPerfCounters perf;
    TrackerConfig config;
    TrackerWifiManager wifi;
    SlimeVROutputRuntime slime;
    Ahrs6Dof ahrs;
    MagRuntimeProcessor magProcessor;
    MagHeadingEstimator magHeading;
    MagYawCorrectionController magYaw;
    MagHeadingReferenceState magReference;
    MagYawCorrectionOutput lastYaw;

    StaticTestRunner::Dependencies deps;
    deps.activeTest = &active;
    deps.lastCompletedTest = &completed;
    deps.lastCompletedValid = &completedValid;
    deps.lastCompletedFinishedMs = &completedMs;
    deps.quality = &qualityMonitor;
    deps.fifo = &fifo;
    deps.perf = &perf;
    deps.config = &config;
    deps.ahrs = &ahrs;
    deps.wifi = &wifi;
    deps.slimevr = &slime;
    deps.magProcessor = &magProcessor;
    deps.magHeading = &magHeading;
    deps.magYawCorrection = &magYaw;
    deps.magHeadingRef = &magReference;
    deps.lastMagYawCorrection = &lastYaw;

    CaptureStream out;
    StaticTestRunner runner;
    trackerTestSetMicros(100000u);
    runner.begin(deps);
    CHECK(ctx, runner.start(1000u, out, 0.0f));
    out.output.clear();
    CHECK(ctx, runner.stop(out));

    Lsm6dsv::Sample sample;
    sample.accel_g = Vec3(0.0f, 0.0f, 1.0f);
    sample.gyro_rad_s = Vec3::zero();
    sample.temp_c = 25.0f;
    ImuQualityResult quality;
    quality.dtUs = 1000u;
    quality.accelConfidence = 1.0f;
    quality.accelNormG = 1.0f;
    quality.accelNormValid = true;
    quality.shouldUpdateAhrs = true;
    runner.updateSample(sample, quality);

    CHECK(ctx, !runner.active());
    CHECK(ctx, completedValid);
    CHECK(ctx, runner.lastCompleted() != nullptr);
    CHECK(ctx, out.output.find("STATIC TEST DONE") != std::string::npos);
    CHECK(ctx, out.output.find("COMMAND STATIC TEST REPORT") == std::string::npos);
    CHECK(ctx, out.output.size() < 192u);

    out.output.clear();
    CHECK(ctx, runner.printLastSummary(out));
    CHECK(ctx, out.output.find("TESTSUM,static,") == 0u);
    CHECK(ctx, out.output.find("COMMAND STATIC TEST REPORT") == std::string::npos);

    out.output.clear();
    CHECK(ctx, runner.printLastReport(out));
    CHECK(ctx, out.output.find("COMMAND STATIC TEST REPORT") != std::string::npos);
    CHECK(ctx, out.output.find("STATIC TEST REPORT END") != std::string::npos);
}

void testRuntimeCompletion(TestContext& ctx) {
    TrackerPerfCounters perf;
    NullLsmTransport bus;
    Lsm6dsv lsm{bus};
    Lsm6dsvFifoReader fifo{bus, lsm};
    ImuQualityMonitor quality;
    TrackerWifiManager wifi;
    SlimeVROutputRuntime slime;
    TrackingStateController tracking;
    uint32_t runtimeSamples = 10u;
    float temperatureC = 25.0f;

    RuntimeTestRunner::Dependencies deps;
    deps.perf = &perf;
    deps.fifo = &fifo;
    deps.quality = &quality;
    deps.wifi = &wifi;
    deps.slimevr = &slime;
    deps.trackingState = &tracking;
    deps.runtimeSamples = &runtimeSamples;
    deps.latestTempC = &temperatureC;

    CaptureStream out;
    RuntimeTestRunner runner;
    runner.begin(deps);
    CHECK(ctx, runner.start(1000u, 100u, out));
    out.output.clear();
    RuntimeLoopTimingSample timing;
    timing.loopUs = 100u;
    runner.recordLoopTiming(timing, true);
    runtimeSamples = 20u;
    runner.update(1100u);

    CHECK(ctx, !runner.active());
    CHECK(ctx, runner.reportReady());
    CHECK(ctx, out.output.find("RUNTIME TEST DONE") != std::string::npos);
    CHECK(ctx, out.output.find("COMMAND RUNTIME TEST REPORT") == std::string::npos);
    CHECK(ctx, out.output.size() < 192u);

    out.output.clear();
    CHECK(ctx, runner.printLastSummary(out));
    CHECK(ctx, out.output.find("TESTSUM,runtime,") == 0u);
    CHECK(ctx, out.output.find("COMMAND RUNTIME TEST REPORT") == std::string::npos);

    out.output.clear();
    CHECK(ctx, runner.printLastReport(out));
    CHECK(ctx, out.output.find("COMMAND RUNTIME TEST REPORT") != std::string::npos);
    CHECK(ctx, out.output.find("RUNTIME TEST REPORT END") != std::string::npos);
}

} // namespace

int main() {
    TestContext ctx;
    testStaticCompletion(ctx);
    testRuntimeCompletion(ctx);
    return ctx.finish("diagnostic_completion_deferred");
}

#include "test_common.hpp"

#include <limits>

#include "sensor/calibration.hpp"
#include "sensor/accel_6pos_calibration.hpp"
#include "sensor/fifo_calibrations.hpp"

using namespace tracker;

namespace {

class CalibrationNullTransport final : public Lsm6dsvTransport {
public:
    bool available = false;
    uint8_t registers[256]{};
    bool read(uint8_t reg, uint8_t* dst, size_t len) override {
        if (!available) return false;
        for (size_t i = 0u; i < len; ++i) dst[i] = registers[static_cast<uint8_t>(reg + i)];
        return true;
    }
    bool write(uint8_t reg, const uint8_t* src, size_t len) override {
        if (!available) return false;
        for (size_t i = 0u; i < len; ++i) registers[static_cast<uint8_t>(reg + i)] = src[i];
        return true;
    }
    void delayMs(uint32_t) override {}
};

struct CalibrationWaitProbe {
    uint32_t calls = 0u;
    bool cancel = false;
    uint32_t nowMs = 0u;
    uint32_t maxSlice = 0u;
    uint32_t services = 0u;
    uint32_t begins = 0u;
    uint32_t ends = 0u;
};

bool calibrationWaitTimeout(uint32_t timeoutMs, void* user) {
    auto& probe = *static_cast<CalibrationWaitProbe*>(user);
    ++probe.calls;
    probe.nowMs += timeoutMs;
    probe.maxSlice = std::max(probe.maxSlice, timeoutMs);
    return false;
}

uint32_t captureClock(void* user) { return static_cast<CalibrationWaitProbe*>(user)->nowMs; }
bool captureService(FifoCalibrationService event, void* user) {
    auto& p = *static_cast<CalibrationWaitProbe*>(user);
    if (event == FifoCalibrationService::Begin) ++p.begins;
    if (event == FifoCalibrationService::Progress) ++p.services;
    if (event == FifoCalibrationService::End) ++p.ends;
    return true;
}

bool calibrationCancelRequested(void* user) {
    return static_cast<CalibrationWaitProbe*>(user)->cancel;
}

} // namespace

static Lsm6dsv::Sample makeSample(const Vec3& gyroRadS, const Vec3& accelG) {
    Lsm6dsv::Sample s;
    s.gyro_rad_s = gyroRadS;
    s.accel_g = accelG;
    return s;
}

static void testImuCalibrationApply(TestContext& ctx) {
    ImuCalibration cal;
    cal.gyroBiasValid = true;
    cal.gyroBiasRadS = Vec3(0.1f, -0.2f, 0.3f);
    cal.accelCalValid = true;
    cal.accelBiasG = Vec3(0.01f, -0.02f, 0.03f);
    cal.accelScale = Mat3::diagonal(2.0f, 3.0f, 4.0f);

    const Lsm6dsv::Sample in = makeSample(Vec3(1.0f, 2.0f, 3.0f), Vec3(0.5f, 0.5f, 0.5f));
    const Lsm6dsv::Sample out = cal.apply(in);

    CHECK_NEAR(ctx, out.gyro_rad_s.x, 0.9f, 1.0e-6f);
    CHECK_NEAR(ctx, out.gyro_rad_s.y, 2.2f, 1.0e-6f);
    CHECK_NEAR(ctx, out.gyro_rad_s.z, 2.7f, 1.0e-6f);
    CHECK_NEAR(ctx, out.accel_g.x, 0.98f, 1.0e-6f);
    CHECK_NEAR(ctx, out.accel_g.y, 1.56f, 1.0e-6f);
    CHECK_NEAR(ctx, out.accel_g.z, 1.88f, 1.0e-6f);
}


static void testAccel6PosFull3x3Calibration(TestContext& ctx) {
    Accel6PosCalibration cal;

    const Vec3 bias(0.035f, -0.020f, 0.045f);
    const Mat3 correction(
        1.040f, -0.030f,  0.020f,
        0.015f,  0.970f, -0.025f,
       -0.018f,  0.012f,  1.060f
    );
    Mat3 rawBasis;
    CHECK(ctx, correction.inverse(rawBasis));

    for (uint8_t i = 0; i < 6; ++i) {
        const auto face = static_cast<Accel6PosCalibration::Face>(i);
        const Vec3 expected = Accel6PosCalibration::expectedVector(face);
        const Vec3 raw = bias + rawBasis * expected;
        CHECK(ctx, cal.setFace(face, raw, 1024, Vec3(1.0e-6f, 1.0e-6f, 1.0e-6f)));
    }

    CHECK(ctx, cal.compute());
    const auto& r = cal.result();
    CHECK(ctx, r.valid);
    CHECK_NEAR(ctx, r.biasG.x, bias.x, 1.0e-5f);
    CHECK_NEAR(ctx, r.biasG.y, bias.y, 1.0e-5f);
    CHECK_NEAR(ctx, r.biasG.z, bias.z, 1.0e-5f);

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            CHECK_NEAR(ctx, r.scaleMatrix.m[row][col], correction.m[row][col], 2.0e-5f);
        }
    }

    const Vec3 rawTest = bias + rawBasis * Vec3(0.25f, -0.50f, 0.75f);
    const Vec3 corrected = Accel6PosCalibration::apply(rawTest, r);
    CHECK_NEAR(ctx, corrected.x, 0.25f, 2.0e-5f);
    CHECK_NEAR(ctx, corrected.y, -0.50f, 2.0e-5f);
    CHECK_NEAR(ctx, corrected.z, 0.75f, 2.0e-5f);
}

static void testAccelAutoFaceDetection(TestContext& ctx) {
    auto xp = Accel6PosCalibration::detectFace(Vec3(0.96f, 0.12f, -0.08f));
    CHECK(ctx, xp.valid);
    CHECK(ctx, xp.face == Accel6PosCalibration::Face::XP);

    auto yn = Accel6PosCalibration::detectFace(Vec3(0.05f, -0.98f, 0.10f));
    CHECK(ctx, yn.valid);
    CHECK(ctx, yn.face == Accel6PosCalibration::Face::YN);

    auto diagonal = Accel6PosCalibration::detectFace(Vec3(0.58f, 0.56f, 0.58f));
    CHECK(ctx, !diagonal.valid);

    Accel6PosCalibration::FaceDetectionParams strict;
    strict.minDominanceMarginG = 0.40f;
    auto slightlyRotated = Accel6PosCalibration::detectFace(Vec3(0.83f, 0.38f, 0.02f), strict);
    CHECK(ctx, slightlyRotated.valid);
    CHECK(ctx, slightlyRotated.face == Accel6PosCalibration::Face::XP);
}

static void testAccelInputValidation(TestContext& ctx) {
    CHECK(ctx, Accel6PosCalibration::parseFace("X") ==
        Accel6PosCalibration::Face::Invalid);
    CHECK(ctx, Accel6PosCalibration::parseFace("XPx") ==
        Accel6PosCalibration::Face::Invalid);
    CHECK(ctx, Accel6PosCalibration::parseFace("x+") ==
        Accel6PosCalibration::Face::XP);

    Accel6PosCalibration cal;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    CHECK(ctx, !cal.setFace(Accel6PosCalibration::Face::XP,
                            Vec3::unitX(), 100u, Vec3(nan, 0.0f, 0.0f)));
    CHECK(ctx, !cal.setFace(Accel6PosCalibration::Face::XP,
                            Vec3::unitX(), 100u, Vec3(-0.1f, 0.0f, 0.0f)));

    Accel6PosCalibration::ValidationParams invalidParams;
    invalidParams.maxPostCalNormErrorG = 0.0f;
    CHECK(ctx, !cal.compute(invalidParams));
    CHECK(ctx, (cal.result().qualityFlags &
                accel_cal_quality_flags::INVALID_VALIDATION_PARAMS) != 0u);
}

static void testFifoCaptureWaitsAreBoundedAndCancellable(TestContext& ctx) {
    CHECK(ctx, fifoCalibrationSampleFlagsAcceptable(Lsm6dsv::FLAG_READ_OUTPUT_OK |
        Lsm6dsvFifoReader::FIFO_FLAG_SOURCE_FIFO | Lsm6dsvFifoReader::FIFO_FLAG_TS_HARDWARE |
        Lsm6dsvFifoReader::FIFO_FLAG_STATUS_WTM));
    CHECK(ctx, !fifoCalibrationSampleFlagsAcceptable(Lsm6dsv::FLAG_GYRO_SATURATED));
    CalibrationNullTransport transport;
    Lsm6dsv lsm(transport);
    Lsm6dsvFifoReader fifo(transport, lsm);
    Lsm6dsv::RawSample raw[8]{};
    CalibrationWaitProbe probe;
    FifoCalibrationIo io;
    io.lsm = &lsm;
    io.fifo = &fifo;
    io.rawBuffer = raw;
    io.rawBufferCapacity = 8u;
    io.waitForFifoEvent = &calibrationWaitTimeout;
    io.waitUser = &probe;
    io.cancelRequested = &calibrationCancelRequested;
    io.cancelUser = &probe;
    io.clockMs = &captureClock;
    io.clockUser = &probe;
    io.serviceCapture = &captureService;
    io.serviceUser = &probe;

    FifoGyroStartupCalibrationParams gyroParams;
    gyroParams.maximumCaptureMs = 120000u;
    gyroParams.maximumConsecutiveWaitTimeouts = 3u;
    FifoGyroStartupCalibrator gyro(gyroParams);
    GyroStartupCalibrationResult gyroResult;
    CHECK(ctx, !gyro.run(io, gyroResult));
    CHECK(ctx, probe.nowMs == gyroParams.fifoWaitTimeoutMs * 3u);
    CHECK(ctx, probe.maxSlice <= 20u);
    CHECK(ctx, probe.services == probe.calls);
    CHECK(ctx, probe.begins == 1u && probe.ends == 1u);
    CHECK(ctx, gyro.lastStatus() == FifoCalibrationCaptureStatus::SensorUnavailable);

    // One session owns train and held-out validation: no second deadline.
    FifoCalibrationCaptureStatus status;
    probe.nowMs = 0xfffffff0u;
    {
        FifoCalibrationCaptureSession session(io, status, 16001u, 1000u, 100u);
        for (unsigned i = 0; i < 16u; ++i) {
            CHECK(ctx, !session.wait());
            CHECK(ctx, !session.failed());
        }
        CHECK(ctx, session.check());
        CHECK(ctx, !session.wait());
        CHECK(ctx, status == FifoCalibrationCaptureStatus::DeadlineExceeded);
    }
    CHECK(ctx, probe.begins == probe.ends);
    transport.available = true;
    CHECK(ctx, fifo.configure(Lsm6dsvFifoReader::Config{}));
    {
        FifoCalibrationCaptureSession session(io, status, 1000u, 100u, 3u);
        Lsm6dsv::RawSample rawSample;
        rawSample.t_us = 1000u;
        CHECK(ctx, session.acceptFresh(rawSample));
        CHECK(ctx, !session.acceptFresh(rawSample));
        rawSample.t_us = 900u;
        CHECK(ctx, !session.acceptFresh(rawSample));
        rawSample.t_us = 950u;
        CHECK(ctx, !session.acceptFresh(rawSample));
        rawSample.t_us = 2000u;
        rawSample.flags = Lsm6dsv::FLAG_GYRO_SATURATED;
        CHECK(ctx, !session.acceptFresh(rawSample));
        rawSample.flags = 0u;
        rawSample.t_us = 3000u;
        CHECK(ctx, session.acceptFresh(rawSample));
        probe.cancel = true;
        CHECK(ctx, !session.check());
        CHECK(ctx, status == FifoCalibrationCaptureStatus::Cancelled);
    }
    probe.cancel = false;
    {
        FifoCalibrationCaptureSession session(io, status, 0u, 100u, 3u);
        CHECK(ctx, status == FifoCalibrationCaptureStatus::InvalidRequest);
    }
    CHECK(ctx, probe.begins == probe.ends);

    probe.calls = 0u;
    probe.cancel = true;
    FifoAccel6PosCaptureParams accelParams;
    accelParams.maximumConsecutiveWaitTimeouts = 3u;
    FifoAccel6PosCalibrationRunner accel(accelParams);
    CHECK(ctx, !accel.captureFace(io, Accel6PosCalibration::Face::XP));
    CHECK(ctx, probe.calls == 0u);
    CHECK(ctx, accel.lastStatus() == FifoCalibrationCaptureStatus::Cancelled);
}

static void testAccelCaptureResetsAfterMotion(TestContext& ctx) {
    Accel6PosCapture::Params params;
    params.requiredSamples = 4;
    params.resetAfterConsecutiveRejected = 2;
    Accel6PosCapture capture(params);
    capture.begin(Accel6PosCalibration::Face::XP);

    CHECK(ctx, !capture.push(makeSample(Vec3::zero(), Vec3(1.0f, 0.0f, 0.0f))));
    CHECK(ctx, !capture.push(makeSample(Vec3::zero(), Vec3(1.0f, 0.0f, 0.0f))));
    CHECK(ctx, capture.snapshot().acceptedSamples == 2);

    capture.push(makeSample(Vec3(10.0f * MATH_DEG_TO_RAD, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f)));
    capture.push(makeSample(Vec3(10.0f * MATH_DEG_TO_RAD, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f)));
    CHECK(ctx, capture.snapshot().acceptedSamples == 0);

    for (int i = 0; i < 4; ++i) {
        capture.push(makeSample(Vec3::zero(), Vec3(0.0f, 1.0f, 0.0f)));
    }
    CHECK(ctx, capture.done());
    CHECK_NEAR(ctx, capture.snapshot().meanG.y, 1.0f, 1.0e-6f);
}

static void testStationaryStats(TestContext& ctx) {
    StationaryStats stats;
    stats.push(makeSample(Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 1.0f)));
    stats.push(makeSample(Vec3(3.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 1.0f)));

    CHECK(ctx, stats.count == 2);
    CHECK_NEAR(ctx, stats.gyroMeanRadS.x, 2.0f, 1.0e-6f);
    CHECK_NEAR(ctx, stats.gyroVarianceRadS2().x, 2.0f, 1.0e-6f);
    CHECK_NEAR(ctx, stats.accelNormMeanG, 1.0f, 1.0e-6f);

    stats.reset();
    CHECK(ctx, stats.count == 0);
    CHECK_NEAR(ctx, stats.gyroVarianceNormRadS2(), 0.0f, 1.0e-6f);
}

static void testStationaryDetectorAndGyroStartup(TestContext& ctx) {
    GyroStartupCalibrationParams params;
    params.requiredStationarySamples = 8;
    params.maxTotalSamples = 64;
    params.stationary.warmupSamples = 2;
    params.stationary.maxGyroNormRadS = 1.0f * MATH_DEG_TO_RAD;
    params.stationary.maxAccelNormErrorG = 0.05f;
    params.stationary.maxGyroVarianceRadS2 = square(0.1f * MATH_DEG_TO_RAD);
    params.stationary.maxAccelVarianceG2 = square(0.002f);

    GyroStartupCalibrator cal(params);
    const Vec3 bias = Vec3(0.2f, -0.1f, 0.05f) * MATH_DEG_TO_RAD;
    bool done = false;
    for (int i = 0; i < 16; ++i) {
        done = cal.push(makeSample(bias, Vec3(0.0f, 0.0f, 1.0f)));
        if (done) break;
    }

    CHECK(ctx, done);
    CHECK(ctx, cal.done());
    CHECK(ctx, cal.result().success);
    CHECK(ctx, cal.result().stationarySamples >= params.requiredStationarySamples);
    CHECK_NEAR(ctx, cal.result().gyroBiasDps.x, 0.2f, 1.0e-5f);
    CHECK_NEAR(ctx, cal.result().gyroBiasDps.y, -0.1f, 1.0e-5f);
    CHECK_NEAR(ctx, cal.result().accelNormMeanG, 1.0f, 1.0e-6f);
}

static void testOnlineGyroBiasEstimator(TestContext& ctx) {
    OnlineGyroBiasEstimator estimator(0.5f);
    CHECK(ctx, !estimator.initialized());

    estimator.updateIfStationary(Vec3(1.0f, 0.0f, 0.0f), false);
    CHECK(ctx, !estimator.initialized());

    estimator.updateIfStationary(Vec3(1.0f, 0.0f, 0.0f), true);
    CHECK(ctx, estimator.initialized());
    CHECK_NEAR(ctx, estimator.biasRadS().x, 1.0f, 1.0e-6f);

    estimator.updateIfStationary(Vec3(3.0f, 0.0f, 0.0f), true);
    CHECK_NEAR(ctx, estimator.biasRadS().x, 2.0f, 1.0e-6f);
}

static void testFifoGyroQualityUsesMeanPrecisionAndHoldout(TestContext& ctx) {
    FifoGyroStartupCalibrationParams params;
    GyroStartupCalibrationResult result;
    result.stationarySamples = params.requiredStationarySamples + params.validationSamples;
    result.validationSamples = params.validationSamples;

    // Hardware log from the first 0023d setup attempt: raw X-axis sample
    // noise is above the old 0.20 dps threshold, but 1536 samples give a
    // precise mean and the independent 384-sample holdout agrees closely.
    result.gyroStdDps = Vec3(0.581890f, 0.298635f, 0.067053f);
    result.gyroMeanStdErrorDps = Vec3(0.014847f, 0.007620f, 0.001711f);
    result.accelStdG = Vec3(0.002893f, 0.009130f, 0.005567f);
    result.validationResidualDps = Vec3(0.002848f, 0.003053f, -0.001800f);
    result.validationGyroStdDps = result.gyroStdDps;
    result.validationGyroMeanStdErrorDps = Vec3(0.029694f, 0.015239f, 0.003422f);
    result.validationAccelStdG = result.accelStdG;
    result.validationAccelNormMeanG = 0.997327f;
    result.validationAccelMeanDeltaG = 0.000117f;
    result.temperatureSpanC = 0.1016f;
    CHECK(ctx, fifoGyroStartupCalibrationEvaluateQuality(result, params));

    result.gyroMeanStdErrorDps.x = 0.10f;
    CHECK(ctx, !fifoGyroStartupCalibrationEvaluateQuality(result, params));
    result.gyroMeanStdErrorDps.x = 0.014847f;
    result.validationResidualDps.x = 0.20f;
    CHECK(ctx, !fifoGyroStartupCalibrationEvaluateQuality(result, params));
}

int main() {
    TestContext ctx;
    testImuCalibrationApply(ctx);
    testAccel6PosFull3x3Calibration(ctx);
    testAccelAutoFaceDetection(ctx);
    testAccelInputValidation(ctx);
    testFifoCaptureWaitsAreBoundedAndCancellable(ctx);
    testAccelCaptureResetsAfterMotion(ctx);
    testStationaryStats(ctx);
    testStationaryDetectorAndGyroStartup(ctx);
    testOnlineGyroBiasEstimator(ctx);
    testFifoGyroQualityUsesMeanPrecisionAndHoldout(ctx);
    return ctx.finish("test_sensor_calibration");
}

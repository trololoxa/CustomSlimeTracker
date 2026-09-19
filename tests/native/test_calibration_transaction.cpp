#include "test_common.hpp"
#include "Preferences.h"
#include "serial/tracker_calibration_transaction.hpp"
#include "serial/tracker_calibration_capture_cancel.hpp"

Stream Serial;
using namespace tracker;

namespace {
bool probationPass = true;
unsigned probationCalls = 0u;
bool preparedBeforeApply = false;
TrackerConfig oldAuthority;
bool injectSelectorFailure = false;
unsigned resetCalls = 0u;
}

namespace tracker {
// This test exercises the real transaction and NVS store; sensor probation is
// injected here. Numerical/freshness verdicts have their own native suite.
bool trackerSerialVerifyCalibrationCandidate(TrackerSerialCommandContext& ctx,
                                             const TrackerConfig& candidate, bool) {
    ++probationCalls;
    TrackerConfig durable;
    preparedBeforeApply = ctx.configStore->verify(durable) &&
        trackerCalibrationModelEqual(durable, oldAuthority) &&
        ctx.configStore->autonomyProbationWriteBarrier() &&
        trackerCalibrationModelEqual(*ctx.config, candidate) &&
        ctx.imuCal->gyroBiasRadS.x == candidate.data.gyroCal.biasRadS.x;
    if (injectSelectorFailure) Preferences::setPutLimitForKey("tx_test", "cfg_s", 0u);
    return probationPass;
}
bool trackerSerialCaptureRuntimeToConfig(TrackerSerialCommandContext& ctx, TrackerConfig& target) {
    if (ctx.imuCal) target.captureFromImuCalibration(*ctx.imuCal);
    if (ctx.gyroTempComp) target.captureFromGyroTempComp(*ctx.gyroTempComp);
    target.updateCrc();
    return target.validateSemanticConfig();
}
void trackerSerialResetCalibrationWorkspaces(TrackerSerialCommandContext&, uint8_t) { ++resetCalls; }
}

int main() {
    TestContext test;
    for (unsigned failure = 0u; failure < 5u; ++failure) {
        Preferences::clearTestStorage();
        TrackerConfigStore store("tx_test", "cfg");
        TrackerConfig config;
        config.resetDefaults();
        config.data.gyroCal.biasValid = true;
        config.data.gyroCal.biasRadS = Vec3(0.001f, 0.002f, 0.003f);
        config.updateCrc();
        CHECK(test, store.save(config));
        oldAuthority = config;
        ImuCalibration imu;
        config.applyToImuCalibration(imu);
        GyroTempCompensator temp;
        temp.setStaticBias(Vec3(0.004f, 0.005f, 0.006f), 37.0f);
        RuntimeGyroBiasEstimator bias;
        bias.runtimeTrimRadS = Vec3(0.0001f, 0.0002f, 0.0003f);
        bias.updates = 17u;
        const auto oldTemp = temp.snapshot(41.0f);
        const auto oldTrim = bias.runtimeTrimRadS;
        TrackerSerialCommandContext ctx;
        ctx.config = &config;
        ctx.configStore = &store;
        ctx.imuCal = &imu;
        ctx.gyroTempComp = &temp;
        ctx.runtimeBias = &bias;
        TrackerConfig candidate = config;
        candidate.data.gyroCal.biasRadS.x = 0.002f;
        candidate.updateCrc();
        const auto frozenBytes = candidate.data;
        TrackerCalibrationTransaction transaction;
        transaction.begin(ctx);
        probationCalls = 0u;
        resetCalls = 0u;
        probationPass = failure != 1u;
        preparedBeforeApply = false;
        injectSelectorFailure = failure == 4u;
        if (failure == 2u) Preferences::setPutLimitForKey("tx_test", "cfg_b", 0u);
        if (failure == 3u) Preferences::setGetFailuresForKey("tx_test", "cfg_b", 1u);
        const bool ok = transaction.commit(ctx, TRACKER_CAL_WORKSPACE_ALL,
                                           TrackerCalibrationProvenance::Manual, &candidate);
        CHECK(test, std::memcmp(&frozenBytes, &candidate.data, sizeof(frozenBytes)) == 0);
        CHECK(test, ok == (failure == 0u));
        if (failure == 2u || failure == 3u) CHECK(test, probationCalls == 0u);
        else CHECK(test, preparedBeforeApply && probationCalls == 1u);
        if (failure == 4u) {
            CHECK(test, transaction.persistentResultUncertain);
            CHECK(test, !store.autonomyProbationWriteBarrier());
            TrackerConfig blocked = candidate;
            CHECK(test, !store.save(blocked));
            CHECK(test, store.lastError() == TrackerConfigError::CommitUncertain);
            transaction.rollback(ctx, "must_not_guess_uncertain_authority");
            CHECK(test, trackerCalibrationModelEqual(config, candidate));
        } else if (!ok) {
            transaction.rollback(ctx, "test_injected_failure");
            CHECK(test, trackerCalibrationModelEqual(config, oldAuthority));
            CHECK(test, imu.gyroBiasRadS.x == oldAuthority.data.gyroCal.biasRadS.x);
            CHECK(test, temp.snapshot(41.0f).referenceTempC == oldTemp.referenceTempC);
            CHECK(test, temp.snapshot(41.0f).currentBiasRadS.x == oldTemp.currentBiasRadS.x);
            CHECK(test, bias.runtimeTrimRadS.x == oldTrim.x && bias.updates == 17u);
        }
        if (failure == 2u || failure == 3u) CHECK(test, resetCalls == 0u);
        if (failure != 4u) CHECK(test, !store.autonomyProbationWriteBarrier());
        TrackerConfig durable;
        CHECK(test, store.verify(durable));
        CHECK(test, trackerCalibrationModelEqual(durable, ok ? candidate : oldAuthority));
        if (failure == 4u) {
            Preferences::setPutLimitForKey("tx_test", "cfg_s", 4096u);
            CHECK(test, store.load(durable));
            config = durable;
            config.applyToImuCalibration(imu);
            config.applyToGyroTempComp(temp);
            store.confirmAuthoritativeConfigApplied();
            CHECK(test, !store.autonomyProbationWriteBarrier());
            CHECK(test, store.save(candidate));
        }
    }
    {
        struct BusyStream final : Stream {
            int reads = 0;
            int available() override { return 100000; }
            int read() override { ++reads; return 'x'; }
        } input;
        bool cancel = false;
        auto previous = +[](void* user) { return *static_cast<bool*>(user); };
        FifoCalibrationIo io;
        io.cancelRequested = previous;
        io.cancelUser = &cancel;
        TrackerSerialCommandContext ctx;
        ctx.calibrationIo = &io;
        { TrackerCalibrationCaptureCancelScope absentStream(ctx); }
        CHECK(test, io.cancelRequested == previous && io.cancelUser == &cancel);
        ctx.io = &input;
        {
            TrackerCalibrationCaptureCancelScope scope(ctx);
            CHECK(test, !io.cancelRequested(io.cancelUser));
            CHECK(test, input.reads == 32);
            cancel = true;
            CHECK(test, io.cancelRequested(io.cancelUser));
            CHECK(test, input.reads == 64);
        }
        CHECK(test, io.cancelRequested == previous && io.cancelUser == &cancel);
    }
    return test.finish("test_calibration_transaction");
}

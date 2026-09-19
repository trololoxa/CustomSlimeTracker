#include "test_common.hpp"

#include <cstddef>
#include <cstdint>

#include "runtime/boot_health.hpp"
#include "runtime/sensor_progress_watchdog.hpp"
#include "runtime/sensor_recovery_controller.hpp"

using namespace tracker;

namespace {

void testProgressFaultsAndRecovery(TestContext& ctx) {
    SensorProgressWatchdog watchdog;
    watchdog.configure(100u, 1042.0f, 128u);
    CHECK(ctx, watchdog.timeoutUs() >= 500000u);

    const uint32_t deadline = 100u + watchdog.timeoutUs();
    CHECK(ctx, watchdog.evaluate(deadline - 1u, true) == SensorProgressFault::None);
    CHECK(ctx, watchdog.evaluate(deadline, true) == SensorProgressFault::NoIrqOrDrain);

    watchdog.noteDrain(deadline);
    CHECK(ctx, watchdog.evaluate(deadline, true) == SensorProgressFault::NoAcceptedGyro);
    watchdog.noteAcceptedGyro();
    CHECK(ctx, watchdog.evaluate(deadline, true) ==
                   SensorProgressFault::NoOrientationPublication);
    watchdog.noteOrientationPublication();
    CHECK(ctx, watchdog.evaluate(deadline, true) == SensorProgressFault::None);

    // A live gyro stream remains sufficient while orientation publication is
    // intentionally not expected during AHRS bootstrap.
    const uint32_t next = deadline + watchdog.timeoutUs();
    watchdog.noteDrain(next);
    watchdog.noteAcceptedGyro();
    CHECK(ctx, watchdog.evaluate(next, false) == SensorProgressFault::None);

    const SensorProgressSnapshot snapshot = watchdog.snapshot();
    CHECK(ctx, snapshot.lastFault == SensorProgressFault::None);
    CHECK(ctx, snapshot.lastTriggeredFault == SensorProgressFault::NoOrientationPublication);
    CHECK(ctx, snapshot.lastFaultAtUs == deadline);
}

void testProducerProgressUsesWatchdogClockDomain(TestContext& ctx) {
    SensorProgressWatchdog watchdog;
    constexpr uint32_t localStartUs = 800000u;
    watchdog.configure(localStartUs, 1048.0f, 12u);

    // The IMU clock may be hundreds of milliseconds behind micros(). Producer
    // progress carries no cross-domain timestamp; evaluate() records the edge
    // at the current watchdog-service time.
    const uint32_t serviceAtUs = localStartUs + watchdog.timeoutUs();
    watchdog.noteDrain(serviceAtUs);
    watchdog.noteAcceptedGyro();
    watchdog.noteOrientationPublication();
    CHECK(ctx, watchdog.evaluate(serviceAtUs, true) == SensorProgressFault::None);

    const SensorProgressSnapshot live = watchdog.snapshot();
    CHECK(ctx, live.lastAcceptedGyroAtUs == serviceAtUs);
    CHECK(ctx, live.lastOrientationAtUs == serviceAtUs);
    CHECK(ctx, watchdog.evaluate(serviceAtUs + watchdog.timeoutUs() - 1u, true) ==
                   SensorProgressFault::None);
}

void testOrientationPublicationContract(TestContext& ctx) {
    CHECK(ctx, !sensorProgressOrientationExpected(false, false, false));
    CHECK(ctx, sensorProgressOrientationExpected(true, false, false));
    CHECK(ctx, !sensorProgressOrientationExpected(true, true, false));
    CHECK(ctx, sensorProgressOrientationExpected(true, true, true));
}

void testProgressSuppressionAndWrap(TestContext& ctx) {
    SensorProgressWatchdog watchdog;
    const uint32_t start = 0xfffffff0u;
    watchdog.configure(start, 1000.0f, 1u);
    watchdog.setSuppressed(SensorProgressSuppressReason::SensorReinit, true, start);
    CHECK(ctx, watchdog.evaluate(start + watchdog.timeoutUs(), true) ==
                   SensorProgressFault::None);

    const uint32_t resume = start + watchdog.timeoutUs() + 123u;
    watchdog.setSuppressed(SensorProgressSuppressReason::SensorReinit, false, resume);
    watchdog.noteIrq(resume + 1u);
    watchdog.noteAcceptedGyro();
    watchdog.noteOrientationPublication();
    CHECK(ctx, watchdog.evaluate(resume + watchdog.timeoutUs() - 1u, true) ==
                   SensorProgressFault::None);
    CHECK(ctx, watchdog.evaluate(resume + watchdog.timeoutUs() + 1u, true) ==
                   SensorProgressFault::NoIrqOrDrain);
}

void testSuppressedEpochProgressIsNotReused(TestContext& ctx) {
    SensorProgressWatchdog watchdog;
    constexpr uint32_t startUs = 2000u;
    watchdog.configure(startUs, 1048.0f, 12u);

    // Progress observed while any suppress reason remains active belongs to the
    // interrupted epoch. Leaving only one of two nested reasons must not arm the
    // watchdog, and leaving the final reason must snapshot (discard) all such
    // producer edges.
    watchdog.setSuppressed(SensorProgressSuppressReason::Boot, true, startUs);
    watchdog.setSuppressed(SensorProgressSuppressReason::SensorReinit, true, startUs);
    watchdog.noteAcceptedGyro();
    watchdog.noteOrientationPublication();

    const uint32_t partialResumeUs = startUs + watchdog.timeoutUs();
    watchdog.setSuppressed(
        SensorProgressSuppressReason::SensorReinit, false, partialResumeUs);
    watchdog.noteAcceptedGyro();
    watchdog.noteOrientationPublication();
    CHECK(ctx, watchdog.evaluate(partialResumeUs, true) == SensorProgressFault::None);

    const uint32_t resumeUs = partialResumeUs + 123u;
    watchdog.setSuppressed(SensorProgressSuppressReason::Boot, false, resumeUs);
    const SensorProgressSnapshot resumed = watchdog.snapshot();
    CHECK(ctx, resumed.suppressMask == 0u);
    CHECK(ctx, resumed.armedAtUs == resumeUs);
    CHECK(ctx, resumed.lastAcceptedGyroAtUs == 0u);
    CHECK(ctx, resumed.lastOrientationAtUs == 0u);

    // A fresh drain cannot make the suppressed gyro/orientation edges count as
    // post-resume progress. Each producer must prove liveness in the new epoch.
    const uint32_t deadlineUs = resumeUs + watchdog.timeoutUs();
    watchdog.noteDrain(deadlineUs);
    CHECK(ctx, watchdog.evaluate(deadlineUs, true) == SensorProgressFault::NoAcceptedGyro);
    watchdog.noteAcceptedGyro();
    CHECK(ctx, watchdog.evaluate(deadlineUs, true) ==
                   SensorProgressFault::NoOrientationPublication);
    watchdog.noteOrientationPublication();
    CHECK(ctx, watchdog.evaluate(deadlineUs, true) == SensorProgressFault::None);
}

void testBoundedRecoveryOrderAndTerminalBackoff(TestContext& ctx) {
    SensorRecoveryController recovery;
    CHECK(ctx, recovery.request(TrackerHealthFaultCode::FifoDrainFailed, 0x10u, 0xfffffff0u));
    CHECK(ctx, recovery.poll(0xfffffff0u) == SensorRecoveryAction::TransactionalFifoReset);
    recovery.completeAttempt(SensorRecoveryAction::TransactionalFifoReset, false, 0xfffffff0u);

    CHECK(ctx, recovery.poll(0x20u) == SensorRecoveryAction::None);
    CHECK(ctx, recovery.poll(0x30u) == SensorRecoveryAction::TransactionalFifoReset);
    recovery.completeAttempt(SensorRecoveryAction::TransactionalFifoReset, false, 0x30u);
    CHECK(ctx, recovery.poll(0x100u) == SensorRecoveryAction::None);
    CHECK(ctx, recovery.poll(0x12au) == SensorRecoveryAction::FullSensorReinit);

    recovery.completeAttempt(SensorRecoveryAction::FullSensorReinit, false, 0x12au);
    CHECK(ctx, recovery.poll(0x511u) == SensorRecoveryAction::None);
    CHECK(ctx, recovery.poll(0x512u) == SensorRecoveryAction::FullSensorReinit);
    recovery.completeAttempt(SensorRecoveryAction::FullSensorReinit, false, 0x512u);
    CHECK(ctx, recovery.poll(0x1899u) == SensorRecoveryAction::None);
    CHECK(ctx, recovery.poll(0x189au) == SensorRecoveryAction::FullSensorReinit);
    recovery.completeAttempt(SensorRecoveryAction::FullSensorReinit, false, 0x189au);

    CHECK(ctx, recovery.active());
    CHECK(ctx, recovery.exhausted());
    CHECK(ctx, recovery.snapshot().attemptsInEpisode == 5u);
    CHECK(ctx, recovery.poll(0x8dc9u) == SensorRecoveryAction::None);
    CHECK(ctx, recovery.poll(0x8dcau) == SensorRecoveryAction::FullSensorReinit);
    recovery.completeAttempt(SensorRecoveryAction::FullSensorReinit, true, 0x8dcau);
    CHECK(ctx, recovery.awaitingProgress());
    CHECK(ctx, !recovery.blocksSampling());
    CHECK(ctx, recovery.confirmProgress(0x8dcbu));
    CHECK(ctx, !recovery.active());
    CHECK(ctx, !recovery.exhausted());
    CHECK(ctx, recovery.snapshot().successCount == 1u);
    CHECK(ctx, !sensorRecoveryReinitializesDevice(SensorRecoveryAction::None));
    CHECK(ctx, !sensorRecoveryReinitializesDevice(SensorRecoveryAction::TransactionalFifoReset));
    CHECK(ctx, sensorRecoveryReinitializesDevice(SensorRecoveryAction::FullSensorReinit));
}

void testRegisterSuccessWithoutStreamEscalates(TestContext& ctx) {
    SensorRecoveryController recovery;
    uint32_t now = 0xfffffff0u;
    recovery.request(TrackerHealthFaultCode::ImuNoProgress, 1u, now);
    for (unsigned attempt = 0; attempt < 8; ++attempt) {
        const auto action = recovery.poll(now);
        CHECK(ctx, action == (attempt < 2 ? SensorRecoveryAction::TransactionalFifoReset
                                        : SensorRecoveryAction::FullSensorReinit));
        recovery.completeAttempt(action, true, now, 500u);
        CHECK(ctx, recovery.awaitingProgress());
        CHECK(ctx, recovery.snapshot().successCount == 0u);
        CHECK(ctx, recovery.poll(now + 499u) == SensorRecoveryAction::None);
        CHECK(ctx, !recovery.confirmProgress(now + 500u));
        CHECK(ctx, recovery.poll(now + 500u) == SensorRecoveryAction::None);
        CHECK(ctx, recovery.blocksSampling());
        CHECK(ctx, recovery.snapshot().failureCount == attempt + 1u);
        const auto before = recovery.snapshot();
        CHECK(ctx, !recovery.request(TrackerHealthFaultCode::FifoDrainFailed, 2u, now + 500u));
        CHECK(ctx, recovery.snapshot().nextAttemptMs == before.nextAttemptMs);
        now = before.nextAttemptMs;
    }
    CHECK(ctx, recovery.exhausted());
    const auto action = recovery.poll(now);
    recovery.completeAttempt(action, true, now, 500u);
    CHECK(ctx, recovery.confirmProgress(now + 1u));
    CHECK(ctx, recovery.snapshot().successCount == 1u);
    CHECK(ctx, !recovery.active());
}

void testBootHealthRejectsRandomAndClearsAfterStable(TestContext& ctx) {
    BootHealthRecord record;
    uint8_t* recordBytes = reinterpret_cast<uint8_t*>(&record);
    for (size_t i = 0u; i < sizeof(record); ++i) recordBytes[i] = 0xa5u;
    CHECK(ctx, !BootHealthController::valid(record));

    BootHealthController first;
    first.begin(record, BootResetClass::CrashOrWatchdog);
    CHECK(ctx, BootHealthController::valid(record));
    CHECK(ctx, record.consecutiveCrashBoots == 1u);
    CHECK(ctx, !first.safeMode());

    BootHealthController second;
    second.begin(record, BootResetClass::CrashOrWatchdog);
    BootHealthController third;
    third.begin(record, BootResetClass::CrashOrWatchdog);
    CHECK(ctx, third.safeMode());
    CHECK(ctx, record.consecutiveCrashBoots == 3u);

    third.markStable(record);
    CHECK(ctx, BootHealthController::valid(record));
    CHECK(ctx, record.consecutiveCrashBoots == 0u);
    CHECK(ctx, !third.safeMode());

    BootHealthController cold;
    cold.begin(record, BootResetClass::Cold);
    CHECK(ctx, record.consecutiveCrashBoots == 0u);
    CHECK(ctx, !cold.safeMode());
}

void testHealthKeepsRecoveryAndCauseVisible(TestContext& ctx) {
    TrackerHealthState health;
    health.beginRecovery(TrackerHealthFaultCode::FifoDrainFailed, "drain failed");
    health.enterRecoveryExhausted(
        TrackerHealthFaultCode::SpiPlausibilityFailed, "probes continue");
    const TrackerHealthSnapshot exhausted = health.snapshot();
    CHECK(ctx, exhausted.fatalActive);
    CHECK(ctx, exhausted.degradedNoImu);
    CHECK(ctx, exhausted.recoveryActive);
    CHECK(ctx, exhausted.faultCode == TrackerHealthFaultCode::RecoveryExhausted);
    CHECK(ctx, exhausted.lastFaultCode == TrackerHealthFaultCode::SpiPlausibilityFailed);

    health.finishRecovery();
    const TrackerHealthSnapshot recovered = health.snapshot();
    CHECK(ctx, !recovered.fatalActive);
    CHECK(ctx, !recovered.degradedNoImu);
    CHECK(ctx, !recovered.recoveryActive);
    CHECK(ctx, recovered.recoverySuccessCount == 1u);
}

} // namespace

int main() {
    TestContext ctx;
    testProgressFaultsAndRecovery(ctx);
    testProducerProgressUsesWatchdogClockDomain(ctx);
    testOrientationPublicationContract(ctx);
    testProgressSuppressionAndWrap(ctx);
    testSuppressedEpochProgressIsNotReused(ctx);
    testBoundedRecoveryOrderAndTerminalBackoff(ctx);
    testRegisterSuccessWithoutStreamEscalates(ctx);
    testBootHealthRejectsRandomAndClearsAfterStable(ctx);
    testHealthKeepsRecoveryAndCauseVisible(ctx);
    return ctx.finish("test_sensor_liveness_recovery");
}

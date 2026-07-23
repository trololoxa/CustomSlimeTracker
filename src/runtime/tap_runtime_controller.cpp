#include "runtime/tap_runtime_controller.hpp"

#include "runtime/slimevr_output_runtime.hpp"

namespace tracker {

namespace {

bool timeReached(uint32_t nowMs, uint32_t deadlineMs) {
    return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

} // namespace

void TapRuntimeController::begin(Lsm6dsv& lsm, SlimeVROutputRuntime& slimevr) {
    lsm_ = &lsm;
    slimevr_ = &slimevr;
    status_ = TapRuntimeStatus{};
    status_.enabled = config_.enabled;
    accumulator_.configure(makeAccumulatorConfig());
    accumulator_.reset();
    nextPollMs_ = 0;
    nextRegisterVerifyMs_ = 0;
    lastReadFailureDiagnosticMs_ = 0;
    if (config_.enabled) {
        (void)configureHardware();
    }
}

bool TapRuntimeController::configure(const TapRuntimeConfig& config) {
    config_ = config;
    status_.enabled = config_.enabled;
    accumulator_.configure(makeAccumulatorConfig());
    return configureHardware();
}

bool TapRuntimeController::setEnabled(bool enabled) {
    config_.enabled = enabled;
    status_.enabled = enabled;
    accumulator_.reset();
    return configureHardware();
}

void TapRuntimeController::setPhysicalTapUserAction(SlimeVRUserAction action) {
    config_.physicalTapUserAction = action;
    status_.physicalTapUserAction = action;
}

void TapRuntimeController::resetCounters() {
    const bool enabled = status_.enabled;
    const bool hardwareConfigured = status_.hardwareConfigured;
    const bool registerVerifyOk = status_.registerVerifyOk;
    const bool lastRegisterReadOk = status_.lastRegisterReadOk;
    const Lsm6dsv::TapRegisterVerification lastVerification = status_.lastRegisterVerification;
    status_ = TapRuntimeStatus{};
    status_.enabled = enabled;
    status_.hardwareConfigured = hardwareConfigured;
    status_.registerVerifyOk = registerVerifyOk;
    status_.lastRegisterReadOk = lastRegisterReadOk;
    status_.lastRegisterVerification = lastVerification;
    accumulator_.reset();
    diagnosticEvents_ = 0;
}

TapRuntimeStatus TapRuntimeController::status() const {
    TapRuntimeStatus out = status_;
    out.physicalTapUserAction = config_.physicalTapUserAction;
    out.enabled = config_.enabled;
    out.diagnosticLogging = diagnosticLogging_;
    out.diagnosticEvents = diagnosticEvents_;

    const TapAccumulatorStatus a = accumulator_.status();
    out.pendingCount = a.pendingCount;
    out.pendingFirstMs = a.pendingFirstMs;
    out.pendingLastMs = a.pendingLastMs;
    out.flushDeadlineMs = a.flushDeadlineMs;
    out.lastPhysicalTapMs = a.lastPhysicalTapMs;
    out.postSendLockoutUntilMs = a.postSendLockoutUntilMs;
    out.physicalTapEvents = a.physicalTapEvents;
    out.physicalTapCount = a.physicalTapCount;
    out.windowsStarted = a.windowsStarted;
    out.windowsFlushed = a.windowsFlushed;
    out.suppressedBelowMin = a.suppressedBelowMin;
    out.suppressedDuplicate = a.suppressedDuplicate;
    out.suppressedLockout = a.suppressedLockout;
    out.clampedOverflow = a.clampedOverflow;
    return out;
}

Lsm6dsv::TapConfig TapRuntimeController::makeHardwareConfig() const {
    Lsm6dsv::TapConfig hw;
    hw.enabled = config_.enabled;
    hw.enableX = true;
    hw.enableY = true;
    hw.enableZ = true;
    hw.enableDoubleTap = config_.hardwareDoubleTap;
    hw.routeSingleTapToInt1 = true;
    hw.routeDoubleTapToInt1 = config_.hardwareDoubleTap;
    hw.latchedInterrupt = true;
    hw.maskDuringAccelSettling = true;
    hw.thresholdX = config_.threshold;
    hw.thresholdY = config_.threshold;
    hw.thresholdZ = config_.threshold;
    hw.priority = 0;
    hw.shock = config_.shock;
    hw.quiet = config_.quiet;
    hw.duration = config_.duration;
    return hw;
}

TapAccumulatorConfig TapRuntimeController::makeAccumulatorConfig() const {
    TapAccumulatorConfig out;
    out.minCount = config_.minCount;
    out.maxCount = config_.maxCount;
    out.aggregationWindowMs = config_.aggregationWindowMs;
    out.slidingWindow = config_.slidingWindow;
    out.duplicateSuppressMs = config_.duplicateSuppressMs;
    out.postSendLockoutMs = config_.postSendLockoutMs;
    return out;
}

bool TapRuntimeController::configureHardware() {
    if (!lsm_) return false;

    const Lsm6dsv::TapConfig hw = makeHardwareConfig();
    const bool ok = lsm_->configureTapDetection(hw);
    status_.hardwareConfigured = ok && config_.enabled;
    status_.lastReadOk = ok;
    if (!ok) {
        ++status_.configureFailures;
        emitDiagnostic(TapDiagnosticKind::HardwareConfigureFailed, 0);
        return false;
    }

    nextPollMs_ = 0;
    nextRegisterVerifyMs_ = 0;
    lastReadFailureDiagnosticMs_ = 0;
    if (config_.enabled) {
        const bool verifyOk = verifyHardware(0, true);
        if (verifyOk) {
            emitDiagnostic(TapDiagnosticKind::HardwareConfigured, 0);
        }
        return verifyOk;
    }

    status_.registerVerifyOk = false;
    status_.lastRegisterReadOk = false;
    emitDiagnostic(TapDiagnosticKind::HardwareDisabled, 0);
    return ok;
}

bool TapRuntimeController::verifyHardware(uint32_t nowMs, bool force) {
    if (!lsm_ || !config_.enabled || !status_.hardwareConfigured) return false;
    if (!force && config_.registerVerifyIntervalMs == 0) return true;

    Lsm6dsv::TapRegisterVerification verification;
    const bool readOk = lsm_->verifyTapDetection(makeHardwareConfig(), verification);
    status_.lastRegisterReadOk = readOk;
    status_.lastRegisterVerifyMs = nowMs;
    if (!readOk) {
        status_.registerVerifyOk = false;
        status_.hardwareConfigured = false;
        ++status_.registerVerifyFailures;
        emitDiagnostic(TapDiagnosticKind::RegisterVerifyFailed, nowMs);
    } else {
        status_.lastRegisterVerification = verification;
        status_.registerVerifyOk = verification.ok;
        if (!verification.ok) {
            ++status_.registerMismatchCount;
            status_.hardwareConfigured = false;
            emitDiagnostic(TapDiagnosticKind::RegisterVerifyFailed, nowMs);
        }
    }

    nextRegisterVerifyMs_ = config_.registerVerifyIntervalMs == 0
        ? 0
        : nowMs + config_.registerVerifyIntervalMs;
    return readOk && status_.registerVerifyOk;
}

void TapRuntimeController::maybeVerifyHardware(uint32_t nowMs) {
    if (config_.registerVerifyIntervalMs == 0 || accumulator_.pending()) return;
    if (nextRegisterVerifyMs_ == 0 || timeReached(nowMs, nextRegisterVerifyMs_)) {
        (void)verifyHardware(nowMs, false);
    }
}

bool TapRuntimeController::update(uint32_t nowMs) {
    if (!config_.enabled || !lsm_) return false;

    bool worked = flushAccumulator(nowMs);

    if (config_.pollIntervalMs > 0 && nextPollMs_ != 0 && !timeReached(nowMs, nextPollMs_)) {
        return worked;
    }
    nextPollMs_ = nowMs + (config_.pollIntervalMs == 0 ? 1u : config_.pollIntervalMs);

    if (!status_.hardwareConfigured) {
        worked = configureHardware() || worked;
        if (!status_.hardwareConfigured) return worked;
    }

    Lsm6dsv::TapSource source;
    if (!lsm_->readTapSource(source)) {
        const bool wasReadOk = status_.lastReadOk;
        status_.lastReadOk = false;
        ++status_.readFailures;
        if (wasReadOk || lastReadFailureDiagnosticMs_ == 0 ||
            timeReached(nowMs, lastReadFailureDiagnosticMs_ + 1000u)) {
            lastReadFailureDiagnosticMs_ = nowMs;
            emitDiagnostic(TapDiagnosticKind::SourceReadFailed, nowMs);
        }
        return true;
    }
    worked = true;
    status_.lastReadOk = true;
    status_.lastRawSource = source.raw;

    // `TAP_SRC` is normally zero. Log every nonzero source byte because it
    // is the decisive evidence for whether the hardware engine sees impact.
    if (source.raw != 0) {
        emitDiagnostic(TapDiagnosticKind::SourceObserved, nowMs, &source);
    }

    if (source.tapDetected || source.singleTap || source.doubleTap) {
        worked = handleSource(source, nowMs, false) || worked;
    }

    maybeVerifyHardware(nowMs);
    return worked;
}

bool TapRuntimeController::flushAccumulator(uint32_t nowMs) {
    const TapAccumulatorStatus before = accumulator_.status();
    const TapAccumulatorAction action = accumulator_.update(nowMs);
    const TapAccumulatorStatus after = accumulator_.status();

    if (after.windowsFlushed != before.windowsFlushed &&
        !action.send &&
        after.suppressedBelowMin != before.suppressedBelowMin) {
        emitDiagnostic(TapDiagnosticKind::SuppressedBelowMin,
                       nowMs,
                       nullptr,
                       0,
                       0,
                       before.pendingCount);
    }
    return handleAccumulatorAction(action, nowMs, false);
}

bool TapRuntimeController::sendManualTap(uint8_t value, uint32_t nowMs) {
    value = normalizeTapValue(value);
    status_.lastEventMs = nowMs;
    status_.lastPhysicalCount = value;
    status_.lastValue = value;
    emitDiagnostic(TapDiagnosticKind::PacketReady, nowMs, nullptr, value, value, 0, true);
    return sendTap(value, nowMs, true);
}

bool TapRuntimeController::injectPhysicalTaps(uint8_t count, uint32_t nowMs) {
    if (count == 0) return false;
    bool anySent = false;
    const uint32_t stepMs = static_cast<uint32_t>(config_.duplicateSuppressMs) + 1u;
    uint32_t eventMs = nowMs;
    for (uint8_t i = 0; i < count; ++i) {
        if (handleSource(Lsm6dsv::TapSource{0x60u, true, true, false, false, false, false, true}, eventMs, true)) {
            anySent = true;
        }
        eventMs += stepMs;
    }
    if (flushAccumulator(eventMs + config_.aggregationWindowMs)) {
        anySent = true;
    }
    return anySent;
}

bool TapRuntimeController::handleSource(const Lsm6dsv::TapSource& source, uint32_t nowMs, bool manual) {
    uint8_t physicalCount = 0;
    if (source.doubleTap) {
        physicalCount = 2;
        ++status_.doubleDetected;
    } else if (source.singleTap) {
        physicalCount = 1;
        ++status_.singleDetected;
    } else if (source.tapDetected) {
        physicalCount = 1;
        ++status_.tapDetectedNoType;
    } else {
        return false;
    }

    status_.lastEventMs = nowMs;
    status_.lastPhysicalCount = physicalCount;
    emitDiagnostic(TapDiagnosticKind::PhysicalTap, nowMs, &source, physicalCount, 0, accumulator_.status().pendingCount, manual);

    const TapAccumulatorStatus before = accumulator_.status();
    const TapAccumulatorAction action = accumulator_.recordPhysicalTaps(physicalCount, nowMs);
    const TapAccumulatorStatus after = accumulator_.status();

    if (after.suppressedDuplicate != before.suppressedDuplicate) {
        emitDiagnostic(TapDiagnosticKind::SuppressedDuplicate,
                       nowMs,
                       &source,
                       physicalCount,
                       0,
                       after.pendingCount,
                       manual);
    } else if (after.suppressedLockout != before.suppressedLockout) {
        emitDiagnostic(TapDiagnosticKind::SuppressedLockout,
                       nowMs,
                       &source,
                       physicalCount,
                       0,
                       after.pendingCount,
                       manual);
    } else if (!action.send && after.pendingCount != before.pendingCount) {
        emitDiagnostic(TapDiagnosticKind::AccumulatorQueued,
                       nowMs,
                       &source,
                       physicalCount,
                       0,
                       after.pendingCount,
                       manual);
    }

    return handleAccumulatorAction(action, nowMs, manual);
}

bool TapRuntimeController::handleAccumulatorAction(const TapAccumulatorAction& action, uint32_t nowMs, bool manual) {
    if (!action.send) return false;
    status_.lastValue = action.value;
    emitDiagnostic(TapDiagnosticKind::PacketReady, nowMs, nullptr, 0, action.value, 0, manual);
    return sendTap(action.value, nowMs, manual);
}

uint8_t TapRuntimeController::normalizeTapValue(uint8_t value) {
    const uint8_t minValue = TRACKER_TAP_VALUE_MIN == 0 ? 1 : TRACKER_TAP_VALUE_MIN;
    const uint8_t maxValue = TRACKER_TAP_VALUE_MAX == 0 ? minValue : TRACKER_TAP_VALUE_MAX;
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

bool TapRuntimeController::sendTap(uint8_t value, uint32_t nowMs, bool manual) {
    if (!slimevr_ || !slimevr_->serverFound()) {
        ++status_.noServer;
        status_.lastSentOk = false;
        emitDiagnostic(TapDiagnosticKind::SlimeVrNoServer, nowMs, nullptr, 0, value, 0, manual);
        return false;
    }

    const bool useUserAction = !manual && config_.physicalTapUserAction != SlimeVRUserAction::None;
    const bool ok = useUserAction
        ? slimevr_->sendUserAction(config_.physicalTapUserAction)
        : slimevr_->sendTap(value);
    status_.lastSentOk = ok;
    if (ok) {
        if (useUserAction) ++status_.userActionsSent;
        ++status_.sent;
        status_.lastSentMs = nowMs;
        emitDiagnostic(TapDiagnosticKind::SlimeVrSent, nowMs, nullptr, 0, value, 0, manual);
    } else {
        if (useUserAction) ++status_.userActionFailures;
        ++status_.sendFailures;
        emitDiagnostic(TapDiagnosticKind::SlimeVrSendFailed, nowMs, nullptr, 0, value, 0, manual);
    }
    return ok;
}

void TapRuntimeController::emitDiagnostic(TapDiagnosticKind kind,
                                          uint32_t nowMs,
                                          const Lsm6dsv::TapSource* source,
                                          uint8_t physicalCount,
                                          uint8_t packetValue,
                                          uint8_t pendingCount,
                                          bool manual) {
    if (!diagnosticLogging_ || !diagnosticSink_) return;

    TapDiagnosticEvent event;
    event.kind = kind;
    event.atMs = nowMs;
    event.physicalCount = physicalCount;
    event.packetValue = packetValue;
    event.pendingCount = pendingCount;
    event.manual = manual;
    if (source) {
        event.rawSource = source->raw;
        event.tapDetected = source->tapDetected;
        event.singleTap = source->singleTap;
        event.doubleTap = source->doubleTap;
        event.negative = source->negative;
        event.x = source->x;
        event.y = source->y;
        event.z = source->z;
    }

    ++diagnosticEvents_;
    diagnosticSink_(event, diagnosticSinkUser_);
}

} // namespace tracker

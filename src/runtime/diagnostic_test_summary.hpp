#pragma once

#include <Arduino.h>
#include <cstdint>

namespace tracker {

enum class DiagnosticTestKind : uint8_t {
    Static,
    Runtime,
};

inline const char* diagnosticTestKindName(DiagnosticTestKind kind) {
    return kind == DiagnosticTestKind::Runtime ? "runtime" : "static";
}

// Compact immutable completion evidence. It is requested only after the
// measured window has closed, so exact full-rate test counters can enter the
// strict capture without formatting the retained multi-page report in FIFO,
// IMU, or ordinary loop hot paths.
struct DiagnosticTestSummary {
    DiagnosticTestKind kind = DiagnosticTestKind::Static;
    uint32_t durationMs = 0;
    bool stoppedByCommand = false;
    uint32_t samples = 0;
    uint32_t hwTimestampSamples = 0;
    uint32_t fallbackTimestampSamples = 0;
    uint32_t badTimestampSamples = 0;
    uint32_t estimatedDroppedSamples = 0;
    uint32_t recoveryRequests = 0;
    uint32_t fifoOverruns = 0;
    uint32_t fifoFull = 0;
    uint32_t fifoUnknown = 0;

    bool networkMetricsValid = false;
    uint32_t wifiDisconnects = 0;
    uint32_t wifiConnectTimeouts = 0;
    uint32_t udpSendFailures = 0;
    uint32_t rotationSendFailures = 0;
    uint32_t rotationMissedDeadlines = 0;
    uint32_t rotationLateEvents = 0;
    uint32_t txPressureFailures = 0;
    uint32_t txOtherFailures = 0;
    uint32_t udpRebindSuccesses = 0;
    uint32_t udpRebindFailures = 0;
    uint32_t udpFullReopens = 0;

    bool staticMetricsValid = false;
    float gyroMeanDps = 0.0f;
    float gyroStdDps = 0.0f;
    float accelNormMeanG = 0.0f;
    float accelNormStdG = 0.0f;
    float tempStartC = 0.0f;
    float tempEndC = 0.0f;

    bool magMetricsValid = false;
    uint32_t magTrusted = 0;
    uint32_t magRejected = 0;
};

inline void diagnosticTestPrintSummary(Stream& out,
                                       const DiagnosticTestSummary& s) {
    out.print("TESTSUM,"); out.print(diagnosticTestKindName(s.kind));
    out.print(','); out.print(s.durationMs);
    out.print(','); out.print(s.stoppedByCommand ? 1 : 0);
    out.print(','); out.print(s.samples);
    out.print(','); out.print(s.hwTimestampSamples);
    out.print(','); out.print(s.fallbackTimestampSamples);
    out.print(','); out.print(s.badTimestampSamples);
    out.print(','); out.print(s.estimatedDroppedSamples);
    out.print(','); out.print(s.recoveryRequests);
    out.print(','); out.print(s.fifoOverruns);
    out.print(','); out.print(s.fifoFull);
    out.print(','); out.print(s.fifoUnknown);
    out.print(','); out.print(s.networkMetricsValid ? 1 : 0);
    out.print(','); out.print(s.wifiDisconnects);
    out.print(','); out.print(s.wifiConnectTimeouts);
    out.print(','); out.print(s.udpSendFailures);
    out.print(','); out.print(s.rotationSendFailures);
    out.print(','); out.print(s.rotationMissedDeadlines);
    out.print(','); out.print(s.rotationLateEvents);
    out.print(','); out.print(s.txPressureFailures);
    out.print(','); out.print(s.txOtherFailures);
    out.print(','); out.print(s.udpRebindSuccesses);
    out.print(','); out.print(s.udpRebindFailures);
    out.print(','); out.print(s.udpFullReopens);
    out.print(','); out.print(s.staticMetricsValid ? 1 : 0);
    out.print(','); out.print(s.gyroMeanDps, 8);
    out.print(','); out.print(s.gyroStdDps, 8);
    out.print(','); out.print(s.accelNormMeanG, 8);
    out.print(','); out.print(s.accelNormStdG, 8);
    out.print(','); out.print(s.tempStartC, 3);
    out.print(','); out.print(s.tempEndC, 3);
    out.print(','); out.print(s.magMetricsValid ? 1 : 0);
    out.print(','); out.print(s.magTrusted);
    out.print(','); out.println(s.magRejected);
}

} // namespace tracker

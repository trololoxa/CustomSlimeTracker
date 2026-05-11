#pragma once

#include <Arduino.h>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>

#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "connection/lsm6dsv_sensorhub.hpp"
#include "sensor/qmc6309.hpp"
#include "sensor/calibration.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/imu_quality.hpp"
#include "sensor/fifo_calibrations.hpp"
#include "config/tracker_config.hpp"

namespace tracker {

// ============================================================
// Lightweight serial command protocol
// ============================================================
// Design goals:
//   - no heap allocation
//   - no String
//   - non-blocking poll(), optionally limited by bytes per loop
//   - nearly zero CPU load when Serial has no bytes
//   - text commands for development/config/calibration
//   - future binary/quaternion output can coexist separately
//
// Usage in main.cpp:
//   static TrackerSerialCommandInterface<> g_cli;
//   static TrackerSerialCommandContext g_cmdCtx;
//
//   void setup() {
//       g_cmdCtx.io = &Serial;
//       g_cmdCtx.config = &g_config;
//       ... fill pointers ...
//       g_cli.begin(g_cmdCtx);
//   }
//
//   void loop() {
//       g_cli.poll(32);
//       // normal tracker loop
//   }
// ============================================================

namespace tracker_serial_detail {

inline char upperChar(char c) {
    if (c >= 'a' && c <= 'z') return static_cast<char>(c - 'a' + 'A');
    return c;
}

inline bool eqIgnoreCase(const char* a, const char* b) {
    if (!a || !b) return false;
    while (*a && *b) {
        if (upperChar(*a) != upperChar(*b)) return false;
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

inline bool startsWithIgnoreCase(const char* s, const char* prefix) {
    if (!s || !prefix) return false;
    while (*prefix) {
        if (upperChar(*s) != upperChar(*prefix)) return false;
        ++s;
        ++prefix;
    }
    return true;
}

inline bool parseBool(const char* s, bool& out) {
    if (!s) return false;
    if (eqIgnoreCase(s, "1") || eqIgnoreCase(s, "on") || eqIgnoreCase(s, "true") || eqIgnoreCase(s, "yes")) {
        out = true;
        return true;
    }
    if (eqIgnoreCase(s, "0") || eqIgnoreCase(s, "off") || eqIgnoreCase(s, "false") || eqIgnoreCase(s, "no")) {
        out = false;
        return true;
    }
    return false;
}

inline bool parseU32(const char* s, uint32_t& out) {
    if (!s || *s == '\0') return false;
    char* end = nullptr;
    const unsigned long v = std::strtoul(s, &end, 0);
    if (!end || *end != '\0') return false;
    out = static_cast<uint32_t>(v);
    return true;
}

inline bool parseFloat(const char* s, float& out) {
    if (!s || *s == '\0') return false;
    char* end = nullptr;
    const float v = std::strtof(s, &end);
    if (!end || *end != '\0' || !std::isfinite(v)) return false;
    out = v;
    return true;
}


inline void printU64Dec(Stream& out, uint64_t v) {
    char buf[21];
    size_t i = sizeof(buf);
    buf[--i] = '\0';
    if (v == 0) {
        buf[--i] = '0';
    } else {
        while (v > 0 && i > 0) {
            buf[--i] = static_cast<char>('0' + (v % 10));
            v /= 10;
        }
    }
    out.print(&buf[i]);
}

inline void printVec3(Stream& out, const char* label, const Vec3& v, uint8_t decimals = 6) {
    out.print(label);
    out.print(" x="); out.print(v.x, decimals);
    out.print(" y="); out.print(v.y, decimals);
    out.print(" z="); out.print(v.z, decimals);
}

inline void printVec3Line(Stream& out, const char* label, const Vec3& v, uint8_t decimals = 6) {
    printVec3(out, label, v, decimals);
    out.println();
}

inline void printQuatLine(Stream& out, const char* label, const Quat& q, uint8_t decimals = 7) {
    out.print(label);
    out.print(" w="); out.print(q.w, decimals);
    out.print(" x="); out.print(q.x, decimals);
    out.print(" y="); out.print(q.y, decimals);
    out.print(" z="); out.println(q.z, decimals);
}

inline void printOk(Stream& out, const char* msg = nullptr) {
    out.print("# OK");
    if (msg && *msg) {
        out.print(' ');
        out.print(msg);
    }
    out.println();
}

inline void printErr(Stream& out, const char* msg) {
    out.print("# ERR ");
    out.println(msg ? msg : "unknown");
}

} // namespace tracker_serial_detail

enum class TrackerStreamMode : uint8_t {
    Off,
    Raw,
    Scaled,
    Quat,
    Debug,
    Heartbeat
};

struct TrackerSerialStreamState {
    TrackerStreamMode mode = TrackerStreamMode::Off;
    uint16_t rateHz = 100;
    uint32_t lastEmitUs = 0;

    uint32_t periodUs() const {
        const uint16_t hz = rateHz == 0 ? 1 : rateHz;
        return 1000000UL / hz;
    }
};

enum class TrackerLogMode : uint8_t {
    Off,
    Basic,
    Full
};

struct TrackerSerialLogState {
    TrackerLogMode mode = TrackerLogMode::Off;
    uint16_t rateHz = 20;
    uint32_t lastEmitUs = 0;
    uint32_t lastMagEmitUs = 0;
    uint32_t sequence = 0;

    bool enabled() const {
        return mode != TrackerLogMode::Off;
    }

    uint32_t periodUs() const {
        const uint16_t hz = rateHz == 0 ? 1 : rateHz;
        return 1000000UL / hz;
    }
};

struct TrackerSerialCommandContext {
    Stream* io = nullptr;

    TrackerConfig* config = nullptr;
    TrackerConfigStore* configStore = nullptr;

    Lsm6dsv* lsm = nullptr;
    Lsm6dsvFifoReader* fifo = nullptr;
    Lsm6dsvSensorHub* sensorHub = nullptr;
    Qmc6309* mag = nullptr;
    ImuCalibration* imuCal = nullptr;
    GyroTempCompensator* gyroTempComp = nullptr;
    ImuQualityMonitor* quality = nullptr;
    Ahrs6Dof* ahrs = nullptr;

    FifoCalibrationIo* calibrationIo = nullptr;
    FifoAccel6PosCalibrationRunner* accelCalRunner = nullptr;

    TrackerSerialStreamState* streamState = nullptr;
    TrackerSerialLogState* logState = nullptr;

    // Optional hooks supplied by main.cpp.
    void (*resetFifoRuntime)(void* user) = nullptr;
    void* resetFifoRuntimeUser = nullptr;

    void (*resetAhrsRuntime)(void* user) = nullptr;
    void* resetAhrsRuntimeUser = nullptr;

    void (*printRuntimeStatus)(Stream& out, void* user) = nullptr;
    void* printRuntimeStatusUser = nullptr;

    void (*printRuntimeHealth)(Stream& out, void* user) = nullptr;
    void* printRuntimeHealthUser = nullptr;

    bool (*setSpiFrequency)(uint32_t hz, void* user) = nullptr;
    void* setSpiFrequencyUser = nullptr;

    void (*emitLogHeader)(Stream& out, void* user) = nullptr;
    void* emitLogHeaderUser = nullptr;

    void (*printLogSummary)(Stream& out, void* user) = nullptr;
    void* printLogSummaryUser = nullptr;

    void (*resetLogCounters)(void* user) = nullptr;
    void* resetLogCountersUser = nullptr;

    void (*printRuntimeGyroBiasStatus)(Stream& out, void* user) = nullptr;
    void* printRuntimeGyroBiasStatusUser = nullptr;

    bool (*setRuntimeGyroBiasEnabled)(bool enabled, void* user) = nullptr;
    void* setRuntimeGyroBiasEnabledUser = nullptr;

    void (*resetRuntimeGyroBiasEstimator)(void* user) = nullptr;
    void* resetRuntimeGyroBiasEstimatorUser = nullptr;

    bool (*startStaticTest)(uint32_t durationMs, void* user) = nullptr;
    void* startStaticTestUser = nullptr;

    bool (*stopStaticTest)(void* user) = nullptr;
    void* stopStaticTestUser = nullptr;

    void (*printStaticTestStatus)(Stream& out, void* user) = nullptr;
    void* printStaticTestStatusUser = nullptr;

    bool (*setMagRuntimeEnabled)(bool enabled, bool persist, void* user) = nullptr;
    void* setMagRuntimeEnabledUser = nullptr;

    void (*printMagRuntimeStatus)(Stream& out, void* user) = nullptr;
    void* printMagRuntimeStatusUser = nullptr;

    void (*printMagProcessedStatus)(Stream& out, void* user) = nullptr;
    void* printMagProcessedStatusUser = nullptr;

    void (*printMagHeadingStatus)(Stream& out, void* user) = nullptr;
    void* printMagHeadingStatusUser = nullptr;

    bool (*setMagHeadingReference)(void* user) = nullptr;
    void* setMagHeadingReferenceUser = nullptr;

    void (*clearMagHeadingReference)(void* user) = nullptr;
    void* clearMagHeadingReferenceUser = nullptr;

    bool (*setMagHeadingAutoReferenceEnabled)(bool enabled, void* user) = nullptr;
    void* setMagHeadingAutoReferenceEnabledUser = nullptr;

    void (*printMagYawCorrectionStatus)(Stream& out, void* user) = nullptr;
    void* printMagYawCorrectionStatusUser = nullptr;

    void (*resetMagYawCorrection)(void* user) = nullptr;
    void* resetMagYawCorrectionUser = nullptr;

    bool (*setMagYawCorrectionApplyEnabled)(bool enabled, bool persist, void* user) = nullptr;
    void* setMagYawCorrectionApplyEnabledUser = nullptr;

    bool (*startMagCalibration)(void* user) = nullptr;
    void* startMagCalibrationUser = nullptr;

    void (*stopMagCalibration)(void* user) = nullptr;
    void* stopMagCalibrationUser = nullptr;

    void (*resetMagCalibration)(void* user) = nullptr;
    void* resetMagCalibrationUser = nullptr;

    bool (*applyMagCalibration)(bool persist, void* user) = nullptr;
    void* applyMagCalibrationUser = nullptr;

    void (*printMagCalibrationStatus)(Stream& out, void* user) = nullptr;
    void* printMagCalibrationStatusUser = nullptr;

    bool (*fitGyroTempFromLastStatic)(bool persist, Stream& out, void* user) = nullptr;
    void* fitGyroTempFromLastStaticUser = nullptr;
};


} // namespace tracker

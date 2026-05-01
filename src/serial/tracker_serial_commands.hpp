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
//   - non-blocking poll()
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
//       g_cli.poll();
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

    // Optional hooks supplied by main.cpp.
    void (*resetFifoRuntime)(void* user) = nullptr;
    void* resetFifoRuntimeUser = nullptr;

    void (*resetAhrsRuntime)(void* user) = nullptr;
    void* resetAhrsRuntimeUser = nullptr;

    void (*printRuntimeStatus)(Stream& out, void* user) = nullptr;
    void* printRuntimeStatusUser = nullptr;

    void (*printRuntimeHealth)(Stream& out, void* user) = nullptr;
    void* printRuntimeHealthUser = nullptr;

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

    void (*printMagYawCorrectionStatus)(Stream& out, void* user) = nullptr;
    void* printMagYawCorrectionStatusUser = nullptr;

    void (*resetMagYawCorrection)(void* user) = nullptr;
    void* resetMagYawCorrectionUser = nullptr;

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
};

class TrackerCommandDispatcher {
public:
    static void dispatch(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (argc <= 0 || !argv || !argv[0]) return;

        if (is(argv[0], "help") || is(argv[0], "?")) {
            cmdHelp(out);
            return;
        }

        if (is(argv[0], "status")) {
            cmdStatus(ctx);
            return;
        }

        if (is(argv[0], "health")) {
            cmdHealth(ctx);
            return;
        }

        if (is(argv[0], "version")) {
            out.println("# tracker firmware proto=serial-cli-v1");
            return;
        }

        if (is(argv[0], "reboot")) {
            out.println("# OK rebooting");
            out.flush();
            delay(50);
            ESP.restart();
            return;
        }

        if (is(argv[0], "factory_reset")) {
            cmdFactoryReset(ctx);
            return;
        }

        if (is(argv[0], "config")) {
            cmdConfig(ctx, argc, argv);
            return;
        }

        if (is(argv[0], "mag")) {
            cmdMag(ctx, argc, argv);
            return;
        }

        if (is(argv[0], "fifo")) {
            cmdFifo(ctx, argc, argv);
            return;
        }

        if (is(argv[0], "imu")) {
            cmdImu(ctx, argc, argv);
            return;
        }

        if (is(argv[0], "quality")) {
            cmdQuality(ctx, argc, argv);
            return;
        }

        if (is(argv[0], "cal")) {
            cmdCal(ctx, argc, argv);
            return;
        }

        if (is(argv[0], "ahrs")) {
            cmdAhrs(ctx, argc, argv);
            return;
        }

        if (is(argv[0], "stream")) {
            cmdStream(ctx, argc, argv);
            return;
        }

        if (is(argv[0], "test")) {
            cmdTest(ctx, argc, argv);
            return;
        }

        if (is(argv[0], "output")) {
            cmdOutput(ctx, argc, argv);
            return;
        }

        tracker_serial_detail::printErr(out, "unknown command; type help");
    }

private:
    static Stream& stream(TrackerSerialCommandContext& ctx) {
        return ctx.io ? *ctx.io : Serial;
    }

    static bool is(const char* a, const char* b) {
        return tracker_serial_detail::eqIgnoreCase(a, b);
    }

    static void cmdHelp(Stream& out) {
        out.println("==============================================================================");
        out.println("TRACKER SERIAL COMMANDS");
        out.println("==============================================================================");
        out.println("help | ?");
        out.println("status");
        out.println("health");
        out.println("version");
        out.println("reboot");
        out.println("factory_reset");
        out.println();
        out.println("config print | load | save | defaults | erase | crc");
        out.println();
        out.println("imu status | whoami | read");
        out.println("fifo status | stats | reset");
        out.println("mag status | enable [save] | disable [save]");
        out.println("mag id | qmcstatus | regs | hub | fifo");
        out.println("mag processed | trust");
        out.println("mag heading | heading ref | heading status | heading clear");
        out.println("mag yaw status | yaw reset");
        out.println("mag axis print | set <bodyX> <bodyY> <bodyZ> [save]");
        out.println("mag axis identity [save] | clear [save]");
        out.println("mag axis examples: set +x +y +z | set +y -x +z");
        out.println("mag cal start | stop | reset | status | print | apply [save]");
        out.println("quality stats | reset");
        out.println();
        out.println("cal gyro");
        out.println("cal gyro save | clear");
        out.println("cal accel face XP|XN|YP|YN|ZP|ZN");
        out.println("cal accel compute | dump | save | clear");
        out.println("cal temp print | set_slope X Y Z | clear");
        out.println("cal save | clear_all");
        out.println();
        out.println("ahrs status | reset");
        out.println();
        out.println("stream off | heartbeat | raw | scaled | quat | debug");
        out.println("stream rate <hz>");
        out.println();
        out.println("test static <seconds>");
        out.println("test stop");
        out.println("test status");
        out.println();
        out.println("output mode debug|binary|slimevr");
        out.println("output rate <hz>");
        out.println("output start | stop");
        out.println("==============================================================================");
    }

    static void cmdStatus(TrackerSerialCommandContext& ctx) {
        Stream& out = stream(ctx);
        out.println("# STATUS");

        if (ctx.printRuntimeStatus) {
            ctx.printRuntimeStatus(out, ctx.printRuntimeStatusUser);
            return;
        }

        if (ctx.lsm) {
            out.print("lsm_initialized="); out.println(ctx.lsm->isInitialized() ? "yes" : "no");
            out.print("lsm_last_error="); out.println(static_cast<int>(ctx.lsm->lastError()));
            out.print("lsm_who=0x"); out.println(ctx.lsm->lastWhoAmI(), HEX);
        }

        if (ctx.config) {
            out.print("config_valid="); out.println(ctx.config->validate() ? "yes" : "no");
        }

        if (ctx.imuCal) {
            out.print("gyro_bias_valid="); out.println(ctx.imuCal->gyroBiasValid ? "yes" : "no");
            out.print("accel_cal_valid="); out.println(ctx.imuCal->accelCalValid ? "yes" : "no");
        }

        if (ctx.fifo) {
            const auto& fs = ctx.fifo->stats();
            out.print("fifo_samples="); out.println(fs.imuSamplesProduced);
            out.print("fifo_hw_ts="); out.println(fs.hwTimestampAssigned);
            out.print("fifo_fb_ts="); out.println(fs.fallbackTimestampAssigned);
            out.print("fifo_ovr="); out.println(fs.overrunEvents);
            out.print("fifo_full="); out.println(fs.fullEvents);
            out.print("fifo_unknown="); out.println(fs.unknownWords);
        }

        if (ctx.streamState) {
            out.print("stream_mode="); out.println(streamModeName(ctx.streamState->mode));
            out.print("stream_rate_hz="); out.println(ctx.streamState->rateHz);
        }
    }

    static void cmdHealth(TrackerSerialCommandContext& ctx) {
        Stream& out = stream(ctx);
        out.println("# HEALTH");

        if (ctx.printRuntimeHealth) {
            ctx.printRuntimeHealth(out, ctx.printRuntimeHealthUser);
            return;
        }

        cmdStatus(ctx);
        if (ctx.fifo) printFifoStats(out, ctx.fifo->stats());
        if (ctx.quality) printQualityStats(out, ctx.quality->counters());
    }

    static void cmdFactoryReset(TrackerSerialCommandContext& ctx) {
        Stream& out = stream(ctx);
        bool ok = true;

        if (ctx.config) {
            ctx.config->resetDefaults();
        }
        if (ctx.configStore) {
            ok = ctx.configStore->erase();
        }

        if (ok) tracker_serial_detail::printOk(out, "factory reset done; reboot recommended");
        else {
            out.print("# ERR factory reset failed: ");
            out.println(ctx.configStore ? ctx.configStore->lastErrorName() : "no config store");
        }
    }

    static void cmdConfig(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.config) {
            tracker_serial_detail::printErr(out, "config not available");
            return;
        }
        if (argc < 2) {
            tracker_serial_detail::printErr(out, "usage: config print|load|save|defaults|erase|crc");
            return;
        }

        if (is(argv[1], "print")) {
            printTrackerConfigSummary(out, *ctx.config);
            return;
        }

        if (is(argv[1], "crc")) {
            out.print("# config_valid="); out.println(ctx.config->validate() ? "yes" : "no");
            out.print("# stored_crc=0x"); out.println(ctx.config->data.crc32, HEX);
            out.print("# computed_crc=0x"); out.println(ctx.config->computeCrc(), HEX);
            return;
        }

        if (is(argv[1], "defaults")) {
            ctx.config->resetDefaults();
            applyConfigToRuntime(ctx);
            tracker_serial_detail::printOk(out, "config defaults loaded into RAM");
            return;
        }

        if (!ctx.configStore) {
            tracker_serial_detail::printErr(out, "config store not available");
            return;
        }

        if (is(argv[1], "load")) {
            if (ctx.configStore->load(*ctx.config)) {
                applyConfigToRuntime(ctx);
                tracker_serial_detail::printOk(out, "config loaded from NVS");
            } else {
                out.print("# ERR config load failed: ");
                out.println(ctx.configStore->lastErrorName());
            }
            return;
        }

        if (is(argv[1], "save")) {
            captureRuntimeToConfig(ctx);
            if (ctx.configStore->save(*ctx.config)) {
                tracker_serial_detail::printOk(out, "config saved to NVS");
            } else {
                out.print("# ERR config save failed: ");
                out.println(ctx.configStore->lastErrorName());
            }
            return;
        }

        if (is(argv[1], "erase")) {
            if (ctx.configStore->erase()) {
                tracker_serial_detail::printOk(out, "config erased from NVS");
            } else {
                out.print("# ERR config erase failed: ");
                out.println(ctx.configStore->lastErrorName());
            }
            return;
        }

        tracker_serial_detail::printErr(out, "unknown config command");
    }

    static void cmdImu(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.lsm) {
            tracker_serial_detail::printErr(out, "imu not available");
            return;
        }
        if (argc < 2 || is(argv[1], "status")) {
            out.print("imu_initialized="); out.println(ctx.lsm->isInitialized() ? "yes" : "no");
            out.print("imu_last_who=0x"); out.println(ctx.lsm->lastWhoAmI(), HEX);
            out.print("imu_last_error="); out.println(static_cast<int>(ctx.lsm->lastError()));
            return;
        }

        if (is(argv[1], "whoami")) {
            uint8_t who = 0;
            const bool ok = ctx.lsm->readWhoAmI(who);
            out.print(ok ? "# OK WHO_AM_I=0x" : "# ERR WHO_AM_I read failed, last=0x");
            out.println(who, HEX);
            return;
        }

        if (is(argv[1], "read")) {
            Lsm6dsv::Sample s;
            if (!ctx.lsm->readSample(s, micros(), true)) {
                tracker_serial_detail::printErr(out, "imu read failed");
                return;
            }
            tracker_serial_detail::printVec3(out, "accel_g", s.accel_g, 6);
            out.print(" norm="); out.println(s.accel_g.norm(), 6);
            tracker_serial_detail::printVec3(out, "gyro_rad_s", s.gyro_rad_s, 7);
            out.print(" norm="); out.println(s.gyro_rad_s.norm(), 7);
            out.print("temp_c="); out.println(s.temp_c, 3);
            return;
        }

        tracker_serial_detail::printErr(out, "unknown imu command");
    }

    static bool magSaveConfigIfRequested(TrackerSerialCommandContext& ctx, bool saveRequested) {
        if (!saveRequested) return true;
        if (!ctx.config || !ctx.configStore) return false;
        ctx.config->updateCrc();
        return ctx.configStore->save(*ctx.config);
    }

    static void rearmMagIfNeeded(TrackerSerialCommandContext& ctx) {
        if (!ctx.config || !ctx.config->data.magCal.driverEnabled) return;
        if (!ctx.setMagRuntimeEnabled) return;
        ctx.setMagRuntimeEnabled(true, false, ctx.setMagRuntimeEnabledUser);
    }

    static char magAxisLower(char c) {
        if (c >= 'A' && c <= 'Z') return static_cast<char>(c - 'A' + 'a');
        return c;
    }

    static bool parseMagAxisToken(const char* token, uint8_t& axis, float& sign) {
        if (token == nullptr || token[0] == '\0') return false;

        sign = 1.0f;
        uint8_t pos = 0;

        if (token[pos] == '+') {
            sign = 1.0f;
            pos++;
        } else if (token[pos] == '-') {
            sign = -1.0f;
            pos++;
        }

        const char a = magAxisLower(token[pos]);
        if (a == '\0' || token[pos + 1] != '\0') return false;

        if (a == 'x') {
            axis = 0;
            return true;
        }
        if (a == 'y') {
            axis = 1;
            return true;
        }
        if (a == 'z') {
            axis = 2;
            return true;
        }

        return false;
    }

    static const char* magAxisNameFromIndex(uint8_t axis) {
        switch (axis) {
            case 0: return "x";
            case 1: return "y";
            case 2: return "z";
        }
        return "?";
    }

    static void printMagAxisToken(Stream& out, const Mat3& m, uint8_t row) {
        uint8_t nonZeroCount = 0;
        uint8_t axis = 0;
        float sign = 1.0f;

        for (uint8_t col = 0; col < 3; ++col) {
            const float v = m.m[row][col];
            if (std::fabs(v) > 0.5f) {
                nonZeroCount++;
                axis = col;
                sign = v >= 0.0f ? 1.0f : -1.0f;
            }
        }

        if (nonZeroCount != 1) {
            out.print("?");
            return;
        }

        out.print(sign >= 0.0f ? "+" : "-");
        out.print(magAxisNameFromIndex(axis));
    }

    static bool makeMagAxisMatrixFromTokens(const char* bodyXToken,
                                            const char* bodyYToken,
                                            const char* bodyZToken,
                                            Mat3& out) {
        uint8_t axis[3] = {};
        float sign[3] = {};

        if (!parseMagAxisToken(bodyXToken, axis[0], sign[0])) return false;
        if (!parseMagAxisToken(bodyYToken, axis[1], sign[1])) return false;
        if (!parseMagAxisToken(bodyZToken, axis[2], sign[2])) return false;

        // Must be a pure permutation/sign matrix:
        // body.x, body.y, body.z must use each mag axis exactly once.
        bool used[3] = {false, false, false};
        for (uint8_t i = 0; i < 3; ++i) {
            if (axis[i] > 2) return false;
            if (used[axis[i]]) return false;
            used[axis[i]] = true;
        }

        out = Mat3::zero();
        for (uint8_t row = 0; row < 3; ++row) {
            out.m[row][axis[row]] = sign[row];
        }

        return true;
    }

    static bool saveConfigIfRequested(TrackerSerialCommandContext& ctx, bool saveRequested) {
        if (!saveRequested) return true;
        if (!ctx.config || !ctx.configStore) return false;
        ctx.config->updateCrc();
        return ctx.configStore->save(*ctx.config);
    }

    static void printMagAxisMatrix(Stream& out, const TrackerConfig& config) {
        const Mat3& m = config.data.magCal.magToImu;

        out.println("# MAG AXIS");
        out.print("axisAlignmentValid=");
        out.println(config.data.magCal.axisAlignmentValid ? "yes" : "no");

        out.print("mapping=");
        printMagAxisToken(out, m, 0);
        out.print(' ');
        printMagAxisToken(out, m, 1);
        out.print(' ');
        printMagAxisToken(out, m, 2);
        out.println();

        out.print("meaning=");
        out.print("body.x=");
        printMagAxisToken(out, m, 0);
        out.print(" body.y=");
        printMagAxisToken(out, m, 1);
        out.print(" body.z=");
        printMagAxisToken(out, m, 2);
        out.println();

        out.print("magToImu_row0=");
        out.print(m.m[0][0], 6); out.print(',');
        out.print(m.m[0][1], 6); out.print(',');
        out.println(m.m[0][2], 6);

        out.print("magToImu_row1=");
        out.print(m.m[1][0], 6); out.print(',');
        out.print(m.m[1][1], 6); out.print(',');
        out.println(m.m[1][2], 6);

        out.print("magToImu_row2=");
        out.print(m.m[2][0], 6); out.print(',');
        out.print(m.m[2][1], 6); out.print(',');
        out.println(m.m[2][2], 6);
    }

    static void cmdMag(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);

        if (argc < 2 || is(argv[1], "status")) {
            out.println("# MAG STATUS");
            if (ctx.config) {
                out.print("mag_config_enabled="); out.println(ctx.config->data.magCal.driverEnabled ? "yes" : "no");
                out.print("mag_cal_valid="); out.println(ctx.config->data.magCal.calibrationValid ? "yes" : "no");
                out.print("mag_axis_valid="); out.println(ctx.config->data.magCal.axisAlignmentValid ? "yes" : "no");
            }
            if (ctx.printMagRuntimeStatus) {
                ctx.printMagRuntimeStatus(out, ctx.printMagRuntimeStatusUser);
            }
            if (ctx.fifo) {
                const auto& fs = ctx.fifo->stats();
                out.print("sensorhub0_words="); out.println(fs.sensorHubSlave0Words);
                out.print("sensorhub_nack_words="); out.println(fs.sensorHubNackWords);
                out.print("mag_samples_produced="); out.println(fs.magSamplesProduced);
                out.print("mag_last_xyz="); out.print(fs.lastMagX); out.print(','); out.print(fs.lastMagY); out.print(','); out.println(fs.lastMagZ);
                out.print("mag_last_norm_raw="); out.println(fs.lastMagRawNorm, 3);
            }
            return;
        }

        if (is(argv[1], "enable") || is(argv[1], "on")) {
            const bool saveRequested = argc >= 3 && is(argv[2], "save");
            if (ctx.config) {
                ctx.config->data.magCal.driverEnabled = true;
                ctx.config->updateCrc();
            }
            bool ok = true;
            if (ctx.setMagRuntimeEnabled) {
                ok = ctx.setMagRuntimeEnabled(true, saveRequested, ctx.setMagRuntimeEnabledUser);
            }
            if (ok && !ctx.setMagRuntimeEnabled) {
                ok = magSaveConfigIfRequested(ctx, saveRequested);
            }
            if (ok) tracker_serial_detail::printOk(out, saveRequested ? "mag enabled and saved" : "mag enabled in RAM");
            else tracker_serial_detail::printErr(out, "mag enable failed");
            return;
        }

        if (is(argv[1], "disable") || is(argv[1], "off")) {
            const bool saveRequested = argc >= 3 && is(argv[2], "save");
            if (ctx.config) {
                ctx.config->data.magCal.driverEnabled = false;
                ctx.config->updateCrc();
            }
            bool ok = true;
            if (ctx.setMagRuntimeEnabled) {
                ok = ctx.setMagRuntimeEnabled(false, saveRequested, ctx.setMagRuntimeEnabledUser);
            }
            if (ok && !ctx.setMagRuntimeEnabled) {
                ok = magSaveConfigIfRequested(ctx, saveRequested);
            }
            if (ok) tracker_serial_detail::printOk(out, saveRequested ? "mag disabled and saved" : "mag disabled in RAM");
            else tracker_serial_detail::printErr(out, "mag disable failed");
            return;
        }

        if (is(argv[1], "cal")) {
            if (argc < 3 || is(argv[2], "status") || is(argv[2], "print")) {
                if (ctx.printMagCalibrationStatus) {
                    ctx.printMagCalibrationStatus(out, ctx.printMagCalibrationStatusUser);
                } else {
                    tracker_serial_detail::printErr(out, "mag calibration status hook not available");
                }
                return;
            }

            if (is(argv[2], "start")) {
                if (!ctx.config || !ctx.config->data.magCal.driverEnabled) {
                    tracker_serial_detail::printErr(out, "enable mag first: mag enable");
                    return;
                }
                if (!ctx.startMagCalibration) {
                    tracker_serial_detail::printErr(out, "mag calibration start hook not available");
                    return;
                }
                const bool ok = ctx.startMagCalibration(ctx.startMagCalibrationUser);
                if (ok) tracker_serial_detail::printOk(out, "mag calibration collection started");
                else tracker_serial_detail::printErr(out, "mag calibration start failed");
                return;
            }

            if (is(argv[2], "stop")) {
                if (ctx.stopMagCalibration) {
                    ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
                    tracker_serial_detail::printOk(out, "mag calibration collection stopped");
                } else {
                    tracker_serial_detail::printErr(out, "mag calibration stop hook not available");
                }
                return;
            }

            if (is(argv[2], "reset")) {
                if (ctx.resetMagCalibration) {
                    ctx.resetMagCalibration(ctx.resetMagCalibrationUser);
                    tracker_serial_detail::printOk(out, "mag calibration collector reset");
                } else {
                    tracker_serial_detail::printErr(out, "mag calibration reset hook not available");
                }
                return;
            }

            if (is(argv[2], "apply")) {
                const bool saveRequested = argc >= 4 && is(argv[3], "save");
                if (!ctx.applyMagCalibration) {
                    tracker_serial_detail::printErr(out, "mag calibration apply hook not available");
                    return;
                }
                const bool ok = ctx.applyMagCalibration(saveRequested, ctx.applyMagCalibrationUser);
                if (ok) tracker_serial_detail::printOk(out, saveRequested ? "mag calibration applied and saved" : "mag calibration applied in RAM");
                else tracker_serial_detail::printErr(out, "mag calibration apply failed");
                return;
            }

            tracker_serial_detail::printErr(out, "unknown mag cal command");
            return;
        }

        if (is(argv[1], "processed") || is(argv[1], "trust")) {
            if (ctx.printMagProcessedStatus) {
                ctx.printMagProcessedStatus(out, ctx.printMagProcessedStatusUser);
            } else {
                tracker_serial_detail::printErr(out, "mag processed hook not available");
            }
            return;
        }

        if (is(argv[1], "heading")) {
            if (argc < 3 || is(argv[2], "status") || is(argv[2], "print")) {
                if (ctx.printMagHeadingStatus) {
                    ctx.printMagHeadingStatus(out, ctx.printMagHeadingStatusUser);
                } else {
                    tracker_serial_detail::printErr(out, "mag heading hook not available");
                }
                return;
            }

            if (is(argv[2], "ref")) {
                if (!ctx.setMagHeadingReference) {
                    tracker_serial_detail::printErr(out, "mag heading ref hook not available");
                    return;
                }

                const bool ok = ctx.setMagHeadingReference(ctx.setMagHeadingReferenceUser);
                if (ok) {
                    tracker_serial_detail::printOk(out, "mag heading reference set");
                    if (ctx.printMagHeadingStatus) {
                        ctx.printMagHeadingStatus(out, ctx.printMagHeadingStatusUser);
                    }
                } else {
                    tracker_serial_detail::printErr(out, "mag heading reference set failed");
                }
                return;
            }

            if (is(argv[2], "clear")) {
                if (!ctx.clearMagHeadingReference) {
                    tracker_serial_detail::printErr(out, "mag heading clear hook not available");
                    return;
                }

                ctx.clearMagHeadingReference(ctx.clearMagHeadingReferenceUser);
                tracker_serial_detail::printOk(out, "mag heading reference cleared");

                if (ctx.printMagHeadingStatus) {
                    ctx.printMagHeadingStatus(out, ctx.printMagHeadingStatusUser);
                }
                return;
            }

            tracker_serial_detail::printErr(out, "unknown mag heading command");
            return;
        }

        if (is(argv[1], "yaw")) {
            if (argc < 3 || is(argv[2], "status")) {
                if (ctx.printMagYawCorrectionStatus) {
                    ctx.printMagYawCorrectionStatus(out, ctx.printMagYawCorrectionStatusUser);
                } else {
                    tracker_serial_detail::printErr(out, "mag yaw status hook not available");
                }
                return;
            }

            if (is(argv[2], "reset")) {
                if (ctx.resetMagYawCorrection) {
                    ctx.resetMagYawCorrection(ctx.resetMagYawCorrectionUser);
                    tracker_serial_detail::printOk(out, "mag yaw correction dry-run stats reset");
                } else {
                    tracker_serial_detail::printErr(out, "mag yaw reset hook not available");
                }
                return;
            }

            tracker_serial_detail::printErr(out, "unknown mag yaw command");
            return;
        }

        if (is(argv[1], "axis")) {
            if (!ctx.config) {
                tracker_serial_detail::printErr(out, "config not available");
                return;
            }

            if (argc < 3 || is(argv[2], "print")) {
                printMagAxisMatrix(out, *ctx.config);
                return;
            }

            if (is(argv[2], "set")) {
                if (argc < 6) {
                    tracker_serial_detail::printErr(out, "usage: mag axis set <bodyX> <bodyY> <bodyZ> [save]");
                    out.println("# examples:");
                    out.println("#   mag axis set +x +y +z");
                    out.println("#   mag axis set +y -x +z");
                    out.println("#   mag axis set -y +x +z save");
                    return;
                }

                Mat3 m = Mat3::identity();
                if (!makeMagAxisMatrixFromTokens(argv[3], argv[4], argv[5], m)) {
                    tracker_serial_detail::printErr(out, "invalid axis mapping; use each of x/y/z exactly once, with optional +/-");
                    out.println("# valid examples:");
                    out.println("#   mag axis set +x +y +z");
                    out.println("#   mag axis set +y -x +z");
                    out.println("#   mag axis set -x +z +y");
                    return;
                }

                const bool saveRequested = argc >= 7 && is(argv[6], "save");

                ctx.config->data.magCal.magToImu = m;
                ctx.config->data.magCal.axisAlignmentValid = true;
                ctx.config->updateCrc();

                if (!saveConfigIfRequested(ctx, saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag axis save failed");
                    return;
                }

                tracker_serial_detail::printOk(out, saveRequested ? "mag axis mapping saved" : "mag axis mapping set in RAM");
                printMagAxisMatrix(out, *ctx.config);
                return;
            }

            if (is(argv[2], "identity")) {
                const bool saveRequested = argc >= 4 && is(argv[3], "save");

                ctx.config->data.magCal.magToImu = Mat3::identity();
                ctx.config->data.magCal.axisAlignmentValid = true;
                ctx.config->updateCrc();

                if (!saveConfigIfRequested(ctx, saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag axis identity save failed");
                    return;
                }

                tracker_serial_detail::printOk(out, saveRequested ? "mag axis identity saved" : "mag axis identity set in RAM");
                printMagAxisMatrix(out, *ctx.config);
                return;
            }

            if (is(argv[2], "clear")) {
                const bool saveRequested = argc >= 4 && is(argv[3], "save");

                ctx.config->data.magCal.magToImu = Mat3::identity();
                ctx.config->data.magCal.axisAlignmentValid = false;
                ctx.config->updateCrc();

                if (!saveConfigIfRequested(ctx, saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag axis clear save failed");
                    return;
                }

                tracker_serial_detail::printOk(out, saveRequested ? "mag axis cleared and saved" : "mag axis cleared in RAM");
                printMagAxisMatrix(out, *ctx.config);
                return;
            }

            tracker_serial_detail::printErr(out, "unknown mag axis command");
            return;
        }

        if (!ctx.mag) {
            tracker_serial_detail::printErr(out, "mag driver not available");
            return;
        }

        if (is(argv[1], "id")) {
            uint8_t id = 0;
            const bool ok = ctx.mag->probe(&id);
            out.print(ok ? "# OK " : "# ERR ");
            out.print("qmc_id=0x"); out.print(id, HEX);
            out.print(" expected=0x"); out.print(Qmc6309::EXPECTED_CHIP_ID, HEX);
            out.print(" qmcErr="); out.print(ctx.mag->lastErrorName());
            if (ctx.sensorHub) { out.print(" hubErr="); out.print(ctx.sensorHub->lastErrorName()); }
            out.println();
            rearmMagIfNeeded(ctx);
            return;
        }

        if (is(argv[1], "qmcstatus")) {
            Qmc6309::Status st;
            const bool ok = ctx.mag->readStatus(st);
            out.print(ok ? "# OK " : "# ERR ");
            out.print("qmc_status=0x"); out.print(st.raw, HEX);
            out.print(" drdy="); out.print(st.dataReady ? 1 : 0);
            out.print(" ovfl="); out.print(st.overflow ? 1 : 0);
            out.print(" nvm_ready="); out.print(st.nvmReady ? 1 : 0);
            out.print(" nvm_load_done="); out.print(st.nvmLoadDone ? 1 : 0);
            out.print(" qmcErr="); out.print(ctx.mag->lastErrorName());
            if (ctx.sensorHub) { out.print(" hubErr="); out.print(ctx.sensorHub->lastErrorName()); }
            out.println();
            rearmMagIfNeeded(ctx);
            return;
        }

        if (is(argv[1], "regs")) {
            out.println("# QMC REGS 00..0B");
            for (uint8_t r = 0; r <= 0x0B; ++r) {
                uint8_t v = 0;
                const bool ok = ctx.mag->readReg(r, v);
                out.print("0x"); if (r < 0x10) out.print('0'); out.print(r, HEX);
                out.print('=');
                if (ok) { out.print("0x"); if (v < 0x10) out.print('0'); out.print(v, HEX); }
                else out.print("ERR");
                out.println();
            }
            rearmMagIfNeeded(ctx);
            return;
        }

        if (is(argv[1], "hub")) {
            if (!ctx.sensorHub) {
                tracker_serial_detail::printErr(out, "sensor hub not available");
                return;
            }
            Lsm6dsvSensorHub::MasterStatus st;
            const bool ok = ctx.sensorHub->readMasterStatus(st);
            out.print(ok ? "# OK " : "# ERR ");
            out.print("hub_status=0x"); out.print(st.raw, HEX);
            out.print(" endop="); out.print(st.endop ? 1 : 0);
            out.print(" write_once_done="); out.print(st.writeOnceDone ? 1 : 0);
            out.print(" nack0="); out.print(st.slave0Nack ? 1 : 0);
            out.print(" nack1="); out.print(st.slave1Nack ? 1 : 0);
            out.print(" nack2="); out.print(st.slave2Nack ? 1 : 0);
            out.print(" nack3="); out.print(st.slave3Nack ? 1 : 0);
            out.print(" hubErr="); out.println(ctx.sensorHub->lastErrorName());
            return;
        }

        if (is(argv[1], "fifo")) {
            if (!ctx.fifo) {
                tracker_serial_detail::printErr(out, "fifo not available");
                return;
            }
            const auto& fs = ctx.fifo->stats();
            out.println("# MAG FIFO");
            out.print("sensorhub0_words="); out.println(fs.sensorHubSlave0Words);
            out.print("sensorhub_nack_words="); out.println(fs.sensorHubNackWords);
            out.print("mag_samples_produced="); out.println(fs.magSamplesProduced);
            out.print("mag_queue_overflow="); out.println(fs.magQueueOverflow);
            out.print("mag_tag_counter_jumps="); out.println(fs.magTagCounterJumps);
            out.print("mag_last_xyz="); out.print(fs.lastMagX); out.print(','); out.print(fs.lastMagY); out.print(','); out.println(fs.lastMagZ);
            out.print("mag_last_norm_raw="); out.println(fs.lastMagRawNorm, 3);
            out.print("mag_dt_last_us="); out.println(fs.lastMagDtUs);
            out.print("mag_dt_min_us="); out.println(fs.minMagDtUs);
            out.print("mag_dt_max_us="); out.println(fs.maxMagDtUs);
            return;
        }

        tracker_serial_detail::printErr(out, "unknown mag command");
    }

    static void cmdFifo(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.fifo) {
            tracker_serial_detail::printErr(out, "fifo not available");
            return;
        }

        if (argc < 2 || is(argv[1], "status")) {
            Lsm6dsvFifoReader::Status st;
            if (!ctx.fifo->readStatus(st)) {
                tracker_serial_detail::printErr(out, "fifo status read failed");
                return;
            }
            out.print("fifo_unread_words="); out.println(st.unreadWords);
            out.print("fifo_status1=0x"); out.println(st.rawStatus1, HEX);
            out.print("fifo_status2=0x"); out.println(st.rawStatus2, HEX);
            out.print("fifo_watermark="); out.println(st.watermark ? "yes" : "no");
            out.print("fifo_overrun="); out.println(st.overrun ? "yes" : "no");
            out.print("fifo_full="); out.println(st.full ? "yes" : "no");
            return;
        }

        if (is(argv[1], "stats")) {
            printFifoStats(out, ctx.fifo->stats());
            return;
        }

        if (is(argv[1], "reset")) {
            const uint64_t lastTs = ctx.fifo->stats().lastAssignedTimestampUs;
            const bool ok = ctx.fifo->resetFifo();
            ctx.fifo->resetTimestampReconstruction(lastTs);
            if (ctx.quality) {
                ctx.quality->reset();
                ctx.quality->syncFifoStats(ctx.fifo->stats());
            }
            if (ctx.resetFifoRuntime) ctx.resetFifoRuntime(ctx.resetFifoRuntimeUser);
            if (ok) tracker_serial_detail::printOk(out, "fifo reset");
            else tracker_serial_detail::printErr(out, "fifo reset failed");
            return;
        }

        tracker_serial_detail::printErr(out, "unknown fifo command");
    }

    static void cmdQuality(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.quality) {
            tracker_serial_detail::printErr(out, "quality monitor not available");
            return;
        }
        if (argc < 2 || is(argv[1], "stats")) {
            printQualityStats(out, ctx.quality->counters());
            return;
        }
        if (is(argv[1], "reset")) {
            ctx.quality->reset();
            if (ctx.fifo) ctx.quality->syncFifoStats(ctx.fifo->stats());
            tracker_serial_detail::printOk(out, "quality counters reset");
            return;
        }
        tracker_serial_detail::printErr(out, "unknown quality command");
    }

    static void cmdCal(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (argc < 2) {
            tracker_serial_detail::printErr(out, "usage: cal gyro|accel|temp|save|clear_all");
            return;
        }

        if (is(argv[1], "gyro")) {
            cmdCalGyro(ctx, argc, argv);
            return;
        }
        if (is(argv[1], "accel")) {
            cmdCalAccel(ctx, argc, argv);
            return;
        }
        if (is(argv[1], "temp")) {
            cmdCalTemp(ctx, argc, argv);
            return;
        }
        if (is(argv[1], "save")) {
            if (!ctx.config || !ctx.configStore) {
                tracker_serial_detail::printErr(out, "config/configStore not available");
                return;
            }
            captureRuntimeToConfig(ctx);
            if (ctx.configStore->save(*ctx.config)) tracker_serial_detail::printOk(out, "calibration saved");
            else {
                out.print("# ERR calibration save failed: ");
                out.println(ctx.configStore->lastErrorName());
            }
            return;
        }
        if (is(argv[1], "clear_all")) {
            if (ctx.imuCal) {
                ctx.imuCal->gyroBiasValid = false;
                ctx.imuCal->gyroBiasRadS = Vec3::zero();
                ctx.imuCal->accelCalValid = false;
                ctx.imuCal->accelBiasG = Vec3::zero();
                ctx.imuCal->accelScale = Mat3::identity();
            }
            if (ctx.config) {
                ctx.config->data.gyroCal = TrackerGyroCalibrationConfig{};
                ctx.config->data.accelCal = TrackerAccelCalibrationConfig{};
                ctx.config->updateCrc();
            }
            tracker_serial_detail::printOk(out, "all calibration cleared in RAM");
            return;
        }

        tracker_serial_detail::printErr(out, "unknown cal command");
    }

    static void cmdCalGyro(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (argc >= 3 && is(argv[2], "save")) {
            if (!ctx.config || !ctx.configStore || !ctx.imuCal) {
                tracker_serial_detail::printErr(out, "config or calibration not available");
                return;
            }
            ctx.config->captureFromImuCalibration(*ctx.imuCal);
            if (ctx.gyroTempComp) ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
            if (ctx.configStore->save(*ctx.config)) tracker_serial_detail::printOk(out, "gyro calibration saved");
            else {
                out.print("# ERR gyro save failed: ");
                out.println(ctx.configStore->lastErrorName());
            }
            return;
        }

        if (argc >= 3 && is(argv[2], "clear")) {
            if (ctx.imuCal) {
                ctx.imuCal->gyroBiasValid = false;
                ctx.imuCal->gyroBiasRadS = Vec3::zero();
            }
            if (ctx.config) {
                ctx.config->data.gyroCal = TrackerGyroCalibrationConfig{};
                ctx.config->updateCrc();
            }
            tracker_serial_detail::printOk(out, "gyro calibration cleared in RAM");
            return;
        }

        if (!ctx.calibrationIo || !ctx.imuCal) {
            tracker_serial_detail::printErr(out, "calibration IO or imuCal not available");
            return;
        }

        out.println("# gyro calibration started; keep tracker still");

        FifoGyroStartupCalibrationParams params;
        FifoGyroStartupCalibrator cal(params);
        GyroStartupCalibrationResult result;

        GyroProgressPrinter pp;
        pp.out = &out;
        pp.lastPrintMs = 0;

        const bool ok = cal.run(*ctx.calibrationIo, result, &gyroProgressCallback, &pp);
        printGyroResult(out, result, ctx.calibrationIo->latestTempC);

        if (!ok) {
            tracker_serial_detail::printErr(out, "gyro calibration failed");
            return;
        }

        FifoGyroStartupCalibrator::applyResultToCalibration(
            result,
            *ctx.imuCal,
            ctx.gyroTempComp,
            ctx.calibrationIo->latestTempC
        );

        if (ctx.config) {
            ctx.config->captureFromImuCalibration(*ctx.imuCal);
            if (ctx.gyroTempComp) ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
        }

        tracker_serial_detail::printOk(out, "gyro calibration applied to RAM; use cal gyro save or config save");
    }

    static void cmdCalAccel(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.accelCalRunner) {
            tracker_serial_detail::printErr(out, "accel calibration runner not available");
            return;
        }

        if (argc < 3) {
            tracker_serial_detail::printErr(out, "usage: cal accel face|compute|dump|save|clear");
            return;
        }

        if (is(argv[2], "face")) {
            if (argc < 4) {
                tracker_serial_detail::printErr(out, "usage: cal accel face XP|XN|YP|YN|ZP|ZN");
                return;
            }
            if (!ctx.calibrationIo) {
                tracker_serial_detail::printErr(out, "calibration IO not available");
                return;
            }
            const Accel6PosCalibration::Face face = Accel6PosCalibration::parseFace(argv[3]);
            if (face == Accel6PosCalibration::Face::Invalid) {
                tracker_serial_detail::printErr(out, "invalid face; use XP XN YP YN ZP ZN");
                return;
            }

            out.print("# accel face capture started: ");
            out.println(Accel6PosCalibration::faceName(face));

            AccelProgressPrinter pp;
            pp.out = &out;
            pp.lastPrintMs = 0;
            const bool ok = ctx.accelCalRunner->captureFace(*ctx.calibrationIo, face, &accelProgressCallback, &pp);
            if (ok) {
                out.print("# OK accel face captured: ");
                out.println(Accel6PosCalibration::faceName(face));
            } else {
                tracker_serial_detail::printErr(out, "accel face capture failed");
            }
            return;
        }

        if (is(argv[2], "compute")) {
            if (!ctx.accelCalRunner->compute()) {
                tracker_serial_detail::printErr(out, "accel calibration compute failed; capture all faces first");
                return;
            }
            printAccelCal(out, ctx.accelCalRunner->calibration());
            if (ctx.imuCal) ctx.accelCalRunner->applyToImuCalibration(*ctx.imuCal);
            if (ctx.config && ctx.imuCal) ctx.config->captureFromImuCalibration(*ctx.imuCal);
            tracker_serial_detail::printOk(out, "accel calibration computed and applied to RAM");
            return;
        }

        if (is(argv[2], "dump")) {
            printAccelCal(out, ctx.accelCalRunner->calibration());
            return;
        }

        if (is(argv[2], "save")) {
            if (!ctx.config || !ctx.configStore || !ctx.imuCal) {
                tracker_serial_detail::printErr(out, "config/configStore/imuCal not available");
                return;
            }
            ctx.config->captureFromImuCalibration(*ctx.imuCal);
            if (ctx.configStore->save(*ctx.config)) tracker_serial_detail::printOk(out, "accel calibration saved");
            else {
                out.print("# ERR accel save failed: ");
                out.println(ctx.configStore->lastErrorName());
            }
            return;
        }

        if (is(argv[2], "clear")) {
            ctx.accelCalRunner->reset();
            if (ctx.imuCal) {
                ctx.imuCal->accelCalValid = false;
                ctx.imuCal->accelBiasG = Vec3::zero();
                ctx.imuCal->accelScale = Mat3::identity();
            }
            if (ctx.config) {
                ctx.config->data.accelCal = TrackerAccelCalibrationConfig{};
                ctx.config->updateCrc();
            }
            tracker_serial_detail::printOk(out, "accel calibration cleared in RAM");
            return;
        }

        tracker_serial_detail::printErr(out, "unknown cal accel command");
    }

    static void cmdCalTemp(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.gyroTempComp) {
            tracker_serial_detail::printErr(out, "gyro temp comp not available");
            return;
        }

        if (argc < 3 || is(argv[2], "print")) {
            const auto s = ctx.gyroTempComp->snapshot(25.0f);
            out.print("temp_comp_valid="); out.println(s.valid ? "yes" : "no");
            out.print("temp_comp_enabled="); out.println(s.enabled ? "yes" : "no");
            out.print("reference_temp_c="); out.println(s.referenceTempC, 3);
            tracker_serial_detail::printVec3Line(out, "slope_dps_per_c", s.slopeDpsPerC, 6);
            tracker_serial_detail::printVec3Line(out, "reference_bias_dps", s.referenceBiasDps, 5);
            return;
        }

        if (is(argv[2], "set_slope")) {
            if (argc < 6) {
                tracker_serial_detail::printErr(out, "usage: cal temp set_slope X Y Z  ; values in dps/C");
                return;
            }
            float x = 0, y = 0, z = 0;
            if (!tracker_serial_detail::parseFloat(argv[3], x) ||
                !tracker_serial_detail::parseFloat(argv[4], y) ||
                !tracker_serial_detail::parseFloat(argv[5], z)) {
                tracker_serial_detail::printErr(out, "invalid slope values");
                return;
            }
            ctx.gyroTempComp->setSlopeDpsPerC(Vec3(x, y, z));
            if (ctx.config) ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
            tracker_serial_detail::printOk(out, "temperature slope applied to RAM");
            return;
        }

        if (is(argv[2], "clear")) {
            ctx.gyroTempComp->setSlopeRadSPerC(Vec3::zero());
            if (ctx.config) {
                ctx.config->data.gyroCal.tempCompValid = false;
                ctx.config->data.gyroCal.tempSlopeRadSPerC = Vec3::zero();
                ctx.config->updateCrc();
            }
            tracker_serial_detail::printOk(out, "temperature compensation slope cleared in RAM");
            return;
        }

        tracker_serial_detail::printErr(out, "unknown cal temp command");
    }

    static void cmdAhrs(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.ahrs) {
            tracker_serial_detail::printErr(out, "ahrs not available");
            return;
        }

        if (argc < 2 || is(argv[1], "status")) {
            const Vec3 e = ctx.ahrs->eulerDeg();
            const Quat q = ctx.ahrs->quaternionPositiveW();
            tracker_serial_detail::printQuatLine(out, "quat", q, 7);
            tracker_serial_detail::printVec3Line(out, "euler_deg", e, 3);
            const auto& st = ctx.ahrs->stats();
            out.print("ahrs_updates="); out.println(st.updateCount);
            out.print("accel_updates="); out.println(st.accelUpdateCount);
            out.print("accel_rejects="); out.println(st.accelRejectedCount);
            return;
        }

        if (is(argv[1], "reset")) {
            ctx.ahrs->reset();
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
            tracker_serial_detail::printOk(out, "ahrs reset");
            return;
        }

        tracker_serial_detail::printErr(out, "unknown ahrs command");
    }

    static void cmdStream(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.streamState) {
            tracker_serial_detail::printErr(out, "stream state not available");
            return;
        }

        if (argc < 2) {
            out.print("stream_mode="); out.println(streamModeName(ctx.streamState->mode));
            out.print("stream_rate_hz="); out.println(ctx.streamState->rateHz);
            return;
        }

        if (is(argv[1], "rate")) {
            if (argc < 3) {
                tracker_serial_detail::printErr(out, "usage: stream rate <hz>");
                return;
            }
            uint32_t hz = 0;
            if (!tracker_serial_detail::parseU32(argv[2], hz) || hz == 0 || hz > 1000) {
                tracker_serial_detail::printErr(out, "invalid stream rate; expected 1..1000");
                return;
            }
            ctx.streamState->rateHz = static_cast<uint16_t>(hz);
            if (ctx.config) {
                ctx.config->data.output.outputRateHz = static_cast<uint16_t>(hz);
                ctx.config->updateCrc();
            }
            tracker_serial_detail::printOk(out, "stream rate set");
            return;
        }

        TrackerStreamMode mode;
        if (!parseStreamMode(argv[1], mode)) {
            tracker_serial_detail::printErr(out, "unknown stream mode; use off|heartbeat|raw|scaled|quat|debug");
            return;
        }
        ctx.streamState->mode = mode;
        ctx.streamState->lastEmitUs = 0;

        if (ctx.config) {
            ctx.config->data.output.quaternionOutputEnabled = (mode == TrackerStreamMode::Quat);
            ctx.config->data.output.serialDebugEnabled = (mode == TrackerStreamMode::Debug);
            ctx.config->updateCrc();
        }

        out.print("# OK stream mode ");
        out.println(streamModeName(mode));
    }

    static void cmdTest(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (argc < 2) {
            tracker_serial_detail::printErr(out, "usage: test static <seconds>|stop|status");
            return;
        }

        if (is(argv[1], "status")) {
            if (ctx.printStaticTestStatus) {
                ctx.printStaticTestStatus(out, ctx.printStaticTestStatusUser);
            } else {
                tracker_serial_detail::printErr(out, "static test status hook not available");
            }
            return;
        }

        if (is(argv[1], "stop")) {
            if (!ctx.stopStaticTest) {
                tracker_serial_detail::printErr(out, "static test stop hook not available");
                return;
            }
            const bool ok = ctx.stopStaticTest(ctx.stopStaticTestUser);
            if (ok) tracker_serial_detail::printOk(out, "static test stop requested");
            else tracker_serial_detail::printErr(out, "static test was not running");
            return;
        }

        if (is(argv[1], "static")) {
            if (argc < 3) {
                tracker_serial_detail::printErr(out, "usage: test static <seconds>");
                return;
            }
            if (!ctx.startStaticTest) {
                tracker_serial_detail::printErr(out, "static test start hook not available");
                return;
            }
            uint32_t seconds = 0;
            if (!tracker_serial_detail::parseU32(argv[2], seconds) || seconds == 0 || seconds > 21600UL) {
                tracker_serial_detail::printErr(out, "invalid duration; expected 1..21600 seconds");
                return;
            }
            const bool ok = ctx.startStaticTest(seconds * 1000UL, ctx.startStaticTestUser);
            if (ok) tracker_serial_detail::printOk(out, "static test started");
            else tracker_serial_detail::printErr(out, "static test already running");
            return;
        }

        tracker_serial_detail::printErr(out, "unknown test command");
    }

    static void cmdOutput(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.config) {
            tracker_serial_detail::printErr(out, "config not available");
            return;
        }
        if (argc < 2) {
            tracker_serial_detail::printErr(out, "usage: output mode|rate|start|stop");
            return;
        }

        if (is(argv[1], "rate")) {
            if (argc < 3) {
                tracker_serial_detail::printErr(out, "usage: output rate <hz>");
                return;
            }
            uint32_t hz = 0;
            if (!tracker_serial_detail::parseU32(argv[2], hz) || hz == 0 || hz > 1000) {
                tracker_serial_detail::printErr(out, "invalid output rate; expected 1..1000");
                return;
            }
            ctx.config->data.output.outputRateHz = static_cast<uint16_t>(hz);
            if (ctx.streamState) ctx.streamState->rateHz = static_cast<uint16_t>(hz);
            ctx.config->updateCrc();
            tracker_serial_detail::printOk(out, "output rate set");
            return;
        }

        if (is(argv[1], "mode")) {
            if (argc < 3) {
                tracker_serial_detail::printErr(out, "usage: output mode debug|binary|slimevr");
                return;
            }
            if (is(argv[2], "debug")) ctx.config->data.output.packetFormat = 0;
            else if (is(argv[2], "binary")) ctx.config->data.output.packetFormat = 1;
            else if (is(argv[2], "slimevr")) ctx.config->data.output.packetFormat = 2;
            else {
                tracker_serial_detail::printErr(out, "unknown output mode");
                return;
            }
            ctx.config->updateCrc();
            tracker_serial_detail::printOk(out, "output mode set");
            return;
        }

        if (is(argv[1], "start")) {
            ctx.config->data.output.quaternionOutputEnabled = true;
            if (ctx.streamState) ctx.streamState->mode = TrackerStreamMode::Quat;
            ctx.config->updateCrc();
            tracker_serial_detail::printOk(out, "output started");
            return;
        }

        if (is(argv[1], "stop")) {
            ctx.config->data.output.quaternionOutputEnabled = false;
            if (ctx.streamState) ctx.streamState->mode = TrackerStreamMode::Off;
            ctx.config->updateCrc();
            tracker_serial_detail::printOk(out, "output stopped");
            return;
        }

        tracker_serial_detail::printErr(out, "unknown output command");
    }

    static void applyConfigToRuntime(TrackerSerialCommandContext& ctx) {
        if (!ctx.config) return;
        if (ctx.imuCal) ctx.config->applyToImuCalibration(*ctx.imuCal);
        if (ctx.gyroTempComp) ctx.config->applyToGyroTempComp(*ctx.gyroTempComp);
        if (ctx.quality) {
            ctx.quality->setConfig(ctx.config->makeQualityConfig());
            ctx.quality->reset();
            if (ctx.fifo) ctx.quality->syncFifoStats(ctx.fifo->stats());
        }
        if (ctx.streamState) {
            ctx.streamState->rateHz = ctx.config->data.output.outputRateHz;
            ctx.streamState->mode = ctx.config->data.output.quaternionOutputEnabled
                ? TrackerStreamMode::Quat
                : TrackerStreamMode::Off;
        }
    }

    static void captureRuntimeToConfig(TrackerSerialCommandContext& ctx) {
        if (!ctx.config) return;
        if (ctx.imuCal) ctx.config->captureFromImuCalibration(*ctx.imuCal);
        if (ctx.gyroTempComp) ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
        if (ctx.streamState) {
            ctx.config->data.output.outputRateHz = ctx.streamState->rateHz;
            ctx.config->data.output.quaternionOutputEnabled = (ctx.streamState->mode == TrackerStreamMode::Quat);
            ctx.config->data.output.serialDebugEnabled = (ctx.streamState->mode == TrackerStreamMode::Debug);
        }
        ctx.config->updateCrc();
    }

    static void printFifoStats(Stream& out, const Lsm6dsvFifoReader::DrainStats& fs) {
        out.println("# FIFO STATS");
        out.print("fifo_words_read="); out.println(fs.fifoWordsRead);
        out.print("imu_samples="); out.println(fs.imuSamplesProduced);
        out.print("gyro_words="); out.println(fs.gyroWords);
        out.print("accel_words="); out.println(fs.accelWords);
        out.print("timestamp_words="); out.println(fs.timestampWords);
        out.print("temperature_words="); out.println(fs.tempWords);
        out.print("unknown_words="); out.println(fs.unknownWords);
        out.print("overrun_events="); out.println(fs.overrunEvents);
        out.print("full_events="); out.println(fs.fullEvents);
        out.print("watermark_events="); out.println(fs.watermarkEvents);
        out.print("max_unread_words="); out.println(fs.maxUnreadWordsSeen);
        out.print("tag_counter_jumps="); out.println(fs.tagCounterJumps);
        out.print("gyro_tag_counter_jumps="); out.println(fs.gyroTagCounterJumps);
        out.print("accel_tag_counter_jumps="); out.println(fs.accelTagCounterJumps);
        out.print("hw_timestamp_assigned="); out.println(fs.hwTimestampAssigned);
        out.print("fallback_timestamp_assigned="); out.println(fs.fallbackTimestampAssigned);
        out.print("timestamp_backwards="); out.println(fs.timestampBackwards);
        out.print("timestamp_duplicate="); out.println(fs.timestampDuplicate);
        out.print("timestamp_large_gap="); out.println(fs.timestampLargeGap);
        out.print("timestamp_meta_bdr_xl_mismatch="); out.println(fs.timestampMetaBdrXlMismatch);
        out.print("timestamp_meta_bdr_gy_mismatch="); out.println(fs.timestampMetaBdrGyMismatch);
        out.print("gyro_saturation_count="); out.println(fs.gyroSaturationCount);
        out.print("accel_saturation_count="); out.println(fs.accelSaturationCount);
        out.print("sensorhub0_words="); out.println(fs.sensorHubSlave0Words);
        out.print("sensorhub_nack_words="); out.println(fs.sensorHubNackWords);
        out.print("mag_samples_produced="); out.println(fs.magSamplesProduced);
        out.print("mag_queue_overflow="); out.println(fs.magQueueOverflow);
        out.print("mag_tag_counter_jumps="); out.println(fs.magTagCounterJumps);
        out.print("mag_raw_saturation_count="); out.println(fs.magRawSaturationCount);
        out.print("mag_last_xyz="); out.print(fs.lastMagX); out.print(','); out.print(fs.lastMagY); out.print(','); out.println(fs.lastMagZ);
        out.print("mag_last_norm_raw="); out.println(fs.lastMagRawNorm, 3);
        out.print("mag_dt_last_us="); out.println(fs.lastMagDtUs);
        out.print("mag_dt_min_us="); out.println(fs.minMagDtUs);
        out.print("mag_dt_max_us="); out.println(fs.maxMagDtUs);
        out.print("latest_temp_valid="); out.println(fs.latestTempValid ? "yes" : "no");
        out.print("latest_temp_c="); out.println(fs.latestTempC, 3);
        out.print("sample_period_us="); out.println(fs.samplePeriodUs, 3);
        out.print("timestamp_tick_us="); out.println(fs.timestampTickUs, 6);
    }

    static void printQualityStats(Stream& out, const ImuQualityCounters& qc) {
        out.println("# QUALITY STATS");
        out.print("samples="); out.println(qc.samples);
        out.print("hw_timestamp_samples="); out.println(qc.hwTimestampSamples);
        out.print("fallback_timestamp_samples="); out.println(qc.fallbackTimestampSamples);
        out.print("zero_timestamp_samples="); out.println(qc.zeroTimestampSamples);
        out.print("nonmonotonic_timestamp_samples="); out.println(qc.nonMonotonicTimestampSamples);
        out.print("large_gap_samples="); out.println(qc.largeGapSamples);
        out.print("estimated_dropped_samples="); out.println(qc.estimatedDroppedSamples);
        out.print("fifo_overrun_events="); out.println(qc.fifoOverrunEvents);
        out.print("fifo_full_events="); out.println(qc.fifoFullEvents);
        out.print("fifo_unknown_tag_events="); out.println(qc.fifoUnknownTagEvents);
        out.print("fifo_tag_counter_jumps="); out.println(qc.fifoTagCounterJumps);
        out.print("fifo_gyro_tag_counter_jumps="); out.println(qc.fifoGyroTagCounterJumps);
        out.print("fifo_accel_tag_counter_jumps="); out.println(qc.fifoAccelTagCounterJumps);
        out.print("gyro_saturated_samples="); out.println(qc.gyroSaturatedSamples);
        out.print("accel_saturated_samples="); out.println(qc.accelSaturatedSamples);
        out.print("gyro_near_saturated_samples="); out.println(qc.gyroNearSaturatedSamples);
        out.print("accel_near_saturated_samples="); out.println(qc.accelNearSaturatedSamples);
        out.print("accel_norm_outliers="); out.println(qc.accelNormOutliers);
        out.print("ahrs_skipped_samples="); out.println(qc.ahrsSkippedSamples);
        out.print("accel_correction_disabled_samples="); out.println(qc.accelCorrectionDisabledSamples);
        out.print("fifo_recovery_requests="); out.println(qc.fifoRecoveryRequests);
        out.print("mean_dt_us="); out.println(qc.meanDtUs(), 6);
        out.print("min_dt_us="); out.println(qc.minDtUs, 6);
        out.print("max_dt_us="); out.println(qc.maxDtUs, 6);
    }

    struct GyroProgressPrinter {
        Stream* out = nullptr;
        uint32_t lastPrintMs = 0;
    };

    static void gyroProgressCallback(const FifoGyroStartupCalibrationProgress& p, void* user) {
        GyroProgressPrinter* pp = static_cast<GyroProgressPrinter*>(user);
        if (!pp || !pp->out) return;
        const uint32_t now = millis();
        if (now - pp->lastPrintMs < 500) return;
        pp->lastPrintMs = now;

        pp->out->print("# cal gyro still=");
        pp->out->print(p.stationarySamples);
        pp->out->print('/');
        pp->out->print(p.requiredStationarySamples);
        pp->out->print(" total=");
        pp->out->print(p.totalSamples);
        pp->out->print(" rejected=");
        pp->out->print(p.rejectedSamples);
        pp->out->print(" temp_c=");
        pp->out->println(p.latestTempC, 3);
    }

    static void printGyroResult(Stream& out, const GyroStartupCalibrationResult& r, float tempC) {
        out.println("# GYRO CAL RESULT");
        out.print("success="); out.println(r.success ? "yes" : "no");
        out.print("stationary_samples="); out.println(r.stationarySamples);
        out.print("total_samples="); out.println(r.totalSamples);
        tracker_serial_detail::printVec3Line(out, "gyro_bias_rad_s", r.gyroBiasRadS, 8);
        tracker_serial_detail::printVec3Line(out, "gyro_bias_dps", r.gyroBiasDps, 5);
        tracker_serial_detail::printVec3(out, "accel_mean_g", r.accelMeanG, 6);
        out.print(" norm="); out.println(r.accelNormMeanG, 6);
        out.print("gyro_noise_norm_rad_s2="); out.println(r.gyroNoiseNormRadS2, 10);
        out.print("accel_noise_norm_g2="); out.println(r.accelNoiseNormG2, 10);
        out.print("reference_temp_c="); out.println(tempC, 3);
    }

    struct AccelProgressPrinter {
        Stream* out = nullptr;
        uint32_t lastPrintMs = 0;
    };

    static void accelProgressCallback(const FifoAccel6PosCaptureProgress& p, void* user) {
        AccelProgressPrinter* pp = static_cast<AccelProgressPrinter*>(user);
        if (!pp || !pp->out) return;
        const uint32_t now = millis();
        if (now - pp->lastPrintMs < 250) return;
        pp->lastPrintMs = now;

        pp->out->print("# cal accel face=");
        pp->out->print(Accel6PosCalibration::faceName(p.face));
        pp->out->print(" accepted=");
        pp->out->print(p.acceptedSamples);
        pp->out->print('/');
        pp->out->print(p.requiredSamples);
        pp->out->print(" rejected=");
        pp->out->print(p.rejectedSamples);
        pp->out->print(" mean=");
        pp->out->print(p.meanG.x, 5); pp->out->print(',');
        pp->out->print(p.meanG.y, 5); pp->out->print(',');
        pp->out->print(p.meanG.z, 5);
        pp->out->print(" norm=");
        pp->out->println(p.meanNormG, 6);
    }

    static void printAccelCal(Stream& out, const Accel6PosCalibration& cal) {
        out.println("# ACCEL CAL DUMP");
        printAccelFace(out, cal, Accel6PosCalibration::Face::XP);
        printAccelFace(out, cal, Accel6PosCalibration::Face::XN);
        printAccelFace(out, cal, Accel6PosCalibration::Face::YP);
        printAccelFace(out, cal, Accel6PosCalibration::Face::YN);
        printAccelFace(out, cal, Accel6PosCalibration::Face::ZP);
        printAccelFace(out, cal, Accel6PosCalibration::Face::ZN);

        const auto& r = cal.result();
        out.print("result_valid="); out.println(r.valid ? "yes" : "no");
        tracker_serial_detail::printVec3Line(out, "accel_bias_g", r.biasG, 8);
        tracker_serial_detail::printVec3Line(out, "accel_scale_diag", r.scale, 8);
        out.print("max_face_norm_error_g="); out.println(r.maxFaceNormErrorG, 8);
        out.print("max_axis_residual_g="); out.println(r.maxAxisResidualG, 8);
    }

    static void printAccelFace(Stream& out, const Accel6PosCalibration& cal, Accel6PosCalibration::Face face) {
        out.print(Accel6PosCalibration::faceName(face));
        out.print('=');
        if (!cal.hasFace(face)) {
            out.println("missing");
            return;
        }
        const auto& d = cal.faceData(face);
        out.print("samples:"); out.print(d.samples);
        out.print(",mean:");
        out.print(d.meanG.x, 6); out.print(',');
        out.print(d.meanG.y, 6); out.print(',');
        out.print(d.meanG.z, 6);
        out.print(",norm:"); out.println(d.meanNormG, 6);
    }

    static bool parseStreamMode(const char* s, TrackerStreamMode& mode) {
        if (is(s, "off"))       { mode = TrackerStreamMode::Off; return true; }
        if (is(s, "heartbeat")) { mode = TrackerStreamMode::Heartbeat; return true; }
        if (is(s, "raw"))       { mode = TrackerStreamMode::Raw; return true; }
        if (is(s, "scaled"))    { mode = TrackerStreamMode::Scaled; return true; }
        if (is(s, "quat"))      { mode = TrackerStreamMode::Quat; return true; }
        if (is(s, "debug"))     { mode = TrackerStreamMode::Debug; return true; }
        return false;
    }

    static const char* streamModeName(TrackerStreamMode mode) {
        switch (mode) {
            case TrackerStreamMode::Off:        return "off";
            case TrackerStreamMode::Heartbeat:  return "heartbeat";
            case TrackerStreamMode::Raw:        return "raw";
            case TrackerStreamMode::Scaled:     return "scaled";
            case TrackerStreamMode::Quat:       return "quat";
            case TrackerStreamMode::Debug:      return "debug";
        }
        return "unknown";
    }
};

template <size_t LINE_CAP = 128, size_t MAX_ARGS = 10>
class TrackerSerialCommandInterface {
public:
    void begin(TrackerSerialCommandContext& ctx) {
        ctx_ = &ctx;
        len_ = 0;
        overflow_ = false;
    }

    void poll() {
        if (!ctx_ || !ctx_->io) return;

        Stream& s = *ctx_->io;
        while (s.available() > 0) {
            const char c = static_cast<char>(s.read());
            feed(c);
        }
    }

    void feed(char c) {
        if (!ctx_ || !ctx_->io) return;

        if (c == '\r') return;

        if (c == '\n') {
            line_[len_] = '\0';
            if (overflow_) {
                tracker_serial_detail::printErr(*ctx_->io, "line too long");
            } else {
                processLine(line_);
            }
            len_ = 0;
            overflow_ = false;
            return;
        }

        if (c == '\b' || c == 0x7F) {
            if (len_ > 0) len_--;
            return;
        }

        if (len_ >= LINE_CAP - 1) {
            overflow_ = true;
            return;
        }

        line_[len_++] = c;
    }

    void processLine(char* line) {
        if (!ctx_ || !line) return;

        char* argv[MAX_ARGS] = {};
        const int argc = tokenize(line, argv, MAX_ARGS);
        if (argc <= 0) return;

        TrackerCommandDispatcher::dispatch(*ctx_, argc, argv);
    }

private:
    static int tokenize(char* line, char** argv, size_t maxArgs) {
        size_t argc = 0;
        char* p = line;

        while (*p && argc < maxArgs) {
            while (*p == ' ' || *p == '\t') ++p;
            if (*p == '\0') break;

            if (*p == '#') break;

            argv[argc++] = p;

            while (*p && *p != ' ' && *p != '\t') ++p;
            if (*p == '\0') break;
            *p++ = '\0';
        }

        return static_cast<int>(argc);
    }

    TrackerSerialCommandContext* ctx_ = nullptr;
    char line_[LINE_CAP] = {};
    size_t len_ = 0;
    bool overflow_ = false;
};

// ============================================================
// Optional stream emit helpers
// ============================================================

inline bool trackerSerialStreamDue(TrackerSerialStreamState& st, uint32_t nowUs) {
    if (st.mode == TrackerStreamMode::Off || st.mode == TrackerStreamMode::Heartbeat) return false;
    const uint32_t period = st.periodUs();
    if (st.lastEmitUs == 0 || nowUs - st.lastEmitUs >= period) {
        st.lastEmitUs = nowUs;
        return true;
    }
    return false;
}

inline void trackerSerialEmitQuat(Stream& out,
                                  uint64_t tUs,
                                  const Quat& q,
                                  uint32_t qualityFlags,
                                  float confidence) {
    out.print("Q,");
    out.print(static_cast<unsigned long>(tUs));
    out.print(','); out.print(q.w, 7);
    out.print(','); out.print(q.x, 7);
    out.print(','); out.print(q.y, 7);
    out.print(','); out.print(q.z, 7);
    out.print(",0x"); out.print(qualityFlags, HEX);
    out.print(','); out.println(confidence, 4);
}

inline void trackerSerialEmitRaw(Stream& out,
                                 const Lsm6dsv::RawSample& raw,
                                 uint32_t qualityFlags) {
    out.print("RAW,");
    out.print(static_cast<unsigned long>(raw.t_us));
    out.print(','); out.print(raw.ax);
    out.print(','); out.print(raw.ay);
    out.print(','); out.print(raw.az);
    out.print(','); out.print(raw.gx);
    out.print(','); out.print(raw.gy);
    out.print(','); out.print(raw.gz);
    out.print(",0x"); out.println(qualityFlags, HEX);
}

inline void trackerSerialEmitScaled(Stream& out,
                                    uint64_t tUs,
                                    const Lsm6dsv::Sample& s,
                                    uint32_t qualityFlags) {
    out.print("S,");
    out.print(static_cast<unsigned long>(tUs));
    out.print(','); out.print(s.accel_g.x, 6);
    out.print(','); out.print(s.accel_g.y, 6);
    out.print(','); out.print(s.accel_g.z, 6);
    out.print(','); out.print(s.gyro_rad_s.x, 7);
    out.print(','); out.print(s.gyro_rad_s.y, 7);
    out.print(','); out.print(s.gyro_rad_s.z, 7);
    out.print(','); out.print(s.temp_c, 3);
    out.print(",0x"); out.println(qualityFlags, HEX);
}

} // namespace tracker
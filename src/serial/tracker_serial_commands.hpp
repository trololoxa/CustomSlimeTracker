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

        if (is(argv[0], "log")) {
            cmdLog(ctx, argc, argv);
            return;
        }

        if (is(argv[0], "bias")) {
            cmdBias(ctx, argc, argv);
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
        out.println("config print | load | save | defaults | erase | crc | nvs");
        out.println("config spi <hz> [save]          (live SPI clock, e.g. 1000000/4000000/8000000)");
        out.println();
        out.println("imu status | whoami | read");
        out.println("imu rate <120|240|480|960> [save]   (live IMU+FIFO ODR reconfigure)");
        out.println("fifo status | stats | reset");
        out.println("fifo watermark <words> [save] | drain <max_words> <rounds> [save]");
        out.println("mag status | enable [save] | disable [save]");
        out.println("mag id | qmcstatus | regs | hub | fifo");
        out.println("mag processed | trust");
        out.println("mag heading | heading ref | heading status | heading clear");
        out.println("mag heading auto on | auto off | auto status");
        out.println("mag yaw status | yaw reset | yaw enable [save] | yaw disable [save]");
        out.println("mag yaw defaults [save] | tc <s> [save] | innovation <deg> [save]");
        out.println("mag yaw gyro_gate <goodDps> <badDps> [save]");
        out.println("mag yaw horiz_gate <bad> <good> [save]");
        out.println("mag yaw accel_gate <bad> <good> [save]");
        out.println("mag yaw age <ms> [save] | rate <deg_s> [save] | step <deg> [save]");
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
        out.println("cal temp print | enable [save] | disable [save]");
        out.println("cal temp set_slope X Y Z [save] | fit_static [save] | clear [save]");
        out.println("cal save | clear_all");
        out.println();
        out.println("ahrs status | config | reset");
        out.println("ahrs defaults [save]");
        out.println("ahrs accel on|off [save] | adaptive on|off [save]");
        out.println("ahrs accel_kp <gain> [save] | max_step <deg> [save]");
        out.println("ahrs accel_norm <goodErrG> <badErrG> [save]");
        out.println("ahrs accel_innovation <goodDeg> <badDeg> [save]");
        out.println("ahrs accel_var <goodStdG> <badStdG> [save]");
        out.println("ahrs gyro_gate <goodDps> <badDps> [save] | dt <minMs> <maxMs> [save]");
        out.println();
        out.println("stream off | heartbeat | raw | scaled | quat | debug");
        out.println("stream rate <hz>");
        out.println();
        out.println("log off | basic | full | start [basic|full] | stop");
        out.println("log rate <hz> | header | summary | reset");
        out.println();
        out.println("bias status | on | off | reset   (reset clears runtime trim)");
        out.println();
        out.println("test static <seconds>");
        out.println("test stop");
        out.println("test status");
        out.println();
        out.println("output mode debug");
        out.println("# output mode binary/slimevr: NOT_IMPLEMENTED in this build");
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
            tracker_serial_detail::printErr(out, "usage: config print|load|save|defaults|erase|crc|nvs|spi");
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
            out.print("# version="); out.println(ctx.config->data.version);
            out.print("# size="); out.println(ctx.config->data.size);
            return;
        }

        if (is(argv[1], "nvs")) {
            if (!ctx.configStore) {
                tracker_serial_detail::printErr(out, "config store not available");
                return;
            }
            TrackerConfigNvsInfo info;
            ctx.configStore->inspect(info);
            out.println("# CONFIG NVS");
            out.print("begin_ok="); out.println(info.beginOk ? "yes" : "no");
            out.print("exists="); out.println(info.exists ? "yes" : "no");
            out.print("stored_len="); out.println(static_cast<uint32_t>(info.storedLen));
            out.print("expected_len="); out.println(static_cast<uint32_t>(info.expectedLen));
            out.print("header_readable="); out.println(info.headerReadable ? "yes" : "no");
            out.print("stored_magic=0x"); out.println(info.storedMagic, HEX);
            out.print("stored_version="); out.println(info.storedVersion);
            out.print("stored_size="); out.println(info.storedSize);
            out.print("stored_crc=0x"); out.println(info.storedCrc, HEX);
            out.print("full_readable="); out.println(info.fullReadable ? "yes" : "no");
            out.print("valid="); out.println(info.valid ? "yes" : "no");
            out.print("error="); out.println(TrackerConfigStore::errorName(info.error));
            return;
        }

        if (is(argv[1], "spi")) {
            if (argc < 3) {
                tracker_serial_detail::printErr(out, "usage: config spi <hz> [save]");
                return;
            }

            uint32_t hz = 0;
            if (!tracker_serial_detail::parseU32(argv[2], hz) ||
                hz < tracker_config_detail::MIN_SPI_HZ ||
                hz > tracker_config_detail::MAX_SPI_HZ) {
                tracker_serial_detail::printErr(out, "invalid SPI Hz; expected 100000..10000000");
                return;
            }

            const bool save = argc >= 4 && is(argv[3], "save");
            if (argc >= 4 && !save) {
                tracker_serial_detail::printErr(out, "usage: config spi <hz> [save]");
                return;
            }

            ctx.config->data.hardware.spiHz = hz;
            ctx.config->sanitize();
            ctx.config->updateCrc();

            if (ctx.setSpiFrequency && !ctx.setSpiFrequency(ctx.config->data.hardware.spiHz, ctx.setSpiFrequencyUser)) {
                tracker_serial_detail::printErr(out, "failed to apply SPI clock");
                return;
            }

            if (save) {
                if (!ctx.configStore) {
                    tracker_serial_detail::printErr(out, "config store not available; changed in RAM only");
                    return;
                }
                if (!ctx.configStore->save(*ctx.config)) {
                    out.print("# ERR config save failed: ");
                    out.println(ctx.configStore->lastErrorName());
                    return;
                }
            }

            tracker_serial_detail::printOk(out, save ? "SPI clock set and saved" : "SPI clock set");
            out.print("# spi_hz="); out.println(ctx.config->data.hardware.spiHz);
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

    static const char* odrName(Lsm6dsv::Odr odr) {
        switch (odr) {
            case Lsm6dsv::Odr::PowerDown: return "0";
            case Lsm6dsv::Odr::Hz1_875: return "1.875";
            case Lsm6dsv::Odr::Hz7_5: return "7.5";
            case Lsm6dsv::Odr::Hz15: return "15";
            case Lsm6dsv::Odr::Hz30: return "30";
            case Lsm6dsv::Odr::Hz60: return "60";
            case Lsm6dsv::Odr::Hz120: return "120";
            case Lsm6dsv::Odr::Hz240: return "240";
            case Lsm6dsv::Odr::Hz480: return "480";
            case Lsm6dsv::Odr::Hz960: return "960";
            case Lsm6dsv::Odr::Hz1920: return "1920";
            case Lsm6dsv::Odr::Hz3840: return "3840";
            case Lsm6dsv::Odr::Hz7680: return "7680";
        }
        return "?";
    }

    static bool parseRuntimeOdr(const char* token, Lsm6dsv::Odr& outOdr) {
        if (!token) return false;
        uint32_t hz = 0;
        if (!tracker_serial_detail::parseU32(token, hz)) return false;
        switch (hz) {
            case 120: outOdr = Lsm6dsv::Odr::Hz120; return true;
            case 240: outOdr = Lsm6dsv::Odr::Hz240; return true;
            case 480: outOdr = Lsm6dsv::Odr::Hz480; return true;
            case 960: outOdr = Lsm6dsv::Odr::Hz960; return true;
            default: return false;
        }
    }

    static void printImuRuntimeStatus(TrackerSerialCommandContext& ctx, Stream& out) {
        out.print("imu_initialized="); out.println(ctx.lsm && ctx.lsm->isInitialized() ? "yes" : "no");
        if (ctx.lsm) {
            out.print("imu_last_who=0x"); out.println(ctx.lsm->lastWhoAmI(), HEX);
            out.print("imu_last_error="); out.println(static_cast<int>(ctx.lsm->lastError()));
        }
        if (ctx.config) {
            out.print("imu_config_odr_hz="); out.println(odrName(ctx.config->data.imu.imuOdr));
            out.print("fifo_accel_bdr_hz="); out.println(odrName(ctx.config->data.fifo.accelBdr));
            out.print("fifo_gyro_bdr_hz="); out.println(odrName(ctx.config->data.fifo.gyroBdr));
            out.print("fifo_watermark_words="); out.println(ctx.config->data.fifo.watermarkWords);
            out.print("fifo_max_words_per_drain="); out.println(ctx.config->data.fifo.maxWordsPerDrain);
            out.print("fifo_max_drain_rounds_per_event="); out.println(ctx.config->data.fifo.maxDrainRoundsPerEvent);
        }
        if (ctx.fifo) {
            const auto& fs = ctx.fifo->stats();
            out.print("fifo_configured="); out.println(ctx.fifo->isConfigured() ? "yes" : "no");
            out.print("fifo_sample_period_us="); out.println(fs.samplePeriodUs, 3);
            out.print("fifo_timestamp_tick_us="); out.println(fs.timestampTickUs, 6);
            out.print("fifo_internal_freq_fine="); out.println(fs.internalFreqFine);
            out.print("fifo_max_unread_words="); out.println(fs.maxUnreadWordsSeen);
            out.print("fifo_overrun_events="); out.println(fs.overrunEvents);
            out.print("fifo_full_events="); out.println(fs.fullEvents);
        }
    }

    static bool liveReconfigureImuFifo(TrackerSerialCommandContext& ctx, Stream& out) {
        if (!ctx.config || !ctx.lsm || !ctx.fifo) {
            tracker_serial_detail::printErr(out, "imu/fifo/config not available");
            return false;
        }

        if (ctx.streamState) ctx.streamState->mode = TrackerStreamMode::Off;
        if (ctx.logState) ctx.logState->mode = TrackerLogMode::Off;

        if (!ctx.lsm->begin(ctx.config->makeLsmConfig())) {
            out.print("# ERR imu reconfigure failed last_error=");
            out.println(static_cast<int>(ctx.lsm->lastError()));
            return false;
        }

        if (!ctx.fifo->configure(ctx.config->makeFifoConfig())) {
            tracker_serial_detail::printErr(out, "fifo reconfigure failed");
            return false;
        }

        ctx.fifo->resetTimestampReconstruction(0);
        if (ctx.quality) {
            ctx.quality->reset();
            ctx.quality->syncFifoStats(ctx.fifo->stats());
        }
        if (ctx.resetFifoRuntime) ctx.resetFifoRuntime(ctx.resetFifoRuntimeUser);
        if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
        rearmMagIfNeeded(ctx);
        return true;
    }

    static bool saveConfigIfRequested(TrackerSerialCommandContext& ctx, Stream& out, bool save) {
        if (!save) return true;
        if (!ctx.config || !ctx.configStore) {
            tracker_serial_detail::printErr(out, "config store not available; changed in RAM only");
            return false;
        }
        ctx.config->updateCrc();
        if (!ctx.configStore->save(*ctx.config)) {
            out.print("# ERR save failed: ");
            out.println(ctx.configStore->lastErrorName());
            return false;
        }
        return true;
    }

    static void cmdImu(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.lsm) {
            tracker_serial_detail::printErr(out, "imu not available");
            return;
        }
        if (argc < 2 || is(argv[1], "status")) {
            printImuRuntimeStatus(ctx, out);
            return;
        }

        if (is(argv[1], "rate") || is(argv[1], "odr")) {
            if (argc < 3) {
                printImuRuntimeStatus(ctx, out);
                out.println("# usage: imu rate <120|240|480|960> [save]");
                return;
            }
            if (!ctx.config) {
                tracker_serial_detail::printErr(out, "config not available");
                return;
            }
            Lsm6dsv::Odr odr;
            if (!parseRuntimeOdr(argv[2], odr)) {
                tracker_serial_detail::printErr(out, "unsupported ODR; expected 120, 240, 480 or 960");
                return;
            }
            const bool save = (argc >= 4 && is(argv[3], "save"));
            ctx.config->data.imu.imuOdr = odr;
            ctx.config->data.fifo.accelBdr = odr;
            ctx.config->data.fifo.gyroBdr = odr;
            ctx.config->data.fifo.samplePeriodUsOverride = 0.0f;
            ctx.config->sanitize();
            ctx.config->updateCrc();

            if (!liveReconfigureImuFifo(ctx, out)) return;

            if (!saveConfigIfRequested(ctx, out, save)) return;

            out.print("# OK imu/fifo rate set hz="); out.print(odrName(odr));
            out.println(save ? " saved=yes" : " saved=no");
            printImuRuntimeStatus(ctx, out);
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

            if (is(argv[2], "auto")) {
                if (argc < 4 || is(argv[3], "status")) {
                    if (ctx.printMagHeadingStatus) {
                        ctx.printMagHeadingStatus(out, ctx.printMagHeadingStatusUser);
                    } else {
                        tracker_serial_detail::printErr(out, "mag heading status hook not available");
                    }
                    return;
                }

                if (is(argv[3], "on") || is(argv[3], "enable")) {
                    if (!ctx.setMagHeadingAutoReferenceEnabled) {
                        tracker_serial_detail::printErr(out, "mag heading auto hook not available");
                        return;
                    }

                    const bool ok = ctx.setMagHeadingAutoReferenceEnabled(true, ctx.setMagHeadingAutoReferenceEnabledUser);
                    if (ok) tracker_serial_detail::printOk(out, "mag heading auto-ref enabled");
                    else tracker_serial_detail::printErr(out, "mag heading auto-ref enable failed");
                    return;
                }

                if (is(argv[3], "off") || is(argv[3], "disable")) {
                    if (!ctx.setMagHeadingAutoReferenceEnabled) {
                        tracker_serial_detail::printErr(out, "mag heading auto hook not available");
                        return;
                    }

                    const bool ok = ctx.setMagHeadingAutoReferenceEnabled(false, ctx.setMagHeadingAutoReferenceEnabledUser);
                    if (ok) tracker_serial_detail::printOk(out, "mag heading auto-ref disabled");
                    else tracker_serial_detail::printErr(out, "mag heading auto-ref disable failed");
                    return;
                }

                tracker_serial_detail::printErr(out, "usage: mag heading auto on|off|status");
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
                    tracker_serial_detail::printOk(out, "mag yaw correction stats reset");
                } else {
                    tracker_serial_detail::printErr(out, "mag yaw reset hook not available");
                }
                return;
            }

            if (is(argv[2], "enable")) {
                const bool saveRequested = argc >= 4 && is(argv[3], "save");

                if (ctx.setMagYawCorrectionApplyEnabled) {
                    const bool ok = ctx.setMagYawCorrectionApplyEnabled(true, saveRequested, ctx.setMagYawCorrectionApplyEnabledUser);
                    if (ok) tracker_serial_detail::printOk(out, saveRequested ? "mag yaw correction enabled and saved" : "mag yaw correction enabled");
                    else tracker_serial_detail::printErr(out, "mag yaw correction enable failed");
                } else {
                    tracker_serial_detail::printErr(out, "mag yaw enable hook not available");
                }
                return;
            }

            if (is(argv[2], "disable")) {
                const bool saveRequested = argc >= 4 && is(argv[3], "save");

                if (ctx.setMagYawCorrectionApplyEnabled) {
                    const bool ok = ctx.setMagYawCorrectionApplyEnabled(false, saveRequested, ctx.setMagYawCorrectionApplyEnabledUser);
                    if (ok) tracker_serial_detail::printOk(out, saveRequested ? "mag yaw correction disabled and saved" : "mag yaw correction disabled");
                    else tracker_serial_detail::printErr(out, "mag yaw correction disable failed");
                } else {
                    tracker_serial_detail::printErr(out, "mag yaw disable hook not available");
                }
                return;
            }

            if (!ctx.config) {
                tracker_serial_detail::printErr(out, "config not available");
                return;
            }

            auto saveYawConfigIfRequested = [&](bool saveRequested) -> bool {
                ctx.config->sanitize();
                ctx.config->updateCrc();

                if (!saveRequested) return true;

                if (!ctx.configStore) return false;
                return ctx.configStore->save(*ctx.config);
            };

            auto resetYawStats = [&]() {
                if (ctx.resetMagYawCorrection) {
                    ctx.resetMagYawCorrection(ctx.resetMagYawCorrectionUser);
                }
            };

            if (is(argv[2], "defaults")) {
                const bool saveRequested = argc >= 4 && is(argv[3], "save");

                const bool keepApply = ctx.config->data.magYaw.applyEnabled;
                ctx.config->data.magYaw = TrackerMagYawCorrectionConfigPersisted{};
                ctx.config->data.magYaw.applyEnabled = keepApply;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw defaults save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw defaults applied and saved" : "mag yaw defaults applied");
                return;
            }

            if (is(argv[2], "tc")) {
                if (argc < 4) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw tc <seconds> [save]");
                    return;
                }

                float v = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], v)) {
                    tracker_serial_detail::printErr(out, "invalid time constant");
                    return;
                }

                const bool saveRequested = argc >= 5 && is(argv[4], "save");
                ctx.config->data.magYaw.timeConstantS = v;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw tc save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw tc saved" : "mag yaw tc set");
                return;
            }

            if (is(argv[2], "innovation")) {
                if (argc < 4) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw innovation <deg> [save]");
                    return;
                }

                float v = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], v)) {
                    tracker_serial_detail::printErr(out, "invalid innovation");
                    return;
                }

                const bool saveRequested = argc >= 5 && is(argv[4], "save");
                ctx.config->data.magYaw.maxInnovationDeg = v;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw innovation save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw innovation saved" : "mag yaw innovation set");
                return;
            }

            if (is(argv[2], "gyro_gate")) {
                if (argc < 5) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw gyro_gate <goodDps> <badDps> [save]");
                    return;
                }

                float good = 0.0f;
                float bad = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], good) ||
                    !tracker_serial_detail::parseFloat(argv[4], bad)) {
                    tracker_serial_detail::printErr(out, "invalid gyro gate");
                    return;
                }

                const bool saveRequested = argc >= 6 && is(argv[5], "save");
                ctx.config->data.magYaw.gyroNormGoodDps = good;
                ctx.config->data.magYaw.gyroNormBadDps = bad;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw gyro gate save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw gyro gate saved" : "mag yaw gyro gate set");
                return;
            }

            if (is(argv[2], "horiz_gate")) {
                if (argc < 5) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw horiz_gate <bad> <good> [save]");
                    return;
                }

                float bad = 0.0f;
                float good = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], bad) ||
                    !tracker_serial_detail::parseFloat(argv[4], good)) {
                    tracker_serial_detail::printErr(out, "invalid horizontal gate");
                    return;
                }

                const bool saveRequested = argc >= 6 && is(argv[5], "save");
                ctx.config->data.magYaw.horizontalNormBad = bad;
                ctx.config->data.magYaw.horizontalNormGood = good;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw horizontal gate save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw horizontal gate saved" : "mag yaw horizontal gate set");
                return;
            }

            if (is(argv[2], "accel_gate")) {
                if (argc < 5) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw accel_gate <badTrust> <goodTrust> [save]");
                    return;
                }

                float bad = 0.0f;
                float good = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], bad) ||
                    !tracker_serial_detail::parseFloat(argv[4], good)) {
                    tracker_serial_detail::printErr(out, "invalid accel gate");
                    return;
                }

                const bool saveRequested = argc >= 6 && is(argv[5], "save");
                ctx.config->data.magYaw.accelTrustBad = bad;
                ctx.config->data.magYaw.accelTrustGood = good;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw accel gate save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw accel gate saved" : "mag yaw accel gate set");
                return;
            }

            if (is(argv[2], "age")) {
                if (argc < 4) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw age <ms> [save]");
                    return;
                }

                uint32_t v = 0;
                if (!tracker_serial_detail::parseU32(argv[3], v)) {
                    tracker_serial_detail::printErr(out, "invalid age");
                    return;
                }

                const bool saveRequested = argc >= 5 && is(argv[4], "save");
                ctx.config->data.magYaw.maxMagAgeMs = v;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw age save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw age saved" : "mag yaw age set");
                return;
            }

            if (is(argv[2], "rate")) {
                if (argc < 4) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw rate <deg_s> [save]");
                    return;
                }

                float v = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], v)) {
                    tracker_serial_detail::printErr(out, "invalid max rate");
                    return;
                }

                const bool saveRequested = argc >= 5 && is(argv[4], "save");
                ctx.config->data.magYaw.maxCorrectionRateDegS = v;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw rate save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw rate saved" : "mag yaw rate set");
                return;
            }

            if (is(argv[2], "step")) {
                if (argc < 4) {
                    tracker_serial_detail::printErr(out, "usage: mag yaw step <deg> [save]");
                    return;
                }

                float v = 0.0f;
                if (!tracker_serial_detail::parseFloat(argv[3], v)) {
                    tracker_serial_detail::printErr(out, "invalid max step");
                    return;
                }

                const bool saveRequested = argc >= 5 && is(argv[4], "save");
                ctx.config->data.magYaw.maxCorrectionStepDeg = v;

                if (!saveYawConfigIfRequested(saveRequested)) {
                    tracker_serial_detail::printErr(out, "mag yaw step save failed");
                    return;
                }

                resetYawStats();
                tracker_serial_detail::printOk(out, saveRequested ? "mag yaw step saved" : "mag yaw step set");
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

                if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
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

                if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
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

                if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
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
            if (ctx.config) {
                out.print("fifo_config_watermark_words="); out.println(ctx.config->data.fifo.watermarkWords);
                out.print("fifo_config_max_words_per_drain="); out.println(ctx.config->data.fifo.maxWordsPerDrain);
                out.print("fifo_config_max_drain_rounds_per_event="); out.println(ctx.config->data.fifo.maxDrainRoundsPerEvent);
            }
            return;
        }

        if (is(argv[1], "stats")) {
            printFifoStats(out, ctx.fifo->stats());
            return;
        }

        if (is(argv[1], "watermark") || is(argv[1], "wm")) {
            if (!ctx.config) {
                tracker_serial_detail::printErr(out, "config not available");
                return;
            }
            if (argc < 3) {
                tracker_serial_detail::printErr(out, "usage: fifo watermark <1..255> [save]");
                return;
            }
            uint32_t words = 0;
            if (!tracker_serial_detail::parseU32(argv[2], words) || words < 1 || words > 255) {
                tracker_serial_detail::printErr(out, "invalid watermark; expected 1..255 words");
                return;
            }
            const bool save = (argc >= 4 && is(argv[3], "save"));
            ctx.config->data.fifo.watermarkWords = static_cast<uint8_t>(words);
            ctx.config->sanitize();
            ctx.config->updateCrc();
            if (!liveReconfigureImuFifo(ctx, out)) return;
            if (!saveConfigIfRequested(ctx, out, save)) return;
            out.print("# OK fifo watermark set words="); out.print(words);
            out.println(save ? " saved=yes" : " saved=no");
            return;
        }

        if (is(argv[1], "drain")) {
            if (!ctx.config) {
                tracker_serial_detail::printErr(out, "config not available");
                return;
            }
            if (argc < 4) {
                tracker_serial_detail::printErr(out, "usage: fifo drain <max_words_per_drain> <rounds_per_event> [save]");
                return;
            }
            uint32_t maxWords = 0;
            uint32_t rounds = 0;
            if (!tracker_serial_detail::parseU32(argv[2], maxWords) || maxWords < 16 || maxWords > 4096) {
                tracker_serial_detail::printErr(out, "invalid max_words_per_drain; expected 16..4096");
                return;
            }
            if (!tracker_serial_detail::parseU32(argv[3], rounds) || rounds < 1 || rounds > 32) {
                tracker_serial_detail::printErr(out, "invalid rounds_per_event; expected 1..32");
                return;
            }
            const bool save = (argc >= 5 && is(argv[4], "save"));
            ctx.config->data.fifo.maxWordsPerDrain = static_cast<uint16_t>(maxWords);
            ctx.config->data.fifo.maxDrainRoundsPerEvent = static_cast<uint8_t>(rounds);
            ctx.config->sanitize();
            ctx.config->updateCrc();
            if (!liveReconfigureImuFifo(ctx, out)) return;
            if (!saveConfigIfRequested(ctx, out, save)) return;
            out.print("# OK fifo drain set max_words="); out.print(maxWords);
            out.print(" rounds="); out.print(rounds);
            out.println(save ? " saved=yes" : " saved=no");
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
                ctx.config->data.gyroCalMeta = TrackerGyroCalibrationMetaPersisted{};
                ctx.config->data.accelCal = TrackerAccelCalibrationConfig{};
                ctx.config->data.accelCalQuality = TrackerAccelCalibrationQualityPersisted{};
                ctx.config->data.magCal = TrackerMagCalibrationConfig{};
                ctx.config->data.magCalQuality = TrackerMagCalibrationQualityPersisted{};
                ctx.config->updateCrc();
            }
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
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
            ctx.config->noteGyroBiasCalibrationCaptured(millis());
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
                ctx.config->data.gyroCalMeta = TrackerGyroCalibrationMetaPersisted{};
                ctx.config->updateCrc();
            }
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
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
            ctx.config->noteGyroBiasCalibrationCaptured(millis());
            if (ctx.gyroTempComp) ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
        }
        if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);

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
                printAccelCal(out, ctx.accelCalRunner->calibration());
                tracker_serial_detail::printErr(out, "accel calibration rejected by quality gates; see quality_flags");
                return;
            }
            printAccelCal(out, ctx.accelCalRunner->calibration());
            if (ctx.imuCal) ctx.accelCalRunner->applyToImuCalibration(*ctx.imuCal);
            if (ctx.config && ctx.imuCal) {
                ctx.config->captureFromImuCalibration(*ctx.imuCal);
                ctx.config->captureFromAccelCalibrationQuality(ctx.accelCalRunner->calibration(), millis());
            }
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
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
            if (ctx.accelCalRunner->calibration().result().valid) {
                ctx.config->captureFromAccelCalibrationQuality(ctx.accelCalRunner->calibration(), millis());
            }
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
                ctx.config->data.accelCalQuality = TrackerAccelCalibrationQualityPersisted{};
                ctx.config->updateCrc();
            }
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
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

        auto saveTempConfigIfRequested = [&](bool saveRequested) -> bool {
            if (!ctx.config) return false;

            ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
            ctx.config->sanitize();
            ctx.config->updateCrc();

            if (!saveRequested) return true;

            if (!ctx.configStore) return false;
            return ctx.configStore->save(*ctx.config);
        };

        const float currentTempC =
            ctx.calibrationIo ? ctx.calibrationIo->latestTempC : 25.0f;

        if (argc < 3 || is(argv[2], "print")) {
            const auto s = ctx.gyroTempComp->snapshot(currentTempC);

            out.println("# GYRO TEMP COMP");
            out.print("temp_comp_valid="); out.println(s.valid ? "yes" : "no");
            out.print("temp_comp_enabled="); out.println(s.enabled ? "yes" : "no");
            out.print("temp_learning_enabled="); out.println(s.learningEnabled ? "yes" : "no");

            out.print("current_temp_c="); out.println(s.currentTempC, 3);
            out.print("reference_temp_c="); out.println(s.referenceTempC, 3);
            out.print("delta_temp_c="); out.println(s.deltaTempC, 3);

            tracker_serial_detail::printVec3Line(out, "reference_bias_dps", s.referenceBiasDps, 6);
            tracker_serial_detail::printVec3Line(out, "slope_dps_per_c", s.slopeDpsPerC, 8);
            tracker_serial_detail::printVec3Line(out, "current_bias_dps", s.currentBiasDps, 6);
            out.print("calibrated_range_valid="); out.println(s.hasCalibratedRange ? "yes" : "no");
            out.print("calibrated_temp_min_c="); out.println(s.calibratedTempMinC, 3);
            out.print("calibrated_temp_max_c="); out.println(s.calibratedTempMaxC, 3);
            out.print("temp_out_of_range="); out.println(s.tempOutOfRange ? "yes" : "no");
            out.print("fit_quality="); out.println(s.fitQuality, 6);
            out.print("fit_residual_before_dps="); out.println(s.fitResidualBeforeDps, 6);
            out.print("fit_residual_after_dps="); out.println(s.fitResidualAfterDps, 6);

            out.print("learn_accepted="); out.println(s.learnAccepted);
            out.print("learn_rejected="); out.println(s.learnRejected);
            return;
        }

        if (is(argv[2], "enable")) {
            const bool saveRequested = argc >= 4 && is(argv[3], "save");

            ctx.gyroTempComp->setEnabled(true);

            if (!saveTempConfigIfRequested(saveRequested)) {
                tracker_serial_detail::printErr(out, "cal temp enable save failed");
                return;
            }

            tracker_serial_detail::printOk(out, saveRequested ? "gyro temp compensation enabled and saved" : "gyro temp compensation enabled");
            return;
        }

        if (is(argv[2], "disable")) {
            const bool saveRequested = argc >= 4 && is(argv[3], "save");

            ctx.gyroTempComp->setEnabled(false);

            if (!saveTempConfigIfRequested(saveRequested)) {
                tracker_serial_detail::printErr(out, "cal temp disable save failed");
                return;
            }

            tracker_serial_detail::printOk(out, saveRequested ? "gyro temp compensation disabled and saved" : "gyro temp compensation disabled");
            return;
        }

        if (is(argv[2], "set_slope")) {
            if (argc < 6) {
                tracker_serial_detail::printErr(out, "usage: cal temp set_slope X Y Z [save] ; values in dps/C");
                return;
            }

            float x = 0, y = 0, z = 0;
            if (!tracker_serial_detail::parseFloat(argv[3], x) ||
                !tracker_serial_detail::parseFloat(argv[4], y) ||
                !tracker_serial_detail::parseFloat(argv[5], z)) {
                tracker_serial_detail::printErr(out, "invalid slope values");
                return;
            }

            const bool saveRequested = argc >= 7 && is(argv[6], "save");

            ctx.gyroTempComp->setSlopeDpsPerC(Vec3(x, y, z));

            if (!saveTempConfigIfRequested(saveRequested)) {
                tracker_serial_detail::printErr(out, "cal temp set_slope save failed");
                return;
            }

            tracker_serial_detail::printOk(out, saveRequested ? "temperature slope applied and saved" : "temperature slope applied to RAM");
            return;
        }

        if (is(argv[2], "fit_static")) {
            const bool saveRequested = argc >= 4 && is(argv[3], "save");

            if (!ctx.fitGyroTempFromLastStatic) {
                tracker_serial_detail::printErr(out, "fit_static hook not available");
                return;
            }

            const bool ok = ctx.fitGyroTempFromLastStatic(
                saveRequested,
                out,
                ctx.fitGyroTempFromLastStaticUser
            );

            if (!ok) {
                tracker_serial_detail::printErr(out, "temperature fit from static test failed");
            }
            return;
        }

        if (is(argv[2], "clear")) {
            const bool saveRequested = argc >= 4 && is(argv[3], "save");

            ctx.gyroTempComp->setSlopeRadSPerC(Vec3::zero());

            if (ctx.config) {
                ctx.config->data.gyroCal.tempCompValid = false;
                ctx.config->data.gyroCal.tempSlopeRadSPerC = Vec3::zero();
                ctx.config->data.gyroTempQuality = TrackerGyroTempQualityConfigPersisted{};
                ctx.config->sanitize();
                ctx.config->updateCrc();

                if (saveRequested) {
                    if (!ctx.configStore || !ctx.configStore->save(*ctx.config)) {
                        tracker_serial_detail::printErr(out, "cal temp clear save failed");
                        return;
                    }
                }
            }

            tracker_serial_detail::printOk(out, saveRequested ? "temperature compensation cleared and saved" : "temperature compensation cleared in RAM");
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
            out.print("last_seen_t_us="); tracker_serial_detail::printU64Dec(out, st.lastSeenTimestampUs); out.println();
            out.print("last_integrated_t_us="); tracker_serial_detail::printU64Dec(out, st.lastIntegratedTimestampUs); out.println();
            out.print("bad_dt_rejects="); out.println(st.skippedBadDt);
            out.print("startup_accel_rejects="); out.println(st.startupAccelRejectedCount);
            out.print("large_dt_clamps="); out.println(st.clampedLargeDt);
            out.print("last_accel_trust="); out.println(st.lastAccelGate.trust, 6);
            out.print("last_accel_norm_trust="); out.println(st.lastAccelGate.normTrust, 6);
            out.print("last_accel_innovation_trust="); out.println(st.lastAccelGate.innovationTrust, 6);
            out.print("last_accel_variance_trust="); out.println(st.lastAccelGate.varianceTrust, 6);
            out.print("last_gyro_motion_trust="); out.println(st.lastAccelGate.gyroMotionTrust, 6);
            out.print("accel_norm_variance_g2="); out.println(st.accelNormVarianceG2, 9);
            return;
        }

        if (is(argv[1], "config")) {
            printAhrsConfig(out, ctx.ahrs->config());
            return;
        }

        if (is(argv[1], "reset")) {
            ctx.ahrs->reset();
            if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
            tracker_serial_detail::printOk(out, "ahrs reset");
            return;
        }

        if (!ctx.config) {
            tracker_serial_detail::printErr(out, "config not available");
            return;
        }

        auto applyAndMaybeSave = [&](bool saveRequested, const char* okMsg, const char* saveErr) -> void {
            ctx.config->sanitize();
            ctx.ahrs->setConfig(ctx.config->makeAhrsConfig());
            ctx.config->updateCrc();
            if (saveRequested) {
                if (!ctx.configStore || !ctx.configStore->save(*ctx.config)) {
                    tracker_serial_detail::printErr(out, saveErr);
                    return;
                }
            }
            tracker_serial_detail::printOk(out, okMsg);
        };

        if (is(argv[1], "defaults")) {
            const bool saveRequested = argc >= 3 && is(argv[2], "save");
            ctx.config->data.ahrsRuntime = TrackerAhrsRuntimeConfigPersisted{};
            ctx.config->data.ahrs.accelCorrectionGain = 3.0f;
            ctx.config->data.ahrs.useAccelCorrection = true;
            applyAndMaybeSave(saveRequested,
                              saveRequested ? "ahrs defaults applied and saved" : "ahrs defaults applied",
                              "ahrs defaults save failed");
            return;
        }

        if (is(argv[1], "accel") || is(argv[1], "adaptive")) {
            if (argc < 3) {
                tracker_serial_detail::printErr(out, "usage: ahrs accel|adaptive on|off [save]");
                return;
            }
            bool enabled = false;
            if (!tracker_serial_detail::parseBool(argv[2], enabled)) {
                tracker_serial_detail::printErr(out, "expected on/off");
                return;
            }
            const bool saveRequested = argc >= 4 && is(argv[3], "save");
            if (is(argv[1], "accel")) {
                ctx.config->data.ahrsRuntime.accelCorrectionEnabled = enabled;
                ctx.config->data.ahrs.useAccelCorrection = enabled;
            } else {
                ctx.config->data.ahrsRuntime.adaptiveAccelCorrection = enabled;
            }
            applyAndMaybeSave(saveRequested,
                              saveRequested ? "ahrs gate saved" : "ahrs gate set",
                              "ahrs gate save failed");
            return;
        }

        if (is(argv[1], "accel_kp") || is(argv[1], "max_step")) {
            if (argc < 3) {
                tracker_serial_detail::printErr(out, "usage: ahrs accel_kp <gain> [save] | max_step <deg> [save]");
                return;
            }
            float v = 0.0f;
            if (!tracker_serial_detail::parseFloat(argv[2], v)) {
                tracker_serial_detail::printErr(out, "invalid value");
                return;
            }
            const bool saveRequested = argc >= 4 && is(argv[3], "save");
            if (is(argv[1], "accel_kp")) {
                ctx.config->data.ahrsRuntime.accelKp = v;
                ctx.config->data.ahrs.accelCorrectionGain = v;
            } else {
                ctx.config->data.ahrsRuntime.maxAccelCorrectionDegPerUpdate = v;
            }
            applyAndMaybeSave(saveRequested,
                              saveRequested ? "ahrs parameter saved" : "ahrs parameter set",
                              "ahrs parameter save failed");
            return;
        }

        if (is(argv[1], "accel_norm") || is(argv[1], "accel_innovation") ||
            is(argv[1], "accel_var") || is(argv[1], "gyro_gate") || is(argv[1], "dt")) {
            if (argc < 4) {
                tracker_serial_detail::printErr(out, "usage: ahrs accel_norm|accel_innovation|accel_var|gyro_gate|dt <good/min> <bad/max> [save]");
                return;
            }
            float a = 0.0f;
            float b = 0.0f;
            if (!tracker_serial_detail::parseFloat(argv[2], a) ||
                !tracker_serial_detail::parseFloat(argv[3], b)) {
                tracker_serial_detail::printErr(out, "invalid values");
                return;
            }
            const bool saveRequested = argc >= 5 && is(argv[4], "save");
            if (is(argv[1], "accel_norm")) {
                ctx.config->data.ahrsRuntime.accelNormGoodErrorG = a;
                ctx.config->data.ahrsRuntime.accelNormBadErrorG = b;
            } else if (is(argv[1], "accel_innovation")) {
                ctx.config->data.ahrsRuntime.accelInnovationGoodDeg = a;
                ctx.config->data.ahrsRuntime.accelInnovationBadDeg = b;
            } else if (is(argv[1], "accel_var")) {
                ctx.config->data.ahrsRuntime.accelNormStdGoodG = a;
                ctx.config->data.ahrsRuntime.accelNormStdBadG = b;
            } else if (is(argv[1], "gyro_gate")) {
                ctx.config->data.ahrsRuntime.gyroMotionGoodDps = a;
                ctx.config->data.ahrsRuntime.gyroMotionBadDps = b;
            } else {
                ctx.config->data.ahrsRuntime.minDtS = a * 0.001f;
                ctx.config->data.ahrsRuntime.maxDtS = b * 0.001f;
            }
            applyAndMaybeSave(saveRequested,
                              saveRequested ? "ahrs gate saved" : "ahrs gate set",
                              "ahrs gate save failed");
            return;
        }

        tracker_serial_detail::printErr(out, "unknown ahrs command");
    }

    static void printAhrsConfig(Stream& out, const Ahrs6DofConfig& cfg) {
        out.println("# AHRS CONFIG");
        out.print("accelCorrectionEnabled="); out.println(cfg.accelCorrectionEnabled ? "yes" : "no");
        out.print("adaptiveAccelCorrection="); out.println(cfg.adaptiveAccelCorrection ? "yes" : "no");
        out.print("accelKp="); out.println(cfg.accelKp, 6);
        out.print("minDtS="); out.println(cfg.minDtS, 7);
        out.print("maxDtS="); out.println(cfg.maxDtS, 7);
        out.print("clampLargeDt="); out.println(cfg.clampLargeDt ? "yes" : "no");
        out.print("maxAccelCorrectionDegPerUpdate="); out.println(cfg.maxAccelCorrectionRadPerUpdate * MATH_RAD_TO_DEG, 6);
        out.print("accelNormGoodBadErrorG="); out.print(cfg.accelNormGoodErrorG, 6); out.print(','); out.println(cfg.accelNormBadErrorG, 6);
        out.print("accelInnovationGoodBadDeg="); out.print(cfg.accelInnovationGoodRad * MATH_RAD_TO_DEG, 3); out.print(','); out.println(cfg.accelInnovationBadRad * MATH_RAD_TO_DEG, 3);
        out.print("accelNormStdGoodBadG="); out.print(std::sqrt(cfg.accelNormVarianceGoodG2), 6); out.print(','); out.println(std::sqrt(cfg.accelNormVarianceBadG2), 6);
        out.print("accelNormVarianceAlpha="); out.println(cfg.accelNormVarianceAlpha, 6);
        out.print("gyroMotionGoodBadDps="); out.print(cfg.gyroNormAccelTrustGoodRadS * MATH_RAD_TO_DEG, 3); out.print(','); out.println(cfg.gyroNormAccelTrustBadRadS * MATH_RAD_TO_DEG, 3);
        out.print("normalizeEvery="); out.println(cfg.normalizeEvery);
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

    static void cmdLog(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);
        if (!ctx.logState) {
            tracker_serial_detail::printErr(out, "log state not available");
            return;
        }

        if (argc < 2) {
            out.print("log_mode="); out.println(logModeName(ctx.logState->mode));
            out.print("log_rate_hz="); out.println(ctx.logState->rateHz);
            out.print("log_sequence="); out.println(ctx.logState->sequence);
            return;
        }

        if (is(argv[1], "rate")) {
            if (argc < 3) {
                tracker_serial_detail::printErr(out, "usage: log rate <hz>");
                return;
            }
            uint32_t hz = 0;
            if (!tracker_serial_detail::parseU32(argv[2], hz) || hz == 0 || hz > 200) {
                tracker_serial_detail::printErr(out, "invalid log rate; expected 1..200");
                return;
            }
            ctx.logState->rateHz = static_cast<uint16_t>(hz);
            ctx.logState->lastEmitUs = 0;
            ctx.logState->lastMagEmitUs = 0;
            tracker_serial_detail::printOk(out, "log rate set");
            return;
        }

        if (is(argv[1], "header")) {
            if (ctx.emitLogHeader) ctx.emitLogHeader(out, ctx.emitLogHeaderUser);
            else tracker_serial_detail::printErr(out, "log header hook not available");
            return;
        }

        if (is(argv[1], "summary")) {
            if (ctx.printLogSummary) ctx.printLogSummary(out, ctx.printLogSummaryUser);
            else tracker_serial_detail::printErr(out, "log summary hook not available");
            return;
        }

        if (is(argv[1], "reset")) {
            ctx.logState->sequence = 0;
            ctx.logState->lastEmitUs = 0;
            ctx.logState->lastMagEmitUs = 0;
            if (ctx.resetLogCounters) ctx.resetLogCounters(ctx.resetLogCountersUser);
            tracker_serial_detail::printOk(out, "log counters reset");
            return;
        }

        if (is(argv[1], "off") || is(argv[1], "stop")) {
            ctx.logState->mode = TrackerLogMode::Off;
            ctx.logState->lastEmitUs = 0;
            ctx.logState->lastMagEmitUs = 0;
            tracker_serial_detail::printOk(out, "log stopped");
            return;
        }

        TrackerLogMode mode = TrackerLogMode::Off;
        bool start = false;
        if (is(argv[1], "start")) {
            start = true;
            if (argc >= 3) {
                if (!parseLogMode(argv[2], mode) || mode == TrackerLogMode::Off) {
                    tracker_serial_detail::printErr(out, "usage: log start [basic|full]");
                    return;
                }
            } else {
                mode = TrackerLogMode::Basic;
            }
        } else if (parseLogMode(argv[1], mode)) {
            start = (mode != TrackerLogMode::Off);
        } else {
            tracker_serial_detail::printErr(out, "unknown log command; use off|basic|full|start|stop|rate|header|summary|reset");
            return;
        }

        ctx.logState->mode = mode;
        ctx.logState->lastEmitUs = 0;
        ctx.logState->lastMagEmitUs = 0;
        if (mode == TrackerLogMode::Off) {
            tracker_serial_detail::printOk(out, "log stopped");
            return;
        }

        out.print("# OK log mode ");
        out.println(logModeName(mode));
        if (start && ctx.emitLogHeader) {
            ctx.emitLogHeader(out, ctx.emitLogHeaderUser);
        }
    }

    static void cmdBias(TrackerSerialCommandContext& ctx, int argc, char** argv) {
        Stream& out = stream(ctx);

        if (argc < 2 || is(argv[1], "status")) {
            if (ctx.printRuntimeGyroBiasStatus) ctx.printRuntimeGyroBiasStatus(out, ctx.printRuntimeGyroBiasStatusUser);
            else tracker_serial_detail::printErr(out, "runtime gyro bias status hook not available");
            return;
        }

        if (is(argv[1], "on") || is(argv[1], "enable")) {
            if (!ctx.setRuntimeGyroBiasEnabled || !ctx.setRuntimeGyroBiasEnabled(true, ctx.setRuntimeGyroBiasEnabledUser)) {
                tracker_serial_detail::printErr(out, "runtime gyro bias enable failed");
                return;
            }
            tracker_serial_detail::printOk(out, "runtime gyro bias estimator enabled in RAM");
            return;
        }

        if (is(argv[1], "off") || is(argv[1], "disable")) {
            if (!ctx.setRuntimeGyroBiasEnabled || !ctx.setRuntimeGyroBiasEnabled(false, ctx.setRuntimeGyroBiasEnabledUser)) {
                tracker_serial_detail::printErr(out, "runtime gyro bias disable failed");
                return;
            }
            tracker_serial_detail::printOk(out, "runtime gyro bias estimator disabled");
            return;
        }

        if (is(argv[1], "reset")) {
            if (ctx.resetRuntimeGyroBiasEstimator) {
                ctx.resetRuntimeGyroBiasEstimator(ctx.resetRuntimeGyroBiasEstimatorUser);
                tracker_serial_detail::printOk(out, "runtime gyro bias estimator counters and runtime trim reset");
            } else {
                tracker_serial_detail::printErr(out, "runtime gyro bias reset hook not available");
            }
            return;
        }

        tracker_serial_detail::printErr(out, "unknown bias command; use status|on|off|reset");
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
                tracker_serial_detail::printErr(out, "usage: output mode debug");
                return;
            }
            if (is(argv[2], "debug")) {
                ctx.config->data.output.packetFormat = 0;
                ctx.config->updateCrc();
                tracker_serial_detail::printOk(out, "output mode set");
                return;
            }
            if (is(argv[2], "binary") || is(argv[2], "slimevr")) {
                tracker_serial_detail::printErr(out, "NOT_IMPLEMENTED: output backend is not available in this build");
                return;
            }
            tracker_serial_detail::printErr(out, "unknown output mode");
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
        if (ctx.ahrs) ctx.ahrs->setConfig(ctx.config->makeAhrsConfig());
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
        if (ctx.setSpiFrequency) ctx.setSpiFrequency(ctx.config->data.hardware.spiHz, ctx.setSpiFrequencyUser);
        if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
    }

    static void captureRuntimeToConfig(TrackerSerialCommandContext& ctx) {
        if (!ctx.config) return;
        if (ctx.imuCal) ctx.config->captureFromImuCalibration(*ctx.imuCal);
        if (ctx.gyroTempComp) ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
        if (ctx.accelCalRunner && ctx.accelCalRunner->calibration().result().valid) {
            ctx.config->captureFromAccelCalibrationQuality(ctx.accelCalRunner->calibration(), millis());
        }
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
        out.print("quality_score="); out.println(r.qualityScore, 6);
        out.print("quality_flags=0x"); out.println(r.qualityFlags, HEX);
        printAccelQualityFlags(out, r.qualityFlags);
        tracker_serial_detail::printVec3Line(out, "accel_bias_g", r.biasG, 8);
        tracker_serial_detail::printVec3Line(out, "accel_scale_diag", r.scale, 8);
        out.print("accel_matrix_row0="); out.print(r.scaleMatrix.m[0][0], 8); out.print(','); out.print(r.scaleMatrix.m[0][1], 8); out.print(','); out.println(r.scaleMatrix.m[0][2], 8);
        out.print("accel_matrix_row1="); out.print(r.scaleMatrix.m[1][0], 8); out.print(','); out.print(r.scaleMatrix.m[1][1], 8); out.print(','); out.println(r.scaleMatrix.m[1][2], 8);
        out.print("accel_matrix_row2="); out.print(r.scaleMatrix.m[2][0], 8); out.print(','); out.print(r.scaleMatrix.m[2][1], 8); out.print(','); out.println(r.scaleMatrix.m[2][2], 8);
        out.print("max_face_norm_error_g="); out.println(r.maxFaceNormErrorG, 8);
        out.print("max_axis_residual_g="); out.println(r.maxAxisResidualG, 8);
        out.print("face_norm_errors_g=");
        for (uint8_t i = 0; i < 6; ++i) {
            if (i) out.print(',');
            out.print(r.faceNormErrorG[i], 8);
        }
        out.println();
        out.print("face_axis_residuals_g=");
        for (uint8_t i = 0; i < 6; ++i) {
            if (i) out.print(',');
            out.print(r.faceAxisResidualG[i], 8);
        }
        out.println();
    }

    static void printAccelQualityFlags(Stream& out, uint32_t flags) {
        if (flags == accel_cal_quality_flags::OK) {
            out.println("quality_flag_names=OK");
            return;
        }

        out.print("quality_flag_names=");
        bool first = true;
        const uint32_t known[] = {
            accel_cal_quality_flags::MISSING_FACE,
            accel_cal_quality_flags::TOO_FEW_SAMPLES,
            accel_cal_quality_flags::FACE_VARIANCE_HIGH,
            accel_cal_quality_flags::FACE_NORM_IMPLAUSIBLE,
            accel_cal_quality_flags::FACE_DIRECTION_BAD,
            accel_cal_quality_flags::AXIS_SEPARATION_LOW,
            accel_cal_quality_flags::BIAS_IMPLAUSIBLE,
            accel_cal_quality_flags::SCALE_IMPLAUSIBLE,
            accel_cal_quality_flags::NORM_RESIDUAL_HIGH,
            accel_cal_quality_flags::AXIS_RESIDUAL_HIGH,
        };
        for (uint8_t i = 0; i < sizeof(known) / sizeof(known[0]); ++i) {
            if ((flags & known[i]) == 0) continue;
            if (!first) out.print('|');
            first = false;
            out.print(Accel6PosCalibration::qualityFlagName(known[i]));
        }
        out.println();
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

    static bool parseLogMode(const char* s, TrackerLogMode& mode) {
        if (is(s, "off"))   { mode = TrackerLogMode::Off; return true; }
        if (is(s, "basic")) { mode = TrackerLogMode::Basic; return true; }
        if (is(s, "full"))  { mode = TrackerLogMode::Full; return true; }
        return false;
    }

    static const char* logModeName(TrackerLogMode mode) {
        switch (mode) {
            case TrackerLogMode::Off:   return "off";
            case TrackerLogMode::Basic: return "basic";
            case TrackerLogMode::Full:  return "full";
        }
        return "unknown";
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

    size_t poll(size_t maxBytes = 0) {
        if (!ctx_ || !ctx_->io) return 0;

        size_t consumed = 0;
        Stream& s = *ctx_->io;
        while (s.available() > 0) {
            if (maxBytes > 0 && consumed >= maxBytes) break;
            const char c = static_cast<char>(s.read());
            feed(c);
            consumed++;
        }
        return consumed;
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
    tracker_serial_detail::printU64Dec(out, tUs);
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
    tracker_serial_detail::printU64Dec(out, raw.t_us);
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
    tracker_serial_detail::printU64Dec(out, tUs);
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
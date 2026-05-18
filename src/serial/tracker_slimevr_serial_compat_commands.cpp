#include "serial/tracker_slimevr_serial_compat_commands.hpp"

#include <Arduino.h>
#include <cstdint>
#include <cstring>

#include "config/tracker_config.hpp"
#include "config/tracker_config_store.hpp"
#include "config/tracker_network_config.hpp"
#include "network/wifi_manager.hpp"
#include "output/slimevr_packet_writer.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "runtime/tracker_console_suppress.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "sensor/calibration.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "serial/tracker_config_commands.hpp"
#include "serial/tracker_calibration_commands.hpp"
#include "serial/tracker_network_commands.hpp"
#include "serial/tracker_serial_parse.hpp"
#include "serial/tracker_serial_print.hpp"
#include "serial/tracker_system_commands.hpp"
#include "serial/tracker_slimevr_commands.hpp"
#include "defines.h"

namespace tracker {
namespace {

Stream& outFor(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

bool is(const char* a, const char* b) {
    return tracker_serial_detail::eqIgnoreCase(a, b);
}

void info(Stream& out, const char* msg) {
    out.print("[INFO ] [SerialCommands] ");
    out.println(msg ? msg : "");
}

void warn(Stream& out, const char* msg) {
    out.print("[WARN ] [SerialCommands] ");
    out.println(msg ? msg : "");
}

void error(Stream& out, const char* msg) {
    out.print("[ERROR] [SerialCommands] ");
    out.println(msg ? msg : "");
}

void copyBounded(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    if (!src) src = "";
    std::strncpy(dst, src, dstSize - 1);
    dst[dstSize - 1] = '\0';
}

const char* yesNo(bool v) { return v ? "true" : "false"; }

uint8_t statusCode(TrackerSerialCommandContext& ctx) {
    if (!ctx.lsm || !ctx.lsm->isInitialized()) return 3; // sensor/error-ish
    if (ctx.slimevrRuntime && ctx.slimevrRuntime->serverFound()) return 2;
    if (ctx.wifiManager && ctx.wifiManager->connected()) return 1;
    return 0;
}

uint8_t wifiStateCode(TrackerSerialCommandContext& ctx) {
    if (!ctx.wifiManager) return 0;
    switch (ctx.wifiManager->state()) {
        case TrackerWifiState::Disabled:   return 0;
        case TrackerWifiState::Idle:       return 1;
        case TrackerWifiState::Connecting: return 2;
        case TrackerWifiState::Backoff:    return 3;
        case TrackerWifiState::Connected:  return 5;
    }
    return 0;
}

void printIp(Stream& out, uint32_t ipv4) {
    out.print(static_cast<unsigned int>((ipv4 >> 24) & 0xFFu));
    out.print('.');
    out.print(static_cast<unsigned int>((ipv4 >> 16) & 0xFFu));
    out.print('.');
    out.print(static_cast<unsigned int>((ipv4 >> 8) & 0xFFu));
    out.print('.');
    out.print(static_cast<unsigned int>(ipv4 & 0xFFu));
}

void printMac(Stream& out, const uint8_t mac[6]) {
    for (int i = 0; i < 6; ++i) {
        if (i) out.print(':');
        if (mac[i] < 0x10) out.print('0');
        out.print(static_cast<unsigned int>(mac[i]), HEX);
    }
}

void printCompatConfig(Stream& out) {
    out.print("BOARD="); out.println(10); // LOLIN_C3_MINI-compatible metadata used by UDP handshake.
    out.print("IMU="); out.println(static_cast<unsigned int>(SlimeVRImuType::LSM6DSV));
    out.print("SECOND_IMU="); out.println(0);
    out.print("IMU_ROTATION="); out.println(0.0f, 6);
    out.print("SECOND_IMU_ROTATION="); out.println(0.0f, 6);
    out.print("BATTERY_MONITOR="); out.println(0);
    out.print("BATTERY_SHIELD_RESISTANCE="); out.println(0);
    out.print("BATTERY_SHIELD_R1="); out.println(0);
    out.print("BATTERY_SHIELD_R2="); out.println(0);
    out.print("PIN_IMU_SDA="); out.println(-1);
    out.print("PIN_IMU_SCL="); out.println(-1);
    out.print("PIN_IMU_INT="); out.println(cfg::PIN_LSM_INT1);
    out.print("PIN_IMU_INT_2="); out.println(-1);
#ifdef TRACKER_BATTERY_ADC_PIN
    out.print("PIN_BATTERY_LEVEL="); out.println(TRACKER_BATTERY_ADC_PIN);
#else
    out.print("PIN_BATTERY_LEVEL="); out.println(-1);
#endif
    out.print("LED_PIN="); out.println(TRACKER_STATUS_LED_PIN);
    out.print("LED_INVERTED="); out.println(TRACKER_STATUS_LED_ACTIVE_LOW ? 1 : 0);
}

void printCompatInfo(TrackerSerialCommandContext& ctx, bool includeGit) {
    Stream& out = outFor(ctx);
    const TrackerWifiManagerStatus wifi = ctx.wifiManager ? ctx.wifiManager->status() : TrackerWifiManagerStatus{};
    const Quat q = ctx.ahrs && ctx.ahrs->initialized() ? ctx.ahrs->quaternionPositiveW() : Quat::identity();

    out.print("[INFO ] [SerialCommands] SlimeVR Tracker, board: 10, hardware: 6, protocol: ");
    out.print(static_cast<unsigned int>(SLIMEVR_PROTOCOL_VERSION));
    out.print(", firmware: track-fw, address: ");
    printIp(out, wifi.ipv4);
    out.print(", mac: ");
    printMac(out, wifi.mac);
    out.print(", status: ");
    out.print(static_cast<unsigned int>(statusCode(ctx)));
    out.print(", wifi state: ");
    out.println(static_cast<unsigned int>(wifiStateCode(ctx)));

    info(out, "Vendor: Unknown, product: ESP32-C3 LSM6DSV SlimeVR Tracker");

    out.print("[INFO ] [SerialCommands] Sensor[0]: LSM6DSV (");
    out.print(q.w, 3); out.print(' ');
    out.print(q.x, 3); out.print(' ');
    out.print(q.y, 3); out.print(' ');
    out.print(q.z, 3); out.print(") is working: ");
    out.print(ctx.lsm && ctx.lsm->isInitialized() ? "true" : "false");
    out.print(", had data: ");
    out.println(ctx.lastImuSampleSequence && *ctx.lastImuSampleSequence != 0 ? "true" : "false");

    if (ctx.config && ctx.config->data.magCal.driverEnabled) {
        info(out, "Sensor[0] magnetometer: QMC6309");
    }

    out.print("[INFO ] [SerialCommands] Battery voltage: ");
    out.print(0.0f, 3);
    out.println(", level: 0.0%");

    if (includeGit) {
        info(out, "Git commit: custom-esp32c3-lsm6dsv");
    }
}

void printCompatTest(TrackerSerialCommandContext& ctx) {
    Stream& out = outFor(ctx);
    const TrackerWifiManagerStatus wifi = ctx.wifiManager ? ctx.wifiManager->status() : TrackerWifiManagerStatus{};
    const Quat q = ctx.ahrs && ctx.ahrs->initialized() ? ctx.ahrs->quaternionPositiveW() : Quat::identity();
    const bool sensorWorking = ctx.lsm && ctx.lsm->isInitialized();
    const bool hadData = ctx.lastImuSampleSequence && *ctx.lastImuSampleSequence != 0;

    out.print("[INFO ] [SerialCommands] [TEST] Board: 10, hardware: 6, protocol: ");
    out.print(static_cast<unsigned int>(SLIMEVR_PROTOCOL_VERSION));
    out.print(", firmware: track-fw, address: ");
    printIp(out, wifi.ipv4);
    out.print(", mac: ");
    printMac(out, wifi.mac);
    out.print(", status: ");
    out.print(static_cast<unsigned int>(statusCode(ctx)));
    out.print(", wifi state: ");
    out.println(static_cast<unsigned int>(wifiStateCode(ctx)));

    out.print("[INFO ] [SerialCommands] [TEST] Sensor[0]: LSM6DSV (");
    out.print(q.w, 3); out.print(' ');
    out.print(q.x, 3); out.print(' ');
    out.print(q.y, 3); out.print(' ');
    out.print(q.z, 3); out.print(") is working: ");
    out.print(sensorWorking ? "true" : "false");
    out.print(", had data: ");
    out.println(hadData ? "true" : "false");

    if (ctx.config && ctx.config->data.magCal.driverEnabled) {
        info(out, "[TEST] Sensor[0] magnetometer: QMC6309");
    } else {
        info(out, "[TEST] Sensor[0] has no magnetometer attached");
    }

    if (hadData) {
        info(out, "[TEST] Sensor[0] sent some data, looks working.");
    } else {
        error(out, "[TEST] Sensor[0] didn't send any data yet!");
    }
}

bool saveNetworkConfig(TrackerSerialCommandContext& ctx) {
    if (!ctx.networkConfig || !ctx.networkConfigStore) return false;
    ctx.networkConfig->sanitize();
    if (!ctx.networkConfigStore->save(*ctx.networkConfig)) return false;
    if (ctx.networkConfigLoadedFromNvs) *ctx.networkConfigLoadedFromNvs = true;
    return true;
}

void applyWifiConfig(TrackerSerialCommandContext& ctx) {
    if (!ctx.networkConfig) return;
    ctx.networkConfig->sanitize();
    if (ctx.wifiManager) {
        TrackerWifiManagerConfig cfg;
        cfg.enabled = ctx.networkConfig->data.wifiEnabled;
        cfg.credentialsValid = ctx.networkConfig->data.credentialsValid;
        cfg.ssid = ctx.networkConfig->data.ssid;
        cfg.password = ctx.networkConfig->data.password;
        cfg.hostname = ctx.networkConfig->data.deviceName;
        cfg.connectTimeoutMs = 15000;
        cfg.reconnectBackoffMs = 5000;
        cfg.statusPollIntervalMs = 250;
        ctx.wifiManager->configure(cfg);
    }
}

void startSlimeRuntime(TrackerSerialCommandContext& ctx) {
    if (!ctx.slimevrRuntime || !ctx.networkConfig) return;
    char* start[] = { const_cast<char*>("slime"), const_cast<char*>("start") };
    trackerSerialDispatchSlimeVRCommand(ctx, 2, start);
}

bool setWifiCredentials(TrackerSerialCommandContext& ctx, const char* ssid, const char* password) {
    if (!ctx.networkConfig || !ctx.wifiManager) return false;
    if (!ssid || ssid[0] == '\0' || std::strlen(ssid) > 32) return false;
    if (password && std::strlen(password) > 64) return false;

    copyBounded(ctx.networkConfig->data.ssid, sizeof(ctx.networkConfig->data.ssid), ssid);
    copyBounded(ctx.networkConfig->data.password, sizeof(ctx.networkConfig->data.password), password ? password : "");
    ctx.networkConfig->data.credentialsValid = true;
    ctx.networkConfig->data.wifiEnabled = true;
    ctx.networkConfig->data.discoveryEnabled = true;
    ctx.networkConfig->sanitize();

    if (ctx.wifiManager) ctx.wifiManager->reset();
    applyWifiConfig(ctx);
    const bool saved = saveNetworkConfig(ctx);
    startSlimeRuntime(ctx);
    return saved;
}

int base64Value(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    if (c == '=') return -2;
    return -1;
}

bool decodeBase64(const char* in, char* out, size_t outCap) {
    if (!in || !out || outCap == 0) return false;
    size_t n = 0;
    int val = 0;
    int valb = -8;
    for (const char* p = in; *p; ++p) {
        const int d = base64Value(*p);
        if (d == -2) break;
        if (d < 0) return false;
        val = (val << 6) + d;
        valb += 6;
        if (valb >= 0) {
            if (n + 1 >= outCap) return false;
            out[n++] = static_cast<char>((val >> valb) & 0xFF);
            valb -= 8;
        }
    }
    out[n] = '\0';
    return true;
}

void dispatchSet(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = outFor(ctx);
    if (argc >= 2 && is(argv[1], "WIFI")) {
        if (argc < 4) {
            error(out, "CMD SET WIFI ERROR: Too few arguments");
            info(out, "Syntax: SET WIFI \"\" \"\"");
            return;
        }
        if (setWifiCredentials(ctx, argv[2], argv[3])) {
            info(out, "CMD SET WIFI OK: New wifi credentials set, reconnecting");
        } else {
            error(out, "CMD SET WIFI ERROR: failed to set or save credentials");
        }
        return;
    }

    if (argc >= 2 && is(argv[1], "BWIFI")) {
        if (argc < 3) {
            error(out, "CMD SET BWIFI ERROR: Too few arguments");
            info(out, "Syntax: SET BWIFI <B64SSID> <B64PASSWORD>");
            return;
        }
        char ssid[33] = {};
        char pass[65] = {};
        if (!decodeBase64(argv[2], ssid, sizeof(ssid)) ||
            (argc >= 4 && !decodeBase64(argv[3], pass, sizeof(pass)))) {
            error(out, "CMD SET BWIFI ERROR: base64 decode failed or value too long");
            return;
        }
        if (setWifiCredentials(ctx, ssid, argc >= 4 ? pass : "")) {
            info(out, "CMD SET BWIFI OK: New wifi credentials set, reconnecting");
        } else {
            error(out, "CMD SET BWIFI ERROR: failed to set or save credentials");
        }
        return;
    }

    error(out, "CMD SET ERROR: Unrecognized variable to set");
}

void dispatchGetWifiScan(TrackerSerialCommandContext& ctx) {
    Stream& out = outFor(ctx);
    if (!ctx.wifiManager) {
        error(out, "[WSCAN] WiFi manager is not available");
        return;
    }

    static WifiScanResult results[32];
    info(out, "[WSCAN] Scanning for WiFi networks...");
    const int16_t seen = ctx.wifiManager->scanNetworks(results, 32, false);
    trackerConsoleSuppressTrackingMessagesFor(TRACKER_SERIAL_COMMAND_RECOVERY_SUPPRESS_MS);

    if (seen < 0) {
        info(out, "[WSCAN] Scan failed!");
        ctx.wifiManager->clearScanResults();
        return;
    }

    out.print("[INFO ] [SerialCommands] [WSCAN] Found ");
    out.print(seen);
    out.println(" networks:");

    const uint8_t shown = static_cast<uint8_t>(seen < 32 ? seen : 32);
    for (uint8_t i = 0; i < shown; ++i) {
        const WifiScanResult& r = results[i];
        out.print("[INFO ] [SerialCommands] [WSCAN] ");
        out.print(static_cast<unsigned int>(i));
        out.print(":\t");
        const size_t ssidLen = std::strlen(r.ssid);
        if (ssidLen < 10) out.print('0');
        out.print(ssidLen);
        out.print("\t'");
        out.print(r.ssid);
        out.print("'\t(");
        out.print(r.rssiDbm);
        out.print(" dBm)\t");
        out.println(wifiAuthTypeName(r.authType));
    }
    ctx.wifiManager->clearScanResults();
}

void dispatchGet(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = outFor(ctx);
    if (argc < 2) return;

    if (is(argv[1], "INFO")) {
        printCompatInfo(ctx, true);
        return;
    }
    if (is(argv[1], "CONFIG")) {
        printCompatConfig(out);
        return;
    }
    if (is(argv[1], "TEST")) {
        printCompatTest(ctx);
        return;
    }
    if (is(argv[1], "WIFISCAN") || is(argv[1], "WIFI") || is(argv[1], "SCAN")) {
        dispatchGetWifiScan(ctx);
        return;
    }

    error(out, "CMD GET ERROR: Unrecognized variable to get");
}

void dispatchTemperatureCalibration(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = outFor(ctx);
    if (argc >= 2 && is(argv[1], "PRINT")) {
        char* cal[] = { const_cast<char*>("cal"), const_cast<char*>("temp"), const_cast<char*>("print") };
        trackerSerialDispatchCalibrationCommand(ctx, 3, cal);
        return;
    }
    if (argc >= 2 && is(argv[1], "RESET")) {
        if (!ctx.gyroTempComp) {
            error(out, "TCAL RESET failed: gyro temp comp not available");
            return;
        }

        // Match upstream TCAL RESET semantics: reset the current temperature
        // calibration state in RAM only. Do not touch ctx.config here; otherwise
        // a later unrelated config save could accidentally persist the reset.
        ctx.gyroTempComp->setSlopeRadSPerC(Vec3::zero());
        ctx.gyroTempComp->setQualityMetadata(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        info(out, "TCAL RESET OK");
        return;
    }
    if (argc >= 2 && is(argv[1], "DEBUG")) {
        // The upstream SlimeVR firmware exposes TCAL DEBUG as a more verbose
        // temperature-calibration dump. Our local temp-comp command already
        // prints the runtime snapshot, quality counters and current bias, so
        // route DEBUG to the same non-mutating path instead of inventing a
        // second partially-compatible format.
        char* cal[] = { const_cast<char*>("cal"), const_cast<char*>("temp"), const_cast<char*>("print") };
        trackerSerialDispatchCalibrationCommand(ctx, 3, cal);
        return;
    }
    if (argc >= 2 && is(argv[1], "SAVE")) {
        if (!ctx.configStore || !ctx.config) {
            error(out, "TCAL SAVE failed: config store not available");
            return;
        }

        // Keep TCAL semantics temperature-only. The previous compatibility
        // wrapper called trackerSerialCaptureRuntimeToConfig(), which also
        // captured gyro/accel/output runtime state and made a host TCAL SAVE
        // more invasive than the SlimeVR command it emulates.
        if (ctx.gyroTempComp) {
            ctx.config->captureFromGyroTempComp(*ctx.gyroTempComp);
        }
        ctx.config->sanitize();
        ctx.config->updateCrc();
        if (ctx.configStore->save(*ctx.config)) info(out, "TCAL SAVE OK");
        else error(out, "TCAL SAVE failed");
        return;
    }
    info(out, "Usage:");
    info(out, " TCAL PRINT: print current temperature calibration config");
    info(out, " TCAL DEBUG: print debug values for the current temperature calibration profile");
    info(out, " TCAL RESET: reset current temperature calibration in RAM");
    info(out, " TCAL SAVE: save current temperature calibration to persistent flash");
}

void eraseCalibration(TrackerSerialCommandContext& ctx) {
    Stream& out = outFor(ctx);
    if (ctx.config) {
        ctx.config->data.gyroCal.biasValid = false;
        ctx.config->data.accelCal.valid = false;
        ctx.config->data.gyroCal.tempCompValid = false;
        ctx.config->data.magCal.calibrationValid = false;
        ctx.config->data.magCal.axisAlignmentValid = false;
        ctx.config->updateCrc();
    }
    if (ctx.imuCal) {
        *ctx.imuCal = ImuCalibration{};
    }
    if (ctx.configStore && ctx.config) {
        if (!ctx.configStore->save(*ctx.config)) {
            error(out, "ERASE CALIBRATION failed while saving config");
            return;
        }
    }
    info(out, "ERASE CALIBRATION");
}

} // namespace

bool trackerSerialDispatchSlimeVRSerialCompatCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
#if !TRACKER_ENABLE_SLIMEVR_SERIAL_COMPAT
    (void)ctx;
    (void)argc;
    (void)argv;
    return false;
#else
    if (argc <= 0 || !argv || !argv[0]) return false;
    Stream& out = outFor(ctx);

    if (is(argv[0], "SET")) {
        dispatchSet(ctx, argc, argv);
        return true;
    }
    if (is(argv[0], "GET")) {
        dispatchGet(ctx, argc, argv);
        return true;
    }
    if (is(argv[0], "REBOOT")) {
        info(out, "REBOOT");
        out.flush();
        delay(50);
        ESP.restart();
        return true;
    }
    if (is(argv[0], "FRST")) {
        info(out, "FACTORY RESET");
        trackerSerialFactoryReset(ctx);
        if (ctx.networkConfig) ctx.networkConfig->resetDefaults();
        if (ctx.networkConfigStore) ctx.networkConfigStore->erase();
        if (ctx.networkConfigLoadedFromNvs) *ctx.networkConfigLoadedFromNvs = false;
        out.flush();
        delay(100);
        ESP.restart();
        return true;
    }
    if (is(argv[0], "DELCAL")) {
        eraseCalibration(ctx);
        return true;
    }
    if (is(argv[0], "TCAL")) {
        dispatchTemperatureCalibration(ctx, argc, argv);
        return true;
    }

    return false;
#endif
}

} // namespace tracker

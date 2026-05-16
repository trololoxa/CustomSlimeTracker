#include "serial/tracker_setup_commands.hpp"

#include <Arduino.h>
#include <cstdint>
#include <cstring>
#include <cmath>

#include "config/tracker_config.hpp"
#include "config/tracker_network_config.hpp"
#include "network/wifi_manager.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "sensor/accel_6pos_calibration.hpp"
#include "sensor/calibration.hpp"
#include "sensor/fifo_calibrations.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "serial/tracker_calibration_commands.hpp"
#include "serial/tracker_config_commands.hpp"
#include "serial/tracker_mag_commands.hpp"
#include "serial/tracker_network_commands.hpp"
#include "serial/tracker_slimevr_commands.hpp"
#include "serial/tracker_serial_print.hpp"

namespace tracker {
namespace {

Stream& out(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

bool is(const char* a, const char* b) {
    return tracker_serial_detail::eqIgnoreCase(a, b);
}

const char* yesNo(bool v) {
    return v ? "yes" : "no";
}

const char* readyWord(bool v) {
    return v ? "ready" : "missing";
}

void copySetupCString(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    if (!src) src = "";
    std::strncpy(dst, src, dstSize - 1);
    dst[dstSize - 1] = '\0';
}

TrackerWifiManagerConfig makeSetupWifiManagerConfig(const TrackerNetworkConfig& net) {
    TrackerWifiManagerConfig cfg;
    cfg.enabled = net.data.wifiEnabled;
    cfg.credentialsValid = net.data.credentialsValid;
    cfg.ssid = net.data.ssid;
    cfg.password = net.data.password;
    cfg.hostname = net.data.deviceName;
    cfg.connectTimeoutMs = 15000;
    cfg.reconnectBackoffMs = 5000;
    cfg.statusPollIntervalMs = 250;
    return cfg;
}

void applySetupWifiConfig(TrackerSerialCommandContext& ctx) {
    if (!ctx.networkConfig) return;
    ctx.networkConfig->sanitize();
    if (ctx.wifiManager) {
        ctx.wifiManager->configure(makeSetupWifiManagerConfig(*ctx.networkConfig));
    }
}

bool saveSetupNetworkConfig(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    if (!ctx.networkConfig || !ctx.networkConfigStore) {
        tracker_serial_detail::printErr(s, "network config store not available");
        return false;
    }
    ctx.networkConfig->sanitize();
    if (!ctx.networkConfigStore->save(*ctx.networkConfig)) {
        s.print("# ERR setup wifi save failed: ");
        s.println(ctx.networkConfigStore->lastErrorName());
        return false;
    }
    if (ctx.networkConfigLoadedFromNvs) *ctx.networkConfigLoadedFromNvs = true;
    tracker_serial_detail::printOk(s, "Wi-Fi config saved to NVS");
    return true;
}

struct SetupReadiness {
    bool configValid = false;
    bool configLoaded = false;
    bool gyroReady = false;
    bool accelReady = false;
    bool tempReady = false;
    bool tempEnabled = false;
    bool tempRangeValid = false;
    bool magDriver = false;
    bool magCal = false;
    bool magAxis = false;
    bool magYawApply = false;
    bool wifiConfigured = false;
    bool wifiEnabled = false;
    bool wifiConnected = false;
    bool slimeEnabled = false;
    bool slimeServerFound = false;
    bool localOutputReady = false;
    bool runtimeWired = false;

    bool tracking6dof() const { return configValid && gyroReady && accelReady; }
    bool tempQuality() const { return gyroReady && tempReady && tempRangeValid; }
    bool magYaw() const { return tracking6dof() && magDriver && magCal && magAxis; }
    bool network() const { return wifiConfigured && wifiEnabled; }
    bool slimevr() const { return tracking6dof() && network() && localOutputReady && runtimeWired; }
    bool production() const { return slimevr() && tempQuality() && magYaw(); }
};

SetupReadiness readSetupReadiness(TrackerSerialCommandContext& ctx) {
    SetupReadiness r;
    r.configValid = ctx.config && ctx.config->validate();
    r.configLoaded = r.configValid;
    r.gyroReady = ctx.imuCal && ctx.imuCal->gyroBiasValid;
    r.accelReady = ctx.imuCal && ctx.imuCal->accelCalValid;

    const float tempC = ctx.calibrationIo ? ctx.calibrationIo->latestTempC : 25.0f;
    if (ctx.gyroTempComp) {
        const GyroTempCompSnapshot s = ctx.gyroTempComp->snapshot(tempC);
        r.tempReady = s.valid && s.enabled && s.hasCalibratedRange && s.fitQuality > 0.0f;
        r.tempEnabled = s.enabled;
        r.tempRangeValid = s.hasCalibratedRange && !s.tempOutOfRange;
    } else if (ctx.config) {
        r.tempReady = ctx.config->data.gyroCal.tempCompValid &&
                      ctx.config->data.gyroTempQuality.fitQuality > 0.0f;
        r.tempEnabled = ctx.config->data.gyroCal.tempCompEnabled;
        r.tempRangeValid = ctx.config->data.gyroTempQuality.tempRangeMaxC >
                           ctx.config->data.gyroTempQuality.tempRangeMinC;
    }

    if (ctx.config) {
        r.magDriver = ctx.config->data.magCal.driverEnabled;
        r.magCal = ctx.config->data.magCal.calibrationValid;
        r.magAxis = ctx.config->data.magCal.axisAlignmentValid;
        r.magYawApply = ctx.config->data.magYaw.applyEnabled;
    }

    if (ctx.networkConfig) {
        const auto& n = ctx.networkConfig->data;
        r.wifiConfigured = n.credentialsValid && n.ssid[0] != '\0';
        r.wifiEnabled = n.wifiEnabled;
    }

    if (ctx.wifiManager) {
        const TrackerWifiManagerStatus st = ctx.wifiManager->status();
        r.wifiConnected = st.connected;
    }

    if (ctx.slimevrRuntime) {
        const SlimeVROutputRuntimeStatus st = ctx.slimevrRuntime->status();
        r.slimeEnabled = st.enabled;
        r.slimeServerFound = st.serverFound;
    }

    r.localOutputReady = ctx.config != nullptr && ctx.streamState != nullptr;
    r.runtimeWired = ctx.networkConfig != nullptr && ctx.wifiManager != nullptr && ctx.slimevrRuntime != nullptr;
    return r;
}

void printStep(Stream& s, const char* name, bool ready, const char* next) {
    s.print(name);
    s.print('=');
    s.println(readyWord(ready));
    if (!ready && next && next[0]) {
        s.print("  next: ");
        s.println(next);
    }
}

void printSetupGuide(Stream& s) {
    s.println("# SETUP GUIDE");
    s.println("# New tracker path:");
    s.println("#   1) setup wifi");
    s.println("#   2) setup calibration [axis <bodyX> <bodyY> <bodyZ>]");
    s.println("#   3) setup status");
    s.println();
    s.println("setup wifi");
    s.println("  Interactive Wi-Fi provisioning: scan, choose network, enter password,");
    s.println("  connect, save to NVS, start SlimeVR discovery and enable autostart.");
    s.println();
    s.println("setup calibration [axis <bodyX> <bodyY> <bodyZ>]");
    s.println("  Blocking guided production calibration. It services FIFO, magnetometer,");
    s.println("  Wi-Fi and SlimeVR while it performs rest gyro, gyro temperature model,");
    s.println("  accel 6-position, mag hard/soft collection, mag axis alignment and");
    s.println("  production tracking enable/save.");
    s.println();
    s.println("setup status");
    s.println("  Readiness checklist for tracking, mag-yaw, temperature model and SlimeVR.");
    s.println();
    s.println("# Low-level net/cal/mag commands remain available for service diagnostics,");
    s.println("# but normal first-run setup should use the two commands above.");
}

void printSetupStatus(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    const SetupReadiness r = readSetupReadiness(ctx);

    s.println("# SETUP STATUS");
    s.print("production_ready="); s.println(yesNo(r.production()));
    s.print("tracking_6dof_ready="); s.println(yesNo(r.tracking6dof()));
    s.print("mag_yaw_ready="); s.println(yesNo(r.magYaw()));
    s.print("temp_model_ready="); s.println(yesNo(r.tempQuality()));
    s.print("slimevr_ready="); s.println(yesNo(r.slimevr()));
    s.println();

    printStep(s, "config", r.configValid, "config print; config save");
    printStep(s, "wifi", r.network(), "setup wifi");
    s.print("wifi_connected="); s.println(yesNo(r.wifiConnected));
    printStep(s, "rest_gyro", r.gyroReady, "setup calibration");
    printStep(s, "accel_6pos", r.accelReady, "setup calibration");
    printStep(s, "mag_driver", r.magDriver, "setup calibration");
    printStep(s, "mag_hard_soft", r.magCal, "setup calibration");
    printStep(s, "mag_axis", r.magAxis, "setup calibration axis <bodyX> <bodyY> <bodyZ>");
    printStep(s, "temperature_model", r.tempQuality(), "setup calibration");
    printStep(s, "slimevr_runtime", r.slimevr(), "setup wifi; slime status");

    s.println();
    s.print("rest_calibration_sent_to_slimevr=");
    s.println(yesNo(r.gyroReady));
    s.print("slimevr_server_found="); s.println(yesNo(r.slimeServerFound));
    s.print("mag_yaw_apply_enabled="); s.println(yesNo(r.magYawApply));

    if (!r.production()) {
        s.println("# Use setup guide for the full first-run sequence.");
    }
}

void dispatchNetwork(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    trackerSerialDispatchNetworkCommand(ctx, argc, argv);
}

void dispatchCal(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    trackerSerialDispatchCalibrationCommand(ctx, argc, argv);
}

void dispatchMag(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    trackerSerialDispatchMagCommand(ctx, argc, argv);
}

bool readSetupLine(TrackerSerialCommandContext& ctx, const char* prompt, char* buf, size_t cap, uint32_t timeoutMs);
bool serviceSetupRuntime(TrackerSerialCommandContext& ctx);
void cmdSetupSave(TrackerSerialCommandContext& ctx);


void printSetupWifiList(Stream& s, const WifiScanResult* results, uint8_t count) {
    s.println("# idx rssi_dbm ch auth ssid");
    for (uint8_t i = 0; i < count; ++i) {
        const WifiScanResult& r = results[i];
        s.print(static_cast<unsigned int>(i + 1));
        s.print(' ');
        s.print(r.rssiDbm);
        s.print(' ');
        s.print(static_cast<unsigned int>(r.channel));
        s.print(' ');
        s.print(wifiAuthTypeName(r.authType));
        s.print(' ');
        s.println(r.ssid);
    }
}

bool setupWaitWifiConnected(TrackerSerialCommandContext& ctx, uint32_t timeoutMs) {
    Stream& s = out(ctx);
    const uint32_t startMs = millis();
    uint32_t lastPrintMs = 0;
    while (millis() - startMs < timeoutMs) {
        serviceSetupRuntime(ctx);
        if (ctx.wifiManager && ctx.wifiManager->connected()) return true;
        const uint32_t nowMs = millis();
        if (nowMs - lastPrintMs >= 1000UL) {
            lastPrintMs = nowMs;
            if (ctx.wifiManager) {
                const TrackerWifiManagerStatus st = ctx.wifiManager->status();
                s.print("# setup wifi connecting state=");
                s.print(trackerWifiStateName(st.state));
                s.print(" link=");
                s.print(wifiLinkStatusName(st.linkStatus));
                s.print(" elapsed_s=");
                s.println((nowMs - startMs) / 1000UL);
            }
        }
        delay(5);
    }
    return ctx.wifiManager && ctx.wifiManager->connected();
}

bool setupWaitSlimeServerFound(TrackerSerialCommandContext& ctx, uint32_t timeoutMs) {
    Stream& s = out(ctx);
    const uint32_t startMs = millis();
    uint32_t lastPrintMs = 0;
    while (millis() - startMs < timeoutMs) {
        serviceSetupRuntime(ctx);
        if (ctx.slimevrRuntime && ctx.slimevrRuntime->serverFound()) return true;
        const uint32_t nowMs = millis();
        if (nowMs - lastPrintMs >= 2000UL) {
            lastPrintMs = nowMs;
            if (ctx.slimevrRuntime) {
                const SlimeVROutputRuntimeStatus st = ctx.slimevrRuntime->status();
                s.print("# setup wifi slimevr state=");
                s.print(slimevrOutputStateName(st.state));
                s.print(" discovery_responses=");
                s.print(st.discoveryResponses);
                s.print(" elapsed_s=");
                s.println((nowMs - startMs) / 1000UL);
            }
        }
        delay(5);
    }
    return ctx.slimevrRuntime && ctx.slimevrRuntime->serverFound();
}

void setupStartSlimeRuntime(TrackerSerialCommandContext& ctx) {
    char* start[] = { const_cast<char*>("slime"), const_cast<char*>("start") };
    trackerSerialDispatchSlimeVRCommand(ctx, 2, start);
}

void cmdSetupWifi(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& s = out(ctx);
    if (argc >= 3 && (is(argv[2], "help") || is(argv[2], "?"))) {
        s.println("setup wifi");
        s.println("  Interactive Wi-Fi provisioning: scan, choose SSID, enter password,");
        s.println("  connect, save to NVS, start SlimeVR discovery and enable autostart.");
        return;
    }
    if (argc >= 3) {
        tracker_serial_detail::printErr(s, "usage: setup wifi  (interactive); use low-level net/slime commands for manual diagnostics");
        return;
    }

    if (!ctx.networkConfig || !ctx.networkConfigStore || !ctx.wifiManager) {
        tracker_serial_detail::printErr(s, "setup wifi failed: network runtime/config is not available");
        return;
    }

    static WifiScanResult results[16];
    s.println("# SETUP WIFI");
    s.println("# Scanning visible 2.4 GHz networks. Keep SlimeVR Server open on the same LAN.");
    const int16_t seen = ctx.wifiManager->scanNetworks(results, 16, false);
    if (seen < 0) {
        s.print("# ERR setup wifi scan failed: ");
        s.println(seen);
        ctx.wifiManager->clearScanResults();
        return;
    }

    const uint8_t shown = static_cast<uint8_t>(seen < 16 ? seen : 16);
    if (shown == 0) {
        ctx.wifiManager->clearScanResults();
        tracker_serial_detail::printErr(s, "no visible Wi-Fi networks found; use low-level net set commands for hidden SSIDs");
        return;
    }
    printSetupWifiList(s, results, shown);

    char line[96] = {};
    uint32_t choice = 0;
    while (choice == 0 || choice > shown) {
        if (!readSetupLine(ctx, "# Choose network number, or q to abort:", line, sizeof(line), 120000UL)) {
            ctx.wifiManager->clearScanResults();
            tracker_serial_detail::printErr(s, "setup wifi aborted: selection timeout");
            return;
        }
        if (is(line, "q") || is(line, "quit") || is(line, "abort")) {
            ctx.wifiManager->clearScanResults();
            tracker_serial_detail::printErr(s, "setup wifi aborted");
            return;
        }
        if (!tracker_serial_detail::parseU32(line, choice) || choice == 0 || choice > shown) {
            choice = 0;
            s.println("# Please enter a valid number from the list.");
        }
    }

    const WifiScanResult selected = results[choice - 1];
    ctx.wifiManager->clearScanResults();
    if (selected.ssid[0] == '\0') {
        tracker_serial_detail::printErr(s, "selected network has an empty/hidden SSID; use low-level net set commands");
        return;
    }

    s.print("# Selected SSID: ");
    s.println(selected.ssid);
    s.println("# Enter Wi-Fi password. Leave blank only for an open network.");
    char password[65] = {};
    if (!readSetupLine(ctx, "# password> ", password, sizeof(password), 180000UL)) {
        tracker_serial_detail::printErr(s, "setup wifi aborted: password timeout");
        return;
    }
    if (selected.authType != WifiAuthType::Open && password[0] == '\0') {
        tracker_serial_detail::printErr(s, "password is required for the selected secured network");
        return;
    }

    TrackerNetworkConfig oldConfig = *ctx.networkConfig;
    copySetupCString(ctx.networkConfig->data.ssid, sizeof(ctx.networkConfig->data.ssid), selected.ssid);
    copySetupCString(ctx.networkConfig->data.password, sizeof(ctx.networkConfig->data.password), password);
    ctx.networkConfig->data.credentialsValid = true;
    ctx.networkConfig->data.wifiEnabled = true;
    ctx.networkConfig->data.discoveryEnabled = true;
    ctx.networkConfig->sanitize();

    s.println("# Trying Wi-Fi connection. Credentials are not saved until connection succeeds.");
    ctx.wifiManager->reset();
    applySetupWifiConfig(ctx);
    if (!setupWaitWifiConnected(ctx, 30000UL)) {
        *ctx.networkConfig = oldConfig;
        ctx.networkConfig->sanitize();
        ctx.wifiManager->reset();
        applySetupWifiConfig(ctx);
        tracker_serial_detail::printErr(s, "setup wifi failed: could not connect; old network config restored in RAM and NVS was not changed");
        return;
    }

    tracker_serial_detail::printOk(s, "Wi-Fi connected");
    if (!saveSetupNetworkConfig(ctx)) {
        s.println("# WARN Wi-Fi works in RAM, but autostart was not persisted");
        return;
    }

    s.println("# Starting SlimeVR UDP discovery. Make sure SlimeVR Server is running on the same network.");
    setupStartSlimeRuntime(ctx);
    if (setupWaitSlimeServerFound(ctx, 30000UL)) {
        tracker_serial_detail::printOk(s, "SlimeVR Server found; Wi-Fi/SlimeVR autostart is enabled");
    } else {
        s.println("# WARN SlimeVR Server was not found within 30s.");
        s.println("# Wi-Fi was saved and autostart remains enabled; discovery will continue during normal runtime.");
        s.println("# Check that SlimeVR Server is running, firewall allows UDP 6969, and PC/tracker are on the same LAN.");
    }

    printSetupStatus(ctx);
}


void cmdSetupSave(TrackerSerialCommandContext& ctx);

bool serviceSetupRuntime(TrackerSerialCommandContext& ctx) {
    if (ctx.serviceCalibrationRuntime) {
        return ctx.serviceCalibrationRuntime(ctx.serviceCalibrationRuntimeUser);
    }
    delay(5);
    return true;
}

void drainSetupInput(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    while (s.available() > 0) {
        (void)s.read();
        serviceSetupRuntime(ctx);
    }
}

bool readSetupLine(TrackerSerialCommandContext& ctx,
                   const char* prompt,
                   char* buf,
                   size_t cap,
                   uint32_t timeoutMs) {
    Stream& s = out(ctx);
    if (!buf || cap == 0) return false;
    buf[0] = '\0';

    drainSetupInput(ctx);
    if (prompt && prompt[0]) s.println(prompt);

    const uint32_t startMs = millis();
    size_t n = 0;
    while (millis() - startMs < timeoutMs) {
        serviceSetupRuntime(ctx);
        while (s.available() > 0) {
            const int c = s.read();
            if (c < 0) break;
            if (c == '\r') continue;
            if (c == '\n') {
                buf[n] = '\0';
                return true;
            }
            if (n + 1 < cap) {
                buf[n++] = static_cast<char>(c);
            }
        }
        delay(5);
    }

    buf[n] = '\0';
    return false;
}

bool waitSetupEnter(TrackerSerialCommandContext& ctx, const char* prompt, uint32_t timeoutMs) {
    char line[8];
    return readSetupLine(ctx, prompt, line, sizeof(line), timeoutMs);
}

bool waitSetupEnterOrTimeout(TrackerSerialCommandContext& ctx,
                             const char* prompt,
                             uint32_t durationMs) {
    Stream& s = out(ctx);
    drainSetupInput(ctx);
    if (prompt && prompt[0]) s.println(prompt);

    const uint32_t startMs = millis();
    uint32_t lastPrintMs = 0;
    while (millis() - startMs < durationMs) {
        serviceSetupRuntime(ctx);
        while (s.available() > 0) {
            const int c = s.read();
            if (c == '\n' || c == '\r') return true;
        }
        const uint32_t nowMs = millis();
        if (nowMs - lastPrintMs >= 5000UL) {
            lastPrintMs = nowMs;
            s.print("# setup calibration move_elapsed_s=");
            s.print((nowMs - startMs) / 1000UL);
            s.print(" target_s=");
            s.println(durationMs / 1000UL);
        }
        delay(5);
    }
    return true;
}

bool setupMaybeStartWifiHeating(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    if (!ctx.networkConfig) {
        s.println("# WARN Wi-Fi config is not wired; temperature calibration will use IMU/self heating only");
        return true;
    }

    const auto& n = ctx.networkConfig->data;
    if (!n.credentialsValid || n.ssid[0] == '\0') {
        s.println("# WARN Wi-Fi credentials are missing; run setup wifi before production calibration");
        s.println("# WARN continuing without Wi-Fi heat source; temp fit may fail if temperature range is too small");
        return true;
    }

    s.println("# setup calibration: enabling Wi-Fi during static warm-up for realistic tracker heating");
    char* enable[] = { const_cast<char*>("net"), const_cast<char*>("enable"), const_cast<char*>("save") };
    dispatchNetwork(ctx, 3, enable);
    char* reconnect[] = { const_cast<char*>("net"), const_cast<char*>("reconnect") };
    dispatchNetwork(ctx, 2, reconnect);
    return true;
}

bool setupRunRestGyro(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    s.println();
    s.println("# SETUP CALIBRATION STEP 1/6: REST/GYRO");
    s.println("# Put the tracker on a stable surface and do not touch it.");
    if (!waitSetupEnter(ctx, "# Press Enter when the tracker is completely still.", 120000UL)) {
        tracker_serial_detail::printErr(s, "setup calibration aborted: rest confirmation timeout");
        return false;
    }

    char* gyro[] = { const_cast<char*>("cal"), const_cast<char*>("gyro") };
    dispatchCal(ctx, 2, gyro);
    if (!ctx.imuCal || !ctx.imuCal->gyroBiasValid) {
        tracker_serial_detail::printErr(s, "setup calibration failed: gyro/rest calibration is not valid");
        return false;
    }
    char* saveGyro[] = { const_cast<char*>("cal"), const_cast<char*>("gyro"), const_cast<char*>("save") };
    dispatchCal(ctx, 3, saveGyro);
    if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();
    return true;
}

bool setupRunTemperatureFit(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    s.println();
    s.println("# SETUP CALIBRATION STEP 2/6: GYRO TEMPERATURE MODEL");
    s.println("# Keep the tracker still. The firmware will stop when temperature reaches a relative plateau.");

    if (!ctx.startStaticTest || !ctx.stopStaticTest || !ctx.fitGyroTempFromLastStatic) {
        tracker_serial_detail::printErr(s, "setup calibration failed: static/temp hooks are not available");
        return false;
    }

    setupMaybeStartWifiHeating(ctx);

    constexpr uint32_t kMinMs = 180000UL;
    constexpr uint32_t kMaxMs = 900000UL;
    constexpr uint32_t kPlateauWindowMs = 60000UL;
    constexpr float kPlateauDeltaC = 0.15f;
    constexpr float kMinTempRangeC = 3.0f;

    const float startTemp = ctx.calibrationIo ? ctx.calibrationIo->latestTempC : 25.0f;
    float minTemp = startTemp;
    float maxTemp = startTemp;
    float windowStartTemp = startTemp;
    uint32_t windowStartMs = millis();
    uint32_t lastPrintMs = 0;

    if (!ctx.startStaticTest(kMaxMs, ctx.startStaticTestUser)) {
        tracker_serial_detail::printErr(s, "setup calibration failed: could not start static temperature capture");
        return false;
    }

    const uint32_t startMs = millis();
    bool plateau = false;
    while (millis() - startMs < kMaxMs) {
        serviceSetupRuntime(ctx);
        const uint32_t nowMs = millis();
        const float t = ctx.calibrationIo ? ctx.calibrationIo->latestTempC : startTemp;
        if (t < minTemp) minTemp = t;
        if (t > maxTemp) maxTemp = t;

        if (nowMs - lastPrintMs >= 5000UL) {
            lastPrintMs = nowMs;
            s.print("# setup temp elapsed_s="); s.print((nowMs - startMs) / 1000UL);
            s.print(" temp_c="); s.print(t, 3);
            s.print(" range_c="); s.print(maxTemp - minTemp, 3);
            s.print(" plateau_window_delta_c="); s.println(std::fabs(t - windowStartTemp), 3);
        }

        if (nowMs - windowStartMs >= kPlateauWindowMs) {
            const float windowDelta = std::fabs(t - windowStartTemp);
            const float totalRange = maxTemp - minTemp;
            if ((nowMs - startMs) >= kMinMs && totalRange >= kMinTempRangeC && windowDelta <= kPlateauDeltaC) {
                plateau = true;
                break;
            }
            windowStartMs = nowMs;
            windowStartTemp = t;
        }
        delay(5);
    }

    if (plateau) s.println("# setup temp: relative plateau detected; stopping static capture");
    else s.println("# setup temp: max capture duration reached; trying fit with collected data");

    ctx.stopStaticTest(ctx.stopStaticTestUser);
    const uint32_t finishStartMs = millis();
    while (millis() - finishStartMs < 5000UL) {
        serviceSetupRuntime(ctx);
        delay(5);
    }

    if (!ctx.fitGyroTempFromLastStatic(true, s, ctx.fitGyroTempFromLastStaticUser)) {
        tracker_serial_detail::printErr(s, "setup calibration failed: gyro temperature fit did not pass quality gates");
        s.println("# TIP: repeat setup calibration after a larger cold-to-warm temperature change");
        return false;
    }
    return true;
}

bool setupRunAccelFacesWithMagCollection(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    s.println();
    s.println("# SETUP CALIBRATION STEP 3/6: ACCEL 6-POS + MAG COLLECTION");

    if (ctx.setMagRuntimeEnabled) {
        (void)ctx.setMagRuntimeEnabled(true, false, ctx.setMagRuntimeEnabledUser);
    }
    if (ctx.config) {
        ctx.config->data.magCal.driverEnabled = true;
        ctx.config->updateCrc();
    }
    if (ctx.resetMagCalibration) ctx.resetMagCalibration(ctx.resetMagCalibrationUser);
    if (!ctx.startMagCalibration || !ctx.startMagCalibration(ctx.startMagCalibrationUser)) {
        tracker_serial_detail::printErr(s, "setup calibration failed: could not start magnetometer calibration");
        return false;
    }

    char* clearAccel[] = { const_cast<char*>("cal"), const_cast<char*>("accel"), const_cast<char*>("clear") };
    dispatchCal(ctx, 3, clearAccel);

    struct FaceStep { Accel6PosCalibration::Face face; const char* token; const char* prompt; };
    const FaceStep faces[] = {
        { Accel6PosCalibration::Face::XP, "XP", "# Place tracker with +X up, keep still, then press Enter." },
        { Accel6PosCalibration::Face::XN, "XN", "# Place tracker with -X up, keep still, then press Enter." },
        { Accel6PosCalibration::Face::YP, "YP", "# Place tracker with +Y up, keep still, then press Enter." },
        { Accel6PosCalibration::Face::YN, "YN", "# Place tracker with -Y up, keep still, then press Enter." },
        { Accel6PosCalibration::Face::ZP, "ZP", "# Place tracker with +Z up, keep still, then press Enter." },
        { Accel6PosCalibration::Face::ZN, "ZN", "# Place tracker with -Z up, keep still, then press Enter." },
    };

    for (const FaceStep& f : faces) {
        if (!waitSetupEnter(ctx, f.prompt, 300000UL)) {
            if (ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
            tracker_serial_detail::printErr(s, "setup calibration aborted while waiting for accel face");
            return false;
        }
        char* faceCmd[] = { const_cast<char*>("cal"), const_cast<char*>("accel"), const_cast<char*>("face"), const_cast<char*>(f.token) };
        dispatchCal(ctx, 4, faceCmd);
        if (!ctx.accelCalRunner || !ctx.accelCalRunner->calibration().hasFace(f.face)) {
            if (ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
            tracker_serial_detail::printErr(s, "setup calibration failed: accel face was not captured");
            return false;
        }
    }

    char* compute[] = { const_cast<char*>("cal"), const_cast<char*>("accel"), const_cast<char*>("compute") };
    dispatchCal(ctx, 3, compute);
    if (!ctx.imuCal || !ctx.imuCal->accelCalValid) {
        if (ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
        tracker_serial_detail::printErr(s, "setup calibration failed: accel 6-position quality gates rejected the result");
        return false;
    }
    char* saveAccel[] = { const_cast<char*>("cal"), const_cast<char*>("accel"), const_cast<char*>("save") };
    dispatchCal(ctx, 3, saveAccel);

    return true;
}

bool setupRunMagMotionAndApply(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    s.println();
    s.println("# SETUP CALIBRATION STEP 4/6: MAG HARD/SOFT MOTION COVERAGE");
    s.println("# Move slowly through as many orientations as possible. Avoid steel tables, speakers, chargers and magnets.");
    waitSetupEnterOrTimeout(ctx, "# Press Enter after good all-axis coverage, or let the timed collection finish.", 90000UL);

    if (ctx.stopMagCalibration) ctx.stopMagCalibration(ctx.stopMagCalibrationUser);
    if (!ctx.applyMagCalibration || !ctx.applyMagCalibration(true, ctx.applyMagCalibrationUser)) {
        tracker_serial_detail::printErr(s, "setup calibration failed: mag hard/soft calibration did not pass quality gates");
        return false;
    }
    return true;
}

bool setupSetAxisIdentity(TrackerSerialCommandContext& ctx) {
    char* identity[] = { const_cast<char*>("mag"), const_cast<char*>("axis"), const_cast<char*>("identity"), const_cast<char*>("save") };
    dispatchMag(ctx, 4, identity);
    return ctx.config && ctx.config->data.magCal.axisAlignmentValid;
}

bool setupSetAxisMapping(TrackerSerialCommandContext& ctx, const char* x, const char* y, const char* z) {
    char* setAxis[] = {
        const_cast<char*>("mag"), const_cast<char*>("axis"), const_cast<char*>("set"),
        const_cast<char*>(x), const_cast<char*>(y), const_cast<char*>(z), const_cast<char*>("save")
    };
    dispatchMag(ctx, 7, setAxis);
    return ctx.config && ctx.config->data.magCal.axisAlignmentValid;
}

bool setupRunAxisAlignment(TrackerSerialCommandContext& ctx,
                           const char* axisX,
                           const char* axisY,
                           const char* axisZ) {
    Stream& s = out(ctx);
    s.println();
    s.println("# SETUP CALIBRATION STEP 5/6: MAG AXIS ALIGNMENT");

    char line[48] = {};
    const char* x = axisX;
    const char* y = axisY;
    const char* z = axisZ;

    if (!x || !y || !z) {
        if (ctx.config && ctx.config->data.magCal.axisAlignmentValid) {
            s.println("# Existing mag axis alignment is valid; keeping it.");
            return true;
        }

        s.println("# Enter mag axis mapping as three tokens for IMU/body X Y Z, for example: +x +y +z");
        s.println("# Leave blank only if this board's magnetometer axes are already known to match IMU axes.");
        if (!readSetupLine(ctx, "# axis mapping> ", line, sizeof(line), 300000UL)) {
            tracker_serial_detail::printErr(s, "setup calibration aborted: axis mapping timeout");
            return false;
        }
        if (line[0] == '\0') {
            return setupSetAxisIdentity(ctx);
        }

        char* tokens[3] = {};
        uint8_t count = 0;
        char* save = nullptr;
        for (char* p = strtok(line, " \t"); p != nullptr && count < 3; p = strtok(nullptr, " \t")) {
            tokens[count++] = p;
        }
        if (count != 3) {
            tracker_serial_detail::printErr(s, "setup calibration failed: expected three axis tokens, for example +x +y +z");
            return false;
        }
        x = tokens[0]; y = tokens[1]; z = tokens[2];
        (void)save;
    }

    return setupSetAxisMapping(ctx, x, y, z);
}

bool setupEnableProductionTracking(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    s.println();
    s.println("# SETUP CALIBRATION STEP 6/6: ENABLE PRODUCTION TRACKING FEATURES");

    if (!ctx.config) {
        tracker_serial_detail::printErr(s, "setup calibration failed: config is not available");
        return false;
    }

    ctx.config->data.ahrs.useAccelCorrection = true;
    ctx.config->data.ahrsRuntime.accelCorrectionEnabled = true;
    ctx.config->data.ahrsRuntime.adaptiveAccelCorrection = true;

    if (ctx.gyroTempComp) ctx.gyroTempComp->setEnabled(true);
    if (ctx.setRuntimeGyroBiasEnabled) {
        (void)ctx.setRuntimeGyroBiasEnabled(true, ctx.setRuntimeGyroBiasEnabledUser);
    }
    if (ctx.setMagYawCorrectionApplyEnabled) {
        if (!ctx.setMagYawCorrectionApplyEnabled(true, true, ctx.setMagYawCorrectionApplyEnabledUser)) {
            tracker_serial_detail::printErr(s, "setup calibration failed: mag yaw apply enable failed");
            return false;
        }
    } else {
        ctx.config->data.magYaw.applyEnabled = true;
    }

    if (ctx.resetAhrsRuntime) ctx.resetAhrsRuntime(ctx.resetAhrsRuntimeUser);
    cmdSetupSave(ctx);

    const SetupReadiness r = readSetupReadiness(ctx);
    if (!r.production()) {
        s.println("# WARN setup calibration finished, but production_ready is still no; run setup status for missing items");
    } else {
        tracker_serial_detail::printOk(s, "setup calibration complete: tracker is production-ready");
    }
    if (ctx.slimevrRuntime) ctx.slimevrRuntime->requestSensorInfoRefresh();
    printSetupStatus(ctx);
    return r.tracking6dof() && r.magYaw() && r.tempQuality();
}

void cmdSetupCalibration(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& s = out(ctx);

    const char* axisX = nullptr;
    const char* axisY = nullptr;
    const char* axisZ = nullptr;
    for (int i = 2; i < argc; ++i) {
        if (is(argv[i], "axis")) {
            if (i + 3 >= argc) {
                tracker_serial_detail::printErr(s, "usage: setup calibration [axis <bodyX> <bodyY> <bodyZ>]");
                return;
            }
            axisX = argv[i + 1];
            axisY = argv[i + 2];
            axisZ = argv[i + 3];
            i += 3;
        }
    }

    s.println("# SETUP CALIBRATION");
    s.println("# This is a blocking guided production calibration flow.");
    s.println("# It services FIFO, magnetometer runtime, Wi-Fi and SlimeVR while waiting.");
    s.println("# Stages: rest gyro -> temperature model -> accel 6-position -> mag hard/soft -> mag axis -> enable tracking features.");

    if (!ctx.calibrationIo || !ctx.imuCal || !ctx.accelCalRunner || !ctx.config || !ctx.configStore) {
        tracker_serial_detail::printErr(s, "setup calibration failed: required calibration dependencies are not available");
        return;
    }

    if (!setupRunRestGyro(ctx)) return;
    if (!setupRunTemperatureFit(ctx)) return;
    if (!setupRunAccelFacesWithMagCollection(ctx)) return;
    if (!setupRunMagMotionAndApply(ctx)) return;
    if (!setupRunAxisAlignment(ctx, axisX, axisY, axisZ)) return;
    if (!setupEnableProductionTracking(ctx)) return;
}

void cmdSetupSave(TrackerSerialCommandContext& ctx) {
    Stream& s = out(ctx);
    if (!ctx.config || !ctx.configStore) {
        tracker_serial_detail::printErr(s, "config store not available");
        return;
    }
    trackerSerialCaptureRuntimeToConfig(ctx);
    ctx.config->sanitize();
    ctx.config->updateCrc();
    if (ctx.configStore->save(*ctx.config)) {
        tracker_serial_detail::printOk(s, "setup calibration/config saved");
    } else {
        s.print("# ERR setup save failed: ");
        s.println(ctx.configStore->lastErrorName());
    }
}

} // namespace

void trackerSerialDispatchSetupCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& s = out(ctx);

    if (argc < 2 || is(argv[1], "guide") || is(argv[1], "help") || is(argv[1], "?")) {
        printSetupGuide(s);
        return;
    }

    if (is(argv[1], "status")) {
        printSetupStatus(ctx);
        return;
    }

    if (is(argv[1], "calibration") || is(argv[1], "calibrate")) {
        cmdSetupCalibration(ctx, argc, argv);
        return;
    }

    if (is(argv[1], "wifi")) {
        cmdSetupWifi(ctx, argc, argv);
        return;
    }

    tracker_serial_detail::printErr(s, "usage: setup guide|status|wifi|calibration [axis <bodyX> <bodyY> <bodyZ>]");
}

} // namespace tracker

#include "serial/tracker_network_commands.hpp"

#include <cstring>

#include "config/tracker_network_config.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "network/wifi_manager.hpp"
#include "runtime/tracker_console_suppress.hpp"
#include "sensor/imu_quality.hpp"
#include "defines.h"

namespace tracker {

namespace {

Stream& netStream(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

bool is(const char* a, const char* b) {
    return tracker_serial_detail::eqIgnoreCase(a, b);
}

constexpr uint8_t WIFI_SCAN_DEFAULT_LIMIT = 32;
constexpr uint8_t WIFI_SCAN_MAX_LIMIT = 64;

void recoverSensorStreamAfterBlockingWifiScan(TrackerSerialCommandContext& ctx, const char* reason) {
    trackerConsoleSuppressTrackingMessagesFor(TRACKER_SERIAL_COMMAND_RECOVERY_SUPPRESS_MS);

    if (!ctx.fifo) return;

    const uint64_t keepTs = ctx.fifo->stats().lastAssignedTimestampUs;
    const bool ok = ctx.fifo->resetFifo();
    ctx.fifo->resetTimestampReconstruction(keepTs);

    if (ctx.quality) {
        ctx.quality->resetStreamRecoveryState();
        ctx.quality->syncFifoStats(ctx.fifo->stats());
    }
    if (ctx.resetFifoRuntime) {
        ctx.resetFifoRuntime(ctx.resetFifoRuntimeUser);
    }
    if (ctx.requestTrackingRecovery) {
        ctx.requestTrackingRecovery(
            imu_quality_flags::FIFO_RECOVERY_REQUESTED,
            "blocking_wifi_scan",
            keepTs,
            ctx.requestTrackingRecoveryUser
        );
    }

    if (!ok) {
        Stream& out = netStream(ctx);
        out.print("# WARN post-scan FIFO reset failed");
        if (reason && reason[0] != '\0') {
            out.print(" reason=");
            out.print(reason);
        }
        out.println();
    }
}

bool hasSaveArg(int argc, char** argv, int startIndex) {
    for (int i = startIndex; i < argc; ++i) {
        if (argv[i] && is(argv[i], "save")) return true;
    }
    return false;
}

void copyBounded(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    std::strncpy(dst, src, dstSize - 1);
    dst[dstSize - 1] = '\0';
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

void printHexByte(Stream& out, uint8_t v) {
    if (v < 0x10u) out.print('0');
    out.print(static_cast<unsigned int>(v), HEX);
}

void printMac(Stream& out, const uint8_t mac[6]) {
    for (int i = 0; i < 6; ++i) {
        if (i) out.print(':');
        printHexByte(out, mac[i]);
    }
}

TrackerWifiManagerConfig makeWifiConfig(const TrackerNetworkConfig& net) {
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

void applyWifiConfig(TrackerSerialCommandContext& ctx) {
    if (ctx.networkConfig) ctx.networkConfig->sanitize();
    if (ctx.wifiManager && ctx.networkConfig) {
        ctx.wifiManager->configure(makeWifiConfig(*ctx.networkConfig));
    }
}

bool saveIfRequested(TrackerSerialCommandContext& ctx, bool save) {
    Stream& out = netStream(ctx);
    if (!save) return true;
    if (!ctx.networkConfig || !ctx.networkConfigStore) {
        tracker_serial_detail::printErr(out, "network config store not available");
        return false;
    }
    ctx.networkConfig->sanitize();
    if (!ctx.networkConfigStore->save(*ctx.networkConfig)) {
        out.print("# ERR net save failed: ");
        out.println(ctx.networkConfigStore->lastErrorName());
        return false;
    }
    if (ctx.networkConfigLoadedFromNvs) *ctx.networkConfigLoadedFromNvs = true;
    tracker_serial_detail::printOk(out, "network config saved");
    return true;
}

void printNetworkHelp(Stream& out) {
    out.println("net status");
    out.println("net print");
    out.println("net scan [visible|hidden] [limit <n>]");
    out.println("net enable [save]");
    out.println("net disable [save]");
    out.println("net set ssid <ssid> [save]");
    out.println("net set pass <password> [save]");
    out.println("net clear pass [save]");
    out.println("net set name <deviceName> [save]");
    out.println("net set server <host> [port] [save]");
    out.println("net discovery on|off [save]");
    out.println("net save | load | defaults | erase | reconnect | counters reset");
}

void printQuotedSsid(Stream& out, const char* ssid) {
    out.print('"');
    if (ssid) {
        for (const char* p = ssid; *p; ++p) {
            if (*p == '"' || *p == '\\') out.print('\\');
            out.print(*p);
        }
    }
    out.print('"');
}


bool parseScanArgs(Stream& out, int argc, char** argv, bool& showHidden, uint8_t& limit) {
    showHidden = true;
    limit = WIFI_SCAN_DEFAULT_LIMIT;

    for (int i = 2; i < argc; ++i) {
        if (!argv[i]) continue;

        if (is(argv[i], "hidden") || is(argv[i], "all")) {
            showHidden = true;
            continue;
        }
        if (is(argv[i], "visible")) {
            showHidden = false;
            continue;
        }
        if (is(argv[i], "limit")) {
            if (i + 1 >= argc) {
                tracker_serial_detail::printErr(out, "usage: net scan [visible|hidden] [limit <n>]");
                return false;
            }
            ++i;
        }

        uint32_t parsed = 0;
        if (!tracker_serial_detail::parseU32(argv[i], parsed) || parsed == 0 || parsed > WIFI_SCAN_MAX_LIMIT) {
            tracker_serial_detail::printErr(out, "scan limit must be 1..64");
            return false;
        }
        limit = static_cast<uint8_t>(parsed);
    }

    return true;
}

void runWifiScan(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = netStream(ctx);
    if (!ctx.wifiManager) {
        tracker_serial_detail::printErr(out, "wifi manager not available");
        return;
    }

    bool showHidden = true;
    uint8_t limit = WIFI_SCAN_DEFAULT_LIMIT;
    if (!parseScanArgs(out, argc, argv, showHidden, limit)) return;

    static WifiScanResult results[WIFI_SCAN_MAX_LIMIT];

    out.println("# NET SCAN");
    out.println("# blocking diagnostic scan; sensor processing pauses until it finishes");
    out.print("show_hidden="); out.println(showHidden ? "yes" : "no");
    out.print("limit="); out.println(limit);

    const int16_t count = ctx.wifiManager->scanNetworks(results, limit, showHidden);
    recoverSensorStreamAfterBlockingWifiScan(ctx, "net_scan");
    if (count < 0) {
        out.print("# ERR wifi scan failed: ");
        out.println(count);
        ctx.wifiManager->clearScanResults();
        return;
    }

    const uint8_t shown = static_cast<uint8_t>(count < limit ? count : limit);
    out.print("networks_seen="); out.println(count);
    out.print("networks_printed="); out.println(shown);
    out.println("# idx rssi_dbm ch auth hidden bssid ssid");

    for (uint8_t i = 0; i < shown; ++i) {
        const WifiScanResult& r = results[i];
        out.print(i);
        out.print(' ');
        out.print(r.rssiDbm);
        out.print(' ');
        out.print(static_cast<unsigned int>(r.channel));
        out.print(' ');
        out.print(wifiAuthTypeName(r.authType));
        out.print(' ');
        out.print(r.hidden ? "yes" : "no");
        out.print(' ');
        out.print(r.bssid[0] ? r.bssid : "<unknown>");
        out.print(' ');
        printQuotedSsid(out, r.hidden ? "<hidden>" : r.ssid);
        out.println();
    }

    ctx.wifiManager->clearScanResults();
    tracker_serial_detail::printOk(out, "wifi scan done");
}

} // namespace

void trackerSerialPrintNetworkStatus(TrackerSerialCommandContext& ctx) {
    Stream& out = netStream(ctx);
    out.println("# NET STATUS");

    if (ctx.networkConfig) {
        const auto& n = ctx.networkConfig->data;
        out.print("config_loaded_from_nvs="); out.println(ctx.networkConfigLoadedFromNvs && *ctx.networkConfigLoadedFromNvs ? "yes" : "no");
        out.print("config_valid="); out.println(ctx.networkConfig->validate() ? "yes" : "no");
        out.print("wifi_enabled="); out.println(n.wifiEnabled ? "yes" : "no");
        out.print("credentials_valid="); out.println(n.credentialsValid ? "yes" : "no");
        out.print("ssid="); out.println(n.ssid);
        out.print("password_set="); out.println(n.password[0] != '\0' ? "yes" : "no");
        out.print("device_name="); out.println(n.deviceName);
        out.print("discovery_enabled="); out.println(n.discoveryEnabled ? "yes" : "no");
        out.print("manual_server_enabled="); out.println(n.manualServerEnabled ? "yes" : "no");
        out.print("server_host="); out.println(n.serverHost);
        out.print("server_port="); out.println(n.serverPort);
        out.print("sensor_id="); out.println(n.sensorId);
    } else {
        out.println("config_available=no");
    }

    if (!ctx.wifiManager) {
        out.println("wifi_manager_available=no");
        return;
    }

    const TrackerWifiManagerStatus st = ctx.wifiManager->status();
    out.print("wifi_state="); out.println(trackerWifiStateName(st.state));
    out.print("link_status="); out.println(wifiLinkStatusName(st.linkStatus));
    out.print("connected="); out.println(st.connected ? "yes" : "no");
    out.print("attempts="); out.println(st.attempts);
    out.print("connect_timeouts="); out.println(st.connectTimeouts);
    out.print("disconnects="); out.println(st.disconnects);
    out.print("connected_since_ms="); out.println(st.connectedSinceMs);
    out.print("last_transition_ms="); out.println(st.lastTransitionMs);
    out.print("next_retry_ms="); out.println(st.nextRetryMs);
    out.print("ip="); printIp(out, st.ipv4); out.println();
    out.print("rssi_dbm="); out.println(st.rssiDbm);
    out.print("mac="); printMac(out, st.mac); out.println();
}

void trackerSerialPrintNetworkConfig(TrackerSerialCommandContext& ctx) {
    Stream& out = netStream(ctx);
    out.println("# NET CONFIG");
    if (!ctx.networkConfig) {
        tracker_serial_detail::printErr(out, "network config not available");
        return;
    }

    const auto& n = ctx.networkConfig->data;
    out.print("wifiEnabled="); out.println(n.wifiEnabled ? "true" : "false");
    out.print("credentialsValid="); out.println(n.credentialsValid ? "true" : "false");
    out.print("ssid="); out.println(n.ssid);
    out.print("password=<"); out.print(n.password[0] != '\0' ? "set" : "empty"); out.println(">");
    out.print("deviceName="); out.println(n.deviceName);
    out.print("discoveryEnabled="); out.println(n.discoveryEnabled ? "true" : "false");
    out.print("manualServerEnabled="); out.println(n.manualServerEnabled ? "true" : "false");
    out.print("serverHost="); out.println(n.serverHost);
    out.print("serverPort="); out.println(n.serverPort);
    out.print("deviceId="); out.println(n.deviceId);
    out.print("sensorId="); out.println(n.sensorId);
    out.print("crc=0x"); out.println(n.crc32, HEX);
}

bool trackerSerialDispatchNetworkCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    if (argc <= 0 || !argv || !argv[0]) return true;
    if (!is(argv[0], "net")) return false;

    Stream& out = netStream(ctx);

    if (argc == 1 || (argc >= 2 && (is(argv[1], "help") || is(argv[1], "?")))) {
        printNetworkHelp(out);
        return true;
    }

    if (!ctx.networkConfig) {
        tracker_serial_detail::printErr(out, "network config not available");
        return true;
    }

    TrackerNetworkConfig& net = *ctx.networkConfig;

    if (is(argv[1], "status")) {
        trackerSerialPrintNetworkStatus(ctx);
        return true;
    }

    if (is(argv[1], "print")) {
        trackerSerialPrintNetworkConfig(ctx);
        return true;
    }

    if (is(argv[1], "scan")) {
        runWifiScan(ctx, argc, argv);
        return true;
    }


    if (is(argv[1], "enable")) {
        if (!net.data.credentialsValid || net.data.ssid[0] == '\0') {
            tracker_serial_detail::printErr(out, "set ssid first: net set ssid <ssid> [save]");
            return true;
        }
        net.data.wifiEnabled = true;
        applyWifiConfig(ctx);
        tracker_serial_detail::printOk(out, "wifi enabled; connection runs in background");
        saveIfRequested(ctx, hasSaveArg(argc, argv, 2));
        return true;
    }

    if (is(argv[1], "disable")) {
        net.data.wifiEnabled = false;
        applyWifiConfig(ctx);
        tracker_serial_detail::printOk(out, "wifi disabled");
        saveIfRequested(ctx, hasSaveArg(argc, argv, 2));
        return true;
    }

    if (is(argv[1], "set")) {
        if (argc < 4) {
            tracker_serial_detail::printErr(out, "usage: net set ssid|pass|name|server <value> [save]");
            return true;
        }

        if (is(argv[2], "ssid")) {
            copyBounded(net.data.ssid, sizeof(net.data.ssid), argv[3]);
            net.data.credentialsValid = net.data.ssid[0] != '\0';
            applyWifiConfig(ctx);
            tracker_serial_detail::printOk(out, "ssid updated");
            saveIfRequested(ctx, hasSaveArg(argc, argv, 4));
            return true;
        }

        if (is(argv[2], "pass") || is(argv[2], "password")) {
            copyBounded(net.data.password, sizeof(net.data.password), argv[3]);
            net.data.credentialsValid = net.data.ssid[0] != '\0';
            applyWifiConfig(ctx);
            tracker_serial_detail::printOk(out, "password updated");
            saveIfRequested(ctx, hasSaveArg(argc, argv, 4));
            return true;
        }

        if (is(argv[2], "name") || is(argv[2], "hostname") || is(argv[2], "device")) {
            copyBounded(net.data.deviceName, sizeof(net.data.deviceName), argv[3]);
            applyWifiConfig(ctx);
            tracker_serial_detail::printOk(out, "device name updated");
            saveIfRequested(ctx, hasSaveArg(argc, argv, 4));
            return true;
        }

        if (is(argv[2], "server")) {
            copyBounded(net.data.serverHost, sizeof(net.data.serverHost), argv[3]);
            net.data.manualServerEnabled = net.data.serverHost[0] != '\0';
            if (argc >= 5 && !is(argv[4], "save")) {
                uint32_t port = 0;
                if (!tracker_serial_detail::parseU32(argv[4], port) || port == 0 || port > 65535u) {
                    tracker_serial_detail::printErr(out, "invalid port");
                    return true;
                }
                net.data.serverPort = static_cast<uint16_t>(port);
            }
            applyWifiConfig(ctx);
            tracker_serial_detail::printOk(out, "server updated");
            saveIfRequested(ctx, hasSaveArg(argc, argv, 4));
            return true;
        }

        tracker_serial_detail::printErr(out, "usage: net set ssid|pass|name|server <value> [save]");
        return true;
    }

    if (is(argv[1], "clear")) {
        if (argc < 3) {
            tracker_serial_detail::printErr(out, "usage: net clear pass|server [save]");
            return true;
        }
        if (is(argv[2], "pass") || is(argv[2], "password")) {
            net.data.password[0] = '\0';
            applyWifiConfig(ctx);
            tracker_serial_detail::printOk(out, "password cleared; open network mode");
            saveIfRequested(ctx, hasSaveArg(argc, argv, 3));
            return true;
        }
        if (is(argv[2], "server")) {
            net.data.serverHost[0] = '\0';
            net.data.manualServerEnabled = false;
            applyWifiConfig(ctx);
            tracker_serial_detail::printOk(out, "manual server cleared");
            saveIfRequested(ctx, hasSaveArg(argc, argv, 3));
            return true;
        }
        tracker_serial_detail::printErr(out, "usage: net clear pass|server [save]");
        return true;
    }

    if (is(argv[1], "discovery")) {
        if (argc < 3) {
            out.print("discovery_enabled="); out.println(net.data.discoveryEnabled ? "yes" : "no");
            return true;
        }
        if (is(argv[2], "on")) net.data.discoveryEnabled = true;
        else if (is(argv[2], "off")) net.data.discoveryEnabled = false;
        else {
            tracker_serial_detail::printErr(out, "usage: net discovery on|off [save]");
            return true;
        }
        applyWifiConfig(ctx);
        tracker_serial_detail::printOk(out, "discovery updated");
        saveIfRequested(ctx, hasSaveArg(argc, argv, 3));
        return true;
    }

    if (is(argv[1], "save")) {
        saveIfRequested(ctx, true);
        return true;
    }

    if (is(argv[1], "load")) {
        if (!ctx.networkConfigStore) {
            tracker_serial_detail::printErr(out, "network config store not available");
            return true;
        }
        TrackerNetworkConfig loaded;
        if (!ctx.networkConfigStore->load(loaded)) {
            out.print("# ERR net load failed: ");
            out.println(ctx.networkConfigStore->lastErrorName());
            return true;
        }
        net = loaded;
        net.sanitize();
        if (ctx.networkConfigLoadedFromNvs) *ctx.networkConfigLoadedFromNvs = true;
        applyWifiConfig(ctx);
        tracker_serial_detail::printOk(out, "network config loaded");
        return true;
    }

    if (is(argv[1], "defaults")) {
        net.resetDefaults();
        if (ctx.networkConfigLoadedFromNvs) *ctx.networkConfigLoadedFromNvs = false;
        applyWifiConfig(ctx);
        tracker_serial_detail::printOk(out, "network config reset to defaults in RAM");
        saveIfRequested(ctx, hasSaveArg(argc, argv, 2));
        return true;
    }

    if (is(argv[1], "erase")) {
        if (!ctx.networkConfigStore) {
            tracker_serial_detail::printErr(out, "network config store not available");
            return true;
        }
        const bool ok = ctx.networkConfigStore->erase();
        net.resetDefaults();
        if (ctx.networkConfigLoadedFromNvs) *ctx.networkConfigLoadedFromNvs = false;
        applyWifiConfig(ctx);
        if (ok) tracker_serial_detail::printOk(out, "network config erased from NVS");
        else {
            out.print("# ERR net erase failed: ");
            out.println(ctx.networkConfigStore->lastErrorName());
        }
        return true;
    }

    if (is(argv[1], "reconnect")) {
        if (ctx.wifiManager) {
            ctx.wifiManager->reset();
            applyWifiConfig(ctx);
            tracker_serial_detail::printOk(out, "wifi reconnect requested");
        } else {
            tracker_serial_detail::printErr(out, "wifi manager not available");
        }
        return true;
    }

    if (is(argv[1], "counters")) {
        if (argc >= 3 && is(argv[2], "reset")) {
            if (ctx.wifiManager) ctx.wifiManager->resetCounters();
            tracker_serial_detail::printOk(out, "wifi counters reset");
        } else {
            tracker_serial_detail::printErr(out, "usage: net counters reset");
        }
        return true;
    }

    tracker_serial_detail::printErr(out, "unknown net command; use net help");
    return true;
}

} // namespace tracker

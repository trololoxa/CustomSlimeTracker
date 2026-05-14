#include "serial/tracker_network_commands.hpp"

#include <cstring>

#if defined(ARDUINO)
#include <WiFi.h>
#endif

#include "config/tracker_network_config.hpp"
#include "network/wifi_manager.hpp"

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
    out.println("net print [reveal]");
    out.println("net scan [visible|hidden] [limit <n>]");
    out.println("net password-info");
    out.println("net connect-test [seconds] [clean] [bssid <mac> ch <channel>]");
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


uint8_t cstringLenBounded(const char* s, uint8_t cap) {
    if (!s) return 0;
    uint8_t n = 0;
    while (n < cap && s[n] != '\0') ++n;
    return n;
}

bool isPrintableAscii(char c) {
    const unsigned char u = static_cast<unsigned char>(c);
    return u >= 0x20u && u <= 0x7Eu;
}

void printByteHex(Stream& out, uint8_t v) {
    out.print("0x");
    if (v < 0x10u) out.print('0');
    out.print(static_cast<unsigned int>(v), HEX);
}

uint32_t fnv1aString(const char* s) {
    uint32_t h = 2166136261u;
    if (!s) return h;
    while (*s) {
        h ^= static_cast<uint8_t>(*s++);
        h *= 16777619u;
    }
    return h;
}

int hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool parseBssid(const char* text, uint8_t out[6]) {
    if (!text || !out) return false;
    for (int i = 0; i < 6; ++i) {
        const int hi = hexNibble(text[0]);
        const int lo = hexNibble(text[1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = static_cast<uint8_t>((hi << 4) | lo);
        text += 2;
        if (i < 5) {
            if (*text != ':') return false;
            ++text;
        }
    }
    return *text == '\0';
}

void runPasswordInfo(TrackerSerialCommandContext& ctx) {
    Stream& out = netStream(ctx);
    if (!ctx.networkConfig) {
        tracker_serial_detail::printErr(out, "network config not available");
        return;
    }

    ctx.networkConfig->sanitize();
    const auto& n = ctx.networkConfig->data;
    const uint8_t ssidLen = cstringLenBounded(n.ssid, sizeof(n.ssid));
    const uint8_t passLen = cstringLenBounded(n.password, sizeof(n.password));

    bool passPrintable = true;
    bool passHasQuote = false;
    bool passHasLeadingOrTrailingSpace = false;
    for (uint8_t i = 0; i < passLen; ++i) {
        const char c = n.password[i];
        if (!isPrintableAscii(c)) passPrintable = false;
        if (c == '\"' || c == '\'') passHasQuote = true;
    }
    if (passLen > 0 && (n.password[0] == ' ' || n.password[passLen - 1] == ' ')) {
        passHasLeadingOrTrailingSpace = true;
    }

    out.println("# NET PASSWORD INFO");
    out.print("ssid_length="); out.println(ssidLen);
    out.print("ssid_hash_fnv1a=0x"); out.println(fnv1aString(n.ssid), HEX);
    out.print("password_set="); out.println(passLen > 0 ? "yes" : "no");
    out.print("password_length="); out.println(passLen);
    out.print("password_hash_fnv1a=0x"); out.println(fnv1aString(n.password), HEX);
    out.print("password_printable_ascii="); out.println(passPrintable ? "yes" : "no");
    out.print("password_has_quote_char="); out.println(passHasQuote ? "yes" : "no");
    out.print("password_leading_or_trailing_space="); out.println(passHasLeadingOrTrailingSpace ? "yes" : "no");
    if (passLen > 0) {
        out.print("password_first_byte="); printByteHex(out, static_cast<uint8_t>(n.password[0])); out.println();
        out.print("password_last_byte="); printByteHex(out, static_cast<uint8_t>(n.password[passLen - 1])); out.println();
    }
    if (passHasQuote) {
        out.println("# WARN password contains quote characters. Serial CLI does not strip quotes; enter without \"...\".");
    }
    if (passLen > 0 && passLen < 8) {
        out.println("# WARN WPA/WPA2-PSK password must be at least 8 characters.");
    }
}

#if defined(ARDUINO)
const char* wlStatusName(int status) {
    switch (status) {
        case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
        case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
        case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
        case WL_CONNECTED: return "WL_CONNECTED";
        case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
        case WL_DISCONNECTED: return "WL_DISCONNECTED";
        default: return "WL_UNKNOWN";
    }
}

void printArduinoIp(Stream& out, const IPAddress& ip) {
    out.print(static_cast<unsigned int>(ip[0]));
    out.print('.');
    out.print(static_cast<unsigned int>(ip[1]));
    out.print('.');
    out.print(static_cast<unsigned int>(ip[2]));
    out.print('.');
    out.print(static_cast<unsigned int>(ip[3]));
}
#endif


void runWifiConnectTest(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    Stream& out = netStream(ctx);
    if (!ctx.networkConfig) {
        tracker_serial_detail::printErr(out, "network config not available");
        return;
    }

    uint32_t timeoutSec = 20;
    bool cleanStart = false;
    bool useBssid = false;
    uint8_t bssid[6] = {0, 0, 0, 0, 0, 0};
    uint32_t channel = 0;

    for (int i = 2; i < argc; ++i) {
        if (!argv[i]) continue;

        if (is(argv[i], "clean") || is(argv[i], "wipe")) {
            cleanStart = true;
            continue;
        }
        if (is(argv[i], "bssid")) {
            if (i + 1 >= argc || !parseBssid(argv[i + 1], bssid)) {
                tracker_serial_detail::printErr(out, "usage: net connect-test [seconds] [clean] [bssid AA:BB:CC:DD:EE:FF ch <channel>]");
                return;
            }
            useBssid = true;
            ++i;
            continue;
        }
        if (is(argv[i], "ch") || is(argv[i], "channel")) {
            if (i + 1 >= argc || !tracker_serial_detail::parseU32(argv[i + 1], channel) || channel < 1 || channel > 14) {
                tracker_serial_detail::printErr(out, "channel must be 1..14");
                return;
            }
            ++i;
            continue;
        }

        uint32_t parsed = 0;
        if (!tracker_serial_detail::parseU32(argv[i], parsed) || parsed < 1 || parsed > 120) {
            tracker_serial_detail::printErr(out, "usage: net connect-test [seconds:1..120] [clean] [bssid AA:BB:CC:DD:EE:FF ch <channel>]");
            return;
        }
        timeoutSec = parsed;
    }

    if (useBssid && channel == 0) {
        tracker_serial_detail::printErr(out, "bssid mode requires channel: net connect-test bssid AA:BB:CC:DD:EE:FF ch <channel>");
        return;
    }

    ctx.networkConfig->sanitize();
    const auto& n = ctx.networkConfig->data;
    if (n.ssid[0] == '\0') {
        tracker_serial_detail::printErr(out, "set ssid first: net set ssid <ssid>");
        return;
    }

#if !defined(ARDUINO)
    tracker_serial_detail::printErr(out, "connect-test is only available on Arduino/ESP32 builds");
    (void)timeoutSec;
    (void)cleanStart;
    (void)useBssid;
    (void)bssid;
    (void)channel;
#else
    out.println("# NET CONNECT TEST");
    out.println("# blocking diagnostic connection; sensor processing pauses until it finishes");
    out.print("ssid="); out.println(n.ssid);
    out.print("ssid_length="); out.println(cstringLenBounded(n.ssid, sizeof(n.ssid)));
    out.print("password_set="); out.println(n.password[0] != '\0' ? "yes" : "no");
    out.print("password_length="); out.println(cstringLenBounded(n.password, sizeof(n.password)));
    out.print("hostname="); out.println(n.deviceName);
    out.print("timeout_s="); out.println(timeoutSec);
    out.print("clean_start="); out.println(cleanStart ? "yes" : "no");
    out.print("explicit_bssid="); out.println(useBssid ? "yes" : "no");
    if (useBssid) {
        out.print("target_bssid="); printMac(out, bssid); out.println();
        out.print("target_channel="); out.println(channel);
    }

    if (ctx.wifiManager) {
        ctx.wifiManager->reset();
    }

    if (cleanStart) {
        out.println("# clean start: WiFi.disconnect(true,true), WIFI_OFF, then WIFI_STA");
        WiFi.disconnect(true, true);
        delay(500);
        WiFi.mode(WIFI_OFF);
        delay(500);
    } else {
        WiFi.disconnect(false, false);
        delay(250);
    }

    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
#if defined(ESP32)
    WiFi.setSleep(false);
#endif

    bool hostOk = true;
    if (n.deviceName[0] != '\0') {
        hostOk = WiFi.setHostname(n.deviceName);
    }
    out.print("set_hostname_ok="); out.println(hostOk ? "yes" : "no");

    const unsigned long start = millis();
    if (useBssid) {
        if (n.password[0] != '\0') WiFi.begin(n.ssid, n.password, static_cast<int32_t>(channel), bssid, true);
        else WiFi.begin(n.ssid, nullptr, static_cast<int32_t>(channel), bssid, true);
    } else {
        if (n.password[0] != '\0') WiFi.begin(n.ssid, n.password);
        else WiFi.begin(n.ssid);
    }

    int lastStatus = -999;
    unsigned long lastPrint = 0;
    while (millis() - start < timeoutSec * 1000UL) {
        const int status = static_cast<int>(WiFi.status());
        const unsigned long now = millis();
        if (status != lastStatus || now - lastPrint >= 1000UL) {
            out.print("t_ms="); out.print(now - start);
            out.print(" raw_status="); out.print(status);
            out.print(" status="); out.println(wlStatusName(status));
            lastStatus = status;
            lastPrint = now;
        }
        if (status == WL_CONNECTED) break;
        delay(100);
    }

    const int finalStatus = static_cast<int>(WiFi.status());
    out.print("final_raw_status="); out.println(finalStatus);
    out.print("final_status="); out.println(wlStatusName(finalStatus));

    if (finalStatus == WL_CONNECTED) {
        out.print("ip="); printArduinoIp(out, WiFi.localIP()); out.println();
        out.print("gateway="); printArduinoIp(out, WiFi.gatewayIP()); out.println();
        out.print("subnet="); printArduinoIp(out, WiFi.subnetMask()); out.println();
        out.print("dns="); printArduinoIp(out, WiFi.dnsIP()); out.println();
        out.print("rssi_dbm="); out.println(WiFi.RSSI());
        out.print("channel="); out.println(WiFi.channel());
        out.print("bssid="); out.println(WiFi.BSSIDstr());
        out.print("mac="); out.println(WiFi.macAddress());
        tracker_serial_detail::printOk(out, "blocking wifi connect succeeded");
    } else {
        tracker_serial_detail::printErr(out, "blocking wifi connect failed");
        out.println("# next checks:");
        out.println("# 1) run: net password-info");
        out.println("# 2) if password_has_quote_char=yes, re-enter password without quotes");
        out.println("# 3) try explicit AP: net connect-test 30 clean bssid <BSSID_FROM_SCAN> ch <CHANNEL_FROM_SCAN>");
        out.println("# 4) try a temporary open hotspot: net clear pass; net set ssid <open_ssid>; net connect-test 30 clean");
    }

    if (ctx.wifiManager) {
        applyWifiConfig(ctx);
    }
#endif
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
        if (argc >= 3 && is(argv[2], "reveal")) {
            out.print("password_reveal="); out.println(net.data.password);
        }
        return true;
    }

    if (is(argv[1], "password-info") || is(argv[1], "pass-info")) {
        runPasswordInfo(ctx);
        return true;
    }

    if (is(argv[1], "scan")) {
        runWifiScan(ctx, argc, argv);
        return true;
    }

    if (is(argv[1], "connect-test") || is(argv[1], "jointest") || is(argv[1], "join-test")) {
        runWifiConnectTest(ctx, argc, argv);
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

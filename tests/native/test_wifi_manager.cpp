#include "test_common.hpp"

#include "network/wifi_manager.hpp"

#include <cstring>

using namespace tracker;

class FakeWifiAdapter final : public IWifiStationAdapter {
public:
    bool beginResult = true;
    WifiStationInfo current;
    uint32_t beginCalls = 0;
    uint32_t disconnectCalls = 0;
    char lastSsid[33] = "";
    char lastPassword[65] = "";
    char lastHostname[32] = "";
    uint32_t scanCalls = 0;
    uint32_t clearScanCalls = 0;
    int16_t scanReturn = 0;

    bool begin(const char* ssid, const char* password, const char* hostname) override {
        ++beginCalls;
        copy(lastSsid, sizeof(lastSsid), ssid);
        copy(lastPassword, sizeof(lastPassword), password);
        copy(lastHostname, sizeof(lastHostname), hostname);
        current.linkStatus = WifiLinkStatus::Connecting;
        return beginResult;
    }

    void disconnect() override {
        ++disconnectCalls;
        current.linkStatus = WifiLinkStatus::Disconnected;
    }

    WifiStationInfo info() const override { return current; }

    int16_t scanNetworks(WifiScanResult* results, uint8_t maxResults, bool showHidden) override {
        ++scanCalls;
        if (results && maxResults >= 2) {
            results[0].rssiDbm = -42;
            results[0].channel = 6;
            results[0].authType = WifiAuthType::Wpa2Psk;
            results[0].hidden = false;
            copy(results[0].ssid, sizeof(results[0].ssid), "net-a");
            copy(results[0].bssid, sizeof(results[0].bssid), "AA:BB:CC:DD:EE:01");
            results[1].rssiDbm = -77;
            results[1].channel = 11;
            results[1].authType = WifiAuthType::Open;
            results[1].hidden = showHidden;
            copy(results[1].ssid, sizeof(results[1].ssid), showHidden ? "" : "guest");
            copy(results[1].bssid, sizeof(results[1].bssid), "AA:BB:CC:DD:EE:02");
        }
        return scanReturn;
    }

    void clearScanResults() override { ++clearScanCalls; }

private:
    static void copy(char* dst, std::size_t n, const char* src) {
        if (!dst || n == 0) return;
        if (!src) src = "";
        std::size_t i = 0;
        for (; i + 1u < n && src[i] != '\0'; ++i) {
            dst[i] = src[i];
        }
        dst[i] = '\0';
    }
};

static TrackerWifiManagerConfig enabledConfig() {
    TrackerWifiManagerConfig cfg;
    cfg.enabled = true;
    cfg.credentialsValid = true;
    cfg.ssid = "test-net";
    cfg.password = "secret";
    cfg.hostname = "tracker-c3";
    cfg.connectTimeoutMs = 1000;
    cfg.reconnectBackoffMs = 500;
    cfg.statusPollIntervalMs = 50;
    return cfg;
}

int main() {
    TestContext ctx;

    CHECK(ctx, std::strcmp(wifiLinkStatusName(WifiLinkStatus::Connected), "connected") == 0);
    CHECK(ctx, std::strcmp(wifiAuthTypeName(WifiAuthType::Wpa2Wpa3Psk), "WPA2/WPA3-PSK") == 0);
    CHECK(ctx, std::strcmp(trackerWifiStateName(TrackerWifiState::Backoff), "backoff") == 0);


    {
        FakeWifiAdapter fake;
        fake.scanReturn = 2;
        TrackerWifiManager wifi;
        wifi.begin(fake);

        WifiScanResult results[2];
        const int16_t count = wifi.scanNetworks(results, 2, true);
        CHECK(ctx, count == 2);
        CHECK(ctx, fake.scanCalls == 1);
        CHECK(ctx, std::strcmp(results[0].ssid, "net-a") == 0);
        CHECK(ctx, results[0].rssiDbm == -42);
        CHECK(ctx, results[0].channel == 6);
        CHECK(ctx, results[0].authType == WifiAuthType::Wpa2Psk);
        CHECK(ctx, results[1].hidden);
        wifi.clearScanResults();
        CHECK(ctx, fake.clearScanCalls == 1);
    }

    {
        FakeWifiAdapter fake;
        TrackerWifiManager wifi;
        wifi.begin(fake);
        wifi.configure(enabledConfig());

        wifi.update(0);
        CHECK(ctx, wifi.state() == TrackerWifiState::Connecting);
        CHECK(ctx, fake.beginCalls == 1);
        CHECK(ctx, std::strcmp(fake.lastSsid, "test-net") == 0);
        CHECK(ctx, std::strcmp(fake.lastPassword, "secret") == 0);
        CHECK(ctx, std::strcmp(fake.lastHostname, "tracker-c3") == 0);

        fake.current.linkStatus = WifiLinkStatus::Connected;
        fake.current.ipv4 = 0xC0A80123UL;
        fake.current.rssiDbm = -47;
        fake.current.mac[0] = 0xAA;
        wifi.update(60);

        const TrackerWifiManagerStatus st = wifi.status();
        CHECK(ctx, wifi.connected());
        CHECK(ctx, st.state == TrackerWifiState::Connected);
        CHECK(ctx, st.ipv4 == 0xC0A80123UL);
        CHECK(ctx, st.rssiDbm == -47);
        CHECK(ctx, st.mac[0] == 0xAA);
    }

    {
        FakeWifiAdapter fake;
        TrackerWifiManager wifi;
        wifi.begin(fake);
        TrackerWifiManagerConfig cfg = enabledConfig();
        cfg.connectTimeoutMs = 100;
        cfg.reconnectBackoffMs = 200;
        wifi.configure(cfg);

        wifi.update(10);
        CHECK(ctx, fake.beginCalls == 1);
        CHECK(ctx, wifi.state() == TrackerWifiState::Connecting);

        wifi.update(120);
        CHECK(ctx, wifi.state() == TrackerWifiState::Backoff);
        CHECK(ctx, fake.disconnectCalls == 1);
        CHECK(ctx, wifi.status().connectTimeouts == 1);

        wifi.update(250);
        CHECK(ctx, fake.beginCalls == 1);
        wifi.update(330);
        CHECK(ctx, fake.beginCalls == 2);
        CHECK(ctx, wifi.state() == TrackerWifiState::Connecting);
    }

    {
        FakeWifiAdapter fake;
        TrackerWifiManager wifi;
        wifi.begin(fake);
        wifi.configure(enabledConfig());
        wifi.update(0);
        CHECK(ctx, fake.beginCalls == 1);

        TrackerWifiManagerConfig cfg = enabledConfig();
        cfg.enabled = false;
        wifi.configure(cfg);
        wifi.update(1);
        CHECK(ctx, wifi.state() == TrackerWifiState::Disabled);
        CHECK(ctx, fake.disconnectCalls >= 1);
        CHECK(ctx, !wifi.connected());
    }

    {
        FakeWifiAdapter fake;
        TrackerWifiManager wifi;
        wifi.begin(fake);
        wifi.configure(enabledConfig());
        wifi.update(0);
        fake.current.linkStatus = WifiLinkStatus::Connected;
        wifi.update(60);
        CHECK(ctx, wifi.connected());

        TrackerWifiManagerConfig cfg = enabledConfig();
        cfg.ssid = "other-net";
        wifi.configure(cfg);
        CHECK(ctx, wifi.state() == TrackerWifiState::Disabled);
        CHECK(ctx, fake.disconnectCalls >= 1);
        wifi.update(100);
        CHECK(ctx, fake.beginCalls == 2);
        CHECK(ctx, std::strcmp(fake.lastSsid, "other-net") == 0);
    }

    {
        FakeWifiAdapter fake;
        TrackerWifiManager wifi;
        wifi.begin(fake);
        TrackerWifiManagerConfig cfg = enabledConfig();
        cfg.credentialsValid = false;
        wifi.configure(cfg);
        wifi.update(0);
        CHECK(ctx, wifi.state() == TrackerWifiState::Disabled);
        CHECK(ctx, fake.beginCalls == 0);
    }

    {
        FakeWifiAdapter fake;
        TrackerWifiManager wifi;
        wifi.begin(fake);
        TrackerWifiManagerConfig cfg = enabledConfig();
        cfg.connectTimeoutMs = 1000;
        cfg.reconnectBackoffMs = 200;
        wifi.configure(cfg);

        wifi.update(0);
        CHECK(ctx, fake.beginCalls == 1);
        CHECK(ctx, wifi.state() == TrackerWifiState::Connecting);

        fake.current.linkStatus = WifiLinkStatus::ConnectFailed;
        wifi.update(60);
        CHECK(ctx, wifi.state() == TrackerWifiState::Connecting);
        CHECK(ctx, fake.disconnectCalls == 0);
        CHECK(ctx, wifi.status().connectTimeouts == 0);
        CHECK(ctx, wifi.linkStatus() == WifiLinkStatus::ConnectFailed);

        fake.current.linkStatus = WifiLinkStatus::NoSsid;
        wifi.update(120);
        CHECK(ctx, wifi.state() == TrackerWifiState::Connecting);
        CHECK(ctx, fake.disconnectCalls == 0);

        wifi.update(1000);
        CHECK(ctx, wifi.state() == TrackerWifiState::Backoff);
        CHECK(ctx, fake.disconnectCalls == 1);
        CHECK(ctx, wifi.status().connectTimeouts == 1);
    }

    return ctx.finish("wifi_manager");
}

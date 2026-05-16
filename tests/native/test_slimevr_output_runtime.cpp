#include "test_common.hpp"

#include <cstring>
#include <vector>

#include "network/udp_transport.hpp"
#include "network/wifi_manager.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "runtime/tracker_runtime_types.hpp"

using namespace tracker;

class FakeWifiAdapter final : public IWifiStationAdapter {
public:
    WifiStationInfo current;
    bool begin(const char*, const char*, const char*) override { current.linkStatus = WifiLinkStatus::Connecting; return true; }
    void disconnect() override { current.linkStatus = WifiLinkStatus::Disconnected; }
    WifiStationInfo info() const override { return current; }
    int16_t scanNetworks(WifiScanResult*, uint8_t, bool) override { return 0; }
    void clearScanResults() override {}
};

class FakeUdp final : public IUdpTransport {
public:
    struct SentPacket { UdpEndpoint endpoint; std::vector<uint8_t> data; };

    bool beginResult = true;
    bool isActive = false;
    uint16_t port = 0;
    uint32_t beginCalls = 0;
    uint32_t stopCalls = 0;
    std::vector<SentPacket> sent;
    std::vector<uint8_t> incoming;
    UdpEndpoint incomingRemote;
    bool incomingPending = false;

    bool begin(uint16_t localPort) override {
        ++beginCalls;
        if (!beginResult) return false;
        isActive = true;
        port = localPort;
        return true;
    }
    void stop() override { ++stopCalls; isActive = false; port = 0; }
    bool active() const override { return isActive; }
    uint16_t localPort() const override { return port; }

    bool send(const UdpEndpoint& endpoint, const uint8_t* data, size_t len) override {
        if (!isActive || !data || len == 0) return false;
        SentPacket p;
        p.endpoint = endpoint;
        p.data.assign(data, data + len);
        sent.push_back(p);
        return true;
    }

    int parsePacket() override { return incomingPending ? static_cast<int>(incoming.size()) : 0; }
    int read(uint8_t* data, size_t maxLen) override {
        if (!incomingPending || !data) return 0;
        const size_t n = incoming.size() < maxLen ? incoming.size() : maxLen;
        std::memcpy(data, incoming.data(), n);
        incomingPending = false;
        return static_cast<int>(n);
    }
    UdpEndpoint remoteEndpoint() const override { return incomingRemote; }
};

struct FakeSnapshotSource {
    TrackerPreparedOutputSnapshot snapshot;

    static bool copy(TrackerPreparedOutputSnapshot& out, void* user) {
        auto* self = static_cast<FakeSnapshotSource*>(user);
        out = self->snapshot;
        return out.valid;
    }
};

static TrackerWifiManagerConfig wifiConfig() {
    TrackerWifiManagerConfig cfg;
    cfg.enabled = true;
    cfg.credentialsValid = true;
    cfg.ssid = "test";
    cfg.password = "pw";
    cfg.hostname = "tracker";
    cfg.connectTimeoutMs = 1000;
    cfg.statusPollIntervalMs = 10;
    return cfg;
}

int main() {
    TestContext ctx;

    char ip[24];
    CHECK(ctx, std::strcmp(udpIpv4ToCString(0xC0A80073UL, ip, sizeof(ip)), "192.168.0.115") == 0);
    CHECK(ctx, std::strcmp(slimevrOutputStateName(SlimeVROutputState::Discovering), "discovering") == 0);

    FakeWifiAdapter wifiAdapter;
    wifiAdapter.current.linkStatus = WifiLinkStatus::Connected;
    wifiAdapter.current.ipv4 = 0xC0A80073UL;
    wifiAdapter.current.mac[0] = 0xE8;
    wifiAdapter.current.rssiDbm = -68;
    TrackerWifiManager wifi;
    wifi.begin(wifiAdapter);
    wifi.configure(wifiConfig());
    wifi.update(0);
    wifiAdapter.current.linkStatus = WifiLinkStatus::Connected;
    wifiAdapter.current.ipv4 = 0xC0A80073UL;
    wifiAdapter.current.mac[0] = 0xE8;
    wifi.update(20);
    CHECK(ctx, wifi.connected());

    FakeUdp udp;
    FakeSnapshotSource snapshots;
    snapshots.snapshot.valid = true;
    snapshots.snapshot.sequence = 1;
    snapshots.snapshot.runtimeSample = 123;
    snapshots.snapshot.timestampUs = 456789;
    snapshots.snapshot.q = Quat::identity();
    snapshots.snapshot.qualityFlags = 0x1234;
    snapshots.snapshot.confidence = 0.99f;

    SlimeVROutputRuntime rt;
    rt.begin(udp, wifi, FakeSnapshotSource::copy, &snapshots);

    SlimeVROutputRuntimeConfig cfg;
    cfg.enabled = true;
    cfg.deviceName = "unit-test";
    cfg.sensorId = 2;
    cfg.serverPort = 6969;
    cfg.localPort = 6969;
    cfg.discoveryIntervalMs = 1000;
    cfg.rotationRateHz = 100;
    cfg.magSupportEnabled = true;
    cfg.latestTemperatureValid = true;
    cfg.latestTemperatureC = 42.5f;
    rt.configure(cfg);

    rt.update(1000);
    CHECK(ctx, udp.beginCalls == 1);
    CHECK(ctx, udp.active());
    CHECK(ctx, udp.sent.size() == 1u);
    CHECK(ctx, udp.sent[0].endpoint.ipv4 == SLIMEVR_DISCOVERY_BROADCAST_IPV4);
    CHECK(ctx, udp.sent[0].endpoint.port == 6969);
    CHECK(ctx, udp.sent[0].data.size() >= SLIMEVR_PACKET_HEADER_SIZE);
    CHECK(ctx, udp.sent[0].data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::Handshake));
    CHECK(ctx, rt.status().handshakesSent == 1);

    udp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    udp.incomingRemote = UdpEndpoint{0xC0A80001UL, 6969};
    udp.incomingPending = true;
    rt.update(1100);

    const SlimeVROutputRuntimeStatus st = rt.status();
    CHECK(ctx, st.serverFound);
    CHECK(ctx, st.state == SlimeVROutputState::ServerFound);
    CHECK(ctx, st.serverIpv4 == 0xC0A80001UL);
    CHECK(ctx, st.discoveryResponses == 1);
    CHECK(ctx, st.sensorInfoSent == 1);
    CHECK(ctx, st.rotationSent == 1);
    CHECK(ctx, st.signalStrengthSent == 1);
    CHECK(ctx, st.temperatureSent == 1);
    CHECK(ctx, st.magnetometerAccuracySent == 0);
    CHECK(ctx, st.magSupportEnabled);
    CHECK(ctx, st.sensorConfig == SLIMEVR_SENSOR_CONFIG_MAG_SUPPORTED_AND_ENABLED);
    CHECK(ctx, st.lastSignalStrength == 64);
    CHECK(ctx, st.lastRssiDbm == -68);
    CHECK(ctx, st.lastTemperatureValid);
    CHECK_NEAR(ctx, st.lastTemperatureC, 42.5f, 1.0e-6f);
    CHECK(ctx, st.rotationNoSnapshot == 0);
    CHECK(ctx, st.rotationDuplicateSnapshot == 0);
    CHECK(ctx, st.lastRotationSnapshotSequence == 1);
    CHECK(ctx, st.lastRotationRuntimeSample == 123);
    CHECK(ctx, st.lastRotationTimestampUs == 456789ULL);
    CHECK(ctx, st.lastRotationQualityFlags == 0x1234u);
    CHECK_NEAR(ctx, st.lastRotationConfidence, 0.99f, 1.0e-6f);
    CHECK(ctx, udp.sent.size() >= 3u);
    CHECK(ctx, udp.sent[1].endpoint.ipv4 == 0xC0A80001UL);
    CHECK(ctx, udp.sent[1].data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::SensorInfo));
    CHECK(ctx, udp.sent.back().endpoint.ipv4 == 0xC0A80001UL);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::RotationData));

    rt.update(1105);
    CHECK(ctx, rt.status().rotationSent == 1);

    snapshots.snapshot.sequence = 2;
    snapshots.snapshot.runtimeSample = 124;
    rt.update(1110);
    CHECK(ctx, rt.status().rotationSent == 2);
    CHECK(ctx, rt.status().lastRotationSnapshotSequence == 2);

    rt.update(6100);
    CHECK(ctx, rt.status().heartbeatSent >= 1);

    return ctx.finish("slimevr_output_runtime");
}

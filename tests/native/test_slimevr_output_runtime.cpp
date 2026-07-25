#include "test_common.hpp"

#include <Arduino.h>

#include <cstring>
#include <initializer_list>
#include <vector>

#include "build_config/build_identity.hpp"
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
    int failPacketType = -1;
    bool resolveResult = true;
    uint32_t resolvedIpv4 = 0xC0A8002AUL;
    uint32_t resolveCalls = 0;
    char lastResolvedHost[64] = {};

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

    bool resolveHost(const char* host, uint32_t& outIpv4) override {
        ++resolveCalls;
        std::strncpy(lastResolvedHost, host ? host : "", sizeof(lastResolvedHost) - 1u);
        lastResolvedHost[sizeof(lastResolvedHost) - 1u] = '\0';
        outIpv4 = resolveResult ? resolvedIpv4 : 0u;
        return resolveResult;
    }

    bool send(const UdpEndpoint& endpoint, const uint8_t* data, size_t len) override {
        if (!isActive || !data || len == 0) return false;
        if (failPacketType >= 0 && len >= 4u && data[3] == static_cast<uint8_t>(failPacketType)) return false;
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



static uint32_t readU32BeLocal(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
}

static float readF32BeLocal(const uint8_t* p) {
    union { uint32_t u; float f; } v{};
    v.u = readU32BeLocal(p);
    return v.f;
}


static std::vector<uint8_t> makeServerPacket(uint8_t type, std::initializer_list<uint8_t> payload) {
    std::vector<uint8_t> packet(12, 0);
    packet[3] = type;
    packet.insert(packet.end(), payload.begin(), payload.end());
    return packet;
}

static std::vector<uint8_t> makeSensorInfoAck(uint8_t sensorId, uint8_t status) {
    return {
        0x00, 0x00, 0x00, static_cast<uint8_t>(SlimeVRReceivePacketType::SensorInfo),
        sensorId, status
    };
}

struct FakeConfigFlagSink {
    uint32_t calls = 0;
    uint8_t sensorId = 0;
    uint16_t configType = 0;
    bool enabled = false;
    bool result = true;

    static bool apply(uint8_t sensorId, uint16_t configType, bool enabled, void* user) {
        auto* self = static_cast<FakeConfigFlagSink*>(user);
        ++self->calls;
        self->sensorId = sensorId;
        self->configType = configType;
        self->enabled = enabled;
        return self->result;
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
    uint32_t parsedIpv4 = 0;
    CHECK(ctx, udpParseIpv4("192.168.0.42", parsedIpv4));
    CHECK(ctx, parsedIpv4 == 0xC0A8002AUL);
    CHECK(ctx, !udpParseIpv4("192.168.0.999", parsedIpv4));
    CHECK(ctx, !udpParseIpv4("192.168.0", parsedIpv4));
    CHECK(ctx, !udpParseIpv4("0.0.0.0", parsedIpv4));
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
    snapshots.snapshot.publishedAtMcuUs = 1200000u;
    snapshots.snapshot.q = Quat::identity();
    snapshots.snapshot.linearAccelerationValid = true;
    snapshots.snapshot.linearAccelerationDeviceG = Vec3(0.25f, -0.5f, 1.75f);
    snapshots.snapshot.qualityFlags = 0x1234;
    snapshots.snapshot.confidence = 0.99f;

    SlimeVROutputRuntime rt;
    FakeConfigFlagSink configFlagSink;
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
    cfg.magEnabled = true;
    cfg.latestTemperatureValid = true;
    cfg.latestTemperatureC = 42.5f;
    cfg.batteryTelemetryEnabled = true;
    cfg.latestBatteryValid = true;
    cfg.latestBatteryVoltage = 3.80f;
    cfg.latestBatteryPercentage = 55.0f;
    cfg.setConfigFlag = FakeConfigFlagSink::apply;
    cfg.setConfigFlagUser = &configFlagSink;
    rt.configure(cfg);

    rt.update(1000);
    CHECK(ctx, udp.beginCalls == 1);
    CHECK(ctx, udp.active());
    CHECK(ctx, udp.sent.size() == 1u);
    CHECK(ctx, udp.sent[0].endpoint.ipv4 == SLIMEVR_DISCOVERY_BROADCAST_IPV4);
    CHECK(ctx, udp.sent[0].endpoint.port == 6969);
    CHECK(ctx, udp.sent[0].data.size() >= SLIMEVR_PACKET_HEADER_SIZE);
    CHECK(ctx, udp.sent[0].data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::Handshake));
    constexpr size_t handshakeFirmwareLengthOffset = SLIMEVR_PACKET_HEADER_SIZE + 7u * sizeof(uint32_t);
    CHECK(ctx, udp.sent[0].data.size() > handshakeFirmwareLengthOffset);
    const size_t firmwareLength = udp.sent[0].data[handshakeFirmwareLengthOffset];
    CHECK(ctx, firmwareLength == std::strlen(trackerBuildFirmwareVersion()));
    CHECK(ctx, udp.sent[0].data.size() >= handshakeFirmwareLengthOffset + 1u + firmwareLength);
    CHECK(ctx, std::memcmp(
        udp.sent[0].data.data() + handshakeFirmwareLengthOffset + 1u,
        trackerBuildFirmwareVersion(),
        firmwareLength
    ) == 0);
    CHECK(ctx, rt.status().handshakesSent == 1);

    // Control/capability packets received before a discovery response do not
    // establish a session or negotiate bundle support.
    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags), {0x01}
    );
    udp.incomingRemote = UdpEndpoint{0xC0A80009UL, 6969};
    udp.incomingPending = true;
    rt.update(1050);
    CHECK(ctx, rt.status().preSessionPacketsDropped == 1u);
    CHECK(ctx, !rt.status().serverFeatureFlagsAvailable);

    udp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    udp.incomingRemote = UdpEndpoint{0xC0A80001UL, 6969};
    udp.incomingPending = true;
    rt.update(1100);

    {
        const SlimeVROutputRuntimeStatus st = rt.status();
        CHECK(ctx, st.protocolVersion == 22u);
        CHECK(ctx, st.serverFound);
        CHECK(ctx, st.state == SlimeVROutputState::ServerFound);
        CHECK(ctx, st.serverIpv4 == 0xC0A80001UL);
        CHECK(ctx, st.discoveryResponses == 1);
        CHECK(ctx, st.sensorInfoSent == 0);
        CHECK(ctx, st.rotationSent == 0);
        CHECK(ctx, st.signalStrengthSent == 0);
        CHECK(ctx, st.temperatureSent == 0);
        CHECK(ctx, st.batterySent == 0);
    }

    trackerTestSetMicros(1201500u);
    rt.update(1201);
    SlimeVROutputRuntimeStatus st = rt.status();
    CHECK(ctx, st.sensorInfoSent == 1u);
    CHECK(ctx, st.sensorInfoSyncState == SlimeVRSensorInfoSyncState::WaitingForAck);
    CHECK(ctx, st.sensorInfoDirty);
    CHECK(ctx, st.sensorInfoLocalConfig == SLIMEVR_SENSOR_CONFIG_MAG_SUPPORTED_AND_ENABLED);
    CHECK(ctx, st.featureFlagsSent == 1u);
    CHECK(ctx, st.featureNegotiationState == SlimeVRFeatureNegotiationState::Waiting);
    CHECK(ctx, st.featureFlagsSendFailures == 0u);
    CHECK(ctx, !st.serverFeatureFlagsAvailable);
    CHECK(ctx, !st.serverBundleSupported);
    CHECK(ctx, st.motionPacketMode == SlimeVRMotionPacketMode::SeparateRotation17Accel4);
    CHECK(ctx, !st.compactMotionEnabled);
    CHECK(ctx, st.compactMotionSent == 0u);
    CHECK(ctx, st.bundledMotionSent == 0u);
    CHECK(ctx, st.rotationSent == 1u);
    CHECK(ctx, st.accelerationSent == 1u);
    {
        uint32_t slackMs = 0u;
        CHECK(ctx, rt.rotationDeadlineSlackMs(1201u, slackMs));
        CHECK(ctx, slackMs == 9u);
        CHECK(ctx, rt.rotationDeadlineSlackMs(1207u, slackMs));
        CHECK(ctx, slackMs == 3u);
        CHECK(ctx, rt.rotationDeadlineSlackMs(1211u, slackMs));
        CHECK(ctx, slackMs == 0u);
    }
    CHECK(ctx, st.accelerationRateLimited == 0u);
    CHECK(ctx, st.accelerationSkippedInvalid == 0u);
    CHECK(ctx, st.accelerationSendFailures == 0u);
    CHECK(ctx, st.lastRotationSnapshotSequence == 1u);
    CHECK(ctx, st.lastRotationRuntimeSample == 123u);
    CHECK(ctx, st.lastRotationTimestampUs == 456789ULL);
    CHECK(ctx, st.lastRotationQualityFlags == 0x1234u);
    CHECK(ctx, st.lastRotationSnapshotAgeUs == 1500u);
    CHECK_NEAR(ctx, st.lastRotationConfidence, 0.99f, 1.0e-6f);

    bool sawFeatureFlags = false;
    bool sawSensorInfo = false;
    bool sawRotation = false;
    bool sawAcceleration = false;
    for (const auto& sent : udp.sent) {
        if (sent.data.size() < 4u) continue;
        const uint8_t type = sent.data[3];
        sawFeatureFlags = sawFeatureFlags || type == static_cast<uint8_t>(SlimeVRSendPacketType::FeatureFlags);
        sawSensorInfo = sawSensorInfo || type == static_cast<uint8_t>(SlimeVRSendPacketType::SensorInfo);
        sawRotation = sawRotation || type == static_cast<uint8_t>(SlimeVRSendPacketType::RotationData);
        if (type == static_cast<uint8_t>(SlimeVRSendPacketType::Accel)) {
            sawAcceleration = true;
            CHECK_NEAR(ctx, readF32BeLocal(sent.data.data() + 12), 0.25f * 9.80665f, 1.0e-5f);
            CHECK_NEAR(ctx, readF32BeLocal(sent.data.data() + 16), -0.5f * 9.80665f, 1.0e-5f);
            CHECK_NEAR(ctx, readF32BeLocal(sent.data.data() + 20), 1.75f * 9.80665f, 1.0e-5f);
        }
    }
    CHECK(ctx, sawFeatureFlags);
    CHECK(ctx, sawSensorInfo);
    CHECK(ctx, sawRotation);
    CHECK(ctx, sawAcceleration);

    // A well-formed ACK for another sensor must not acknowledge our latest
    // SensorInfo state or stop resend.
    udp.incoming = makeSensorInfoAck(
        static_cast<uint8_t>(cfg.sensorId + 1u),
        static_cast<uint8_t>(SlimeVRSensorState::Online)
    );
    udp.incomingRemote = UdpEndpoint{0xC0A80001UL, 6969};
    udp.incomingPending = true;
    rt.update(1206);
    CHECK(ctx, rt.status().sensorInfoAckReceived == 1u);
    CHECK(ctx, rt.status().sensorInfoAckMismatch == 1u);
    CHECK(ctx, rt.status().sensorInfoSyncState == SlimeVRSensorInfoSyncState::WaitingForAck);
    CHECK(ctx, rt.status().sensorInfoDirty);

    udp.incoming = makeSensorInfoAck(
        cfg.sensorId, static_cast<uint8_t>(SlimeVRSensorState::Online)
    );
    udp.incomingRemote = UdpEndpoint{0xC0A80001UL, 6969};
    udp.incomingPending = true;
    rt.update(1211);
    CHECK(ctx, rt.status().sensorInfoAckReceived == 2u);
    CHECK(ctx, rt.status().sensorInfoAckMismatch == 1u);
    CHECK(ctx, rt.status().sensorInfoSyncState == SlimeVRSensorInfoSyncState::Acknowledged);
    CHECK(ctx, !rt.status().sensorInfoDirty);
    CHECK(ctx, rt.status().sensorInfoAckConfig == SLIMEVR_SENSOR_CONFIG_MAG_SUPPORTED_AND_ENABLED);

    const size_t sentBeforeTap = udp.sent.size();
    CHECK(ctx, rt.sendTap(2));
    CHECK(ctx, rt.status().tapSent == 1u);
    CHECK(ctx, rt.status().tapSendFailures == 0u);
    CHECK(ctx, rt.status().lastTapValue == 2u);
    CHECK(ctx, udp.sent.size() == sentBeforeTap + 1u);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::Tap));

    const size_t sentBeforeAction = udp.sent.size();
    CHECK(ctx, rt.sendUserAction(SlimeVRUserAction::YawReset));
    CHECK(ctx, rt.status().userActionSent == 1u);
    CHECK(ctx, rt.status().userActionSendFailures == 0u);
    CHECK(ctx, rt.status().lastUserAction == SlimeVRUserAction::YawReset);
    CHECK(ctx, udp.sent.size() == sentBeforeAction + 1u);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::UserAction));
    CHECK(ctx, udp.sent.back().data[12] == static_cast<uint8_t>(SlimeVRUserAction::YawReset));

    rt.update(1212);
    CHECK(ctx, rt.status().rotationSent == 1u);

    // A second server cannot hijack a live endpoint with either a discovery
    // response or FeatureFlags. Foreign traffic also must not refresh the
    // selected server's silence timer.
    const uint32_t selectedServerIp = rt.status().serverIpv4;
    const uint32_t acceptedRxBeforeForeign = rt.status().lastIncomingPacketMs;
    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags), {0x01}
    );
    udp.incomingRemote = UdpEndpoint{0xC0A80009UL, 6969};
    udp.incomingPending = true;
    rt.update(1216);
    CHECK(ctx, rt.status().foreignEndpointPacketsDropped == 1u);
    CHECK(ctx, rt.status().serverIpv4 == selectedServerIp);
    CHECK(ctx, rt.status().lastIncomingPacketMs == acceptedRxBeforeForeign);
    CHECK(ctx, !rt.status().serverFeatureFlagsAvailable);

    udp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    udp.incomingRemote = UdpEndpoint{0xC0A80009UL, 6969};
    udp.incomingPending = true;
    rt.update(1221);
    CHECK(ctx, rt.status().foreignEndpointPacketsDropped == 2u);
    CHECK(ctx, rt.status().serverIpv4 == selectedServerIp);
    CHECK(ctx, rt.status().discoveryResponses == 1u);

    // Empty FeatureFlags from the selected server are malformed. They do not
    // complete negotiation, so a later valid response can still enable bundle.
    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags), {}
    );
    udp.incomingRemote = UdpEndpoint{selectedServerIp, 6969};
    udp.incomingPending = true;
    rt.update(1231);
    CHECK(ctx, rt.status().featureFlagsReceived == 1u);
    CHECK(ctx, rt.status().malformedFeatureFlags == 1u);
    CHECK(ctx, rt.status().malformedPackets == 1u);
    CHECK(ctx, rt.status().featureNegotiationState == SlimeVRFeatureNegotiationState::Waiting);
    CHECK(ctx, !rt.status().serverFeatureFlagsAvailable);

    // Server FeatureFlags bit 0 explicitly negotiates packet-100 bundles.
    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags), {0x01}
    );
    udp.incomingRemote = UdpEndpoint{selectedServerIp, 6969};
    udp.incomingPending = true;
    snapshots.snapshot.sequence = 2u;
    snapshots.snapshot.runtimeSample = 124u;
    snapshots.snapshot.publishedAtMcuUs = 1240500u;
    const size_t sentBeforeBundle = udp.sent.size();
    rt.update(1241);
    st = rt.status();
    CHECK(ctx, st.featureFlagsReceived == 2u);
    CHECK(ctx, st.serverFeatureFlagsAvailable);
    CHECK(ctx, st.featureNegotiationState == SlimeVRFeatureNegotiationState::Negotiated);
    CHECK(ctx, st.serverBundleSupported);
    CHECK(ctx, !st.serverCompactBundleSupported);
    CHECK(ctx, st.motionPacketMode == SlimeVRMotionPacketMode::Bundle100Rotation17Accel4);
    CHECK(ctx, st.bundledMotionEnabled);
    CHECK(ctx, st.bundledMotionSent == 1u);
    CHECK(ctx, st.rotationSent == 2u);
    CHECK(ctx, st.accelerationSent == 2u);
    CHECK(ctx, udp.sent.size() == sentBeforeBundle + 1u);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::Bundle));
    CHECK(ctx, udp.sent.back().data.size() == 56u);
    CHECK(ctx, udp.sent.back().data[17] == static_cast<uint8_t>(SlimeVRSendPacketType::RotationData));
    CHECK(ctx, udp.sent.back().data[42] == static_cast<uint8_t>(SlimeVRSendPacketType::Accel));

    // Hard-invalid acceleration falls back to a normal rotation packet.
    snapshots.snapshot.sequence = 3u;
    snapshots.snapshot.runtimeSample = 125u;
    snapshots.snapshot.publishedAtMcuUs = 1250500u;
    snapshots.snapshot.linearAccelerationValid = false;
    snapshots.snapshot.linearAccelerationInvalidFlags =
        prepared_output_motion_flags::ACCEL_COMPONENT_MISSING |
        prepared_output_motion_flags::PAIR_COHERENCY_DEGRADED;
    const size_t sentBeforeInvalidAcceleration = udp.sent.size();
    rt.update(1251);
    CHECK(ctx, rt.status().rotationSent == 3u);
    CHECK(ctx, rt.status().accelerationSent == 2u);
    CHECK(ctx, rt.status().accelerationSkippedInvalid == 1u);
    CHECK(ctx, rt.status().accelerationSkippedComponentMissing == 1u);
    CHECK(ctx, rt.status().accelerationSkippedPairDegraded == 1u);
    CHECK(ctx, udp.sent.size() == sentBeforeInvalidAcceleration + 1u);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::RotationData));

    // A failed bundle is one physical datagram failure and two logical motion
    // delivery failures. Packet 23 stays compiled but disabled by default.
    snapshots.snapshot.sequence = 4u;
    snapshots.snapshot.publishedAtMcuUs = 1260500u;
    snapshots.snapshot.linearAccelerationValid = true;
    snapshots.snapshot.linearAccelerationInvalidFlags = prepared_output_motion_flags::NONE;
    udp.failPacketType = static_cast<int>(SlimeVRSendPacketType::Bundle);
    const size_t sentBeforeBundleFailure = udp.sent.size();
    rt.update(1261);
    udp.failPacketType = -1;
    CHECK(ctx, rt.status().rotationSent == 3u);
    CHECK(ctx, rt.status().accelerationSent == 2u);
    CHECK(ctx, rt.status().bundledMotionSendFailures == 1u);
    CHECK(ctx, rt.status().compactMotionSendFailures == 0u);
    CHECK(ctx, rt.status().rotationSendFailures == 1u);
    CHECK(ctx, rt.status().accelerationSendFailures == 1u);
    CHECK(ctx, udp.sent.size() == sentBeforeBundleFailure);
    snapshots.snapshot.sequence = 3u;

    cfg.hasCompletedRestCalibration = true;
    rt.configure(cfg);
    const size_t sentBeforeRestRefresh = udp.sent.size();
    rt.update(1270);
    CHECK(ctx, rt.status().hasCompletedRestCalibration);
    CHECK(ctx, rt.status().sensorInfoSent == 2u);
    CHECK(ctx, udp.sent.size() == sentBeforeRestRefresh + 1u);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::SensorInfo));
    CHECK(ctx, udp.sent.back().data[17] == 1u);
    CHECK(ctx, rt.status().sensorInfoSyncState == SlimeVRSensorInfoSyncState::WaitingForAck);

    udp.incoming = makeSensorInfoAck(
        cfg.sensorId, static_cast<uint8_t>(SlimeVRSensorState::Online)
    );
    udp.incomingRemote = UdpEndpoint{selectedServerIp, 6969};
    udp.incomingPending = true;
    rt.update(1275);
    CHECK(ctx, rt.status().sensorInfoAckReceived == 3u);
    CHECK(ctx, rt.status().sensorInfoSyncState == SlimeVRSensorInfoSyncState::Acknowledged);
    CHECK(ctx, rt.status().sensorInfoAckRestCalibration);

    const uint32_t sensorInfoSentAfterAck = rt.status().sensorInfoSent;
    rt.update(2300);
    CHECK(ctx, rt.status().sensorInfoSent == sensorInfoSentAfterAck);

    rt.update(6101);
    CHECK(ctx, rt.status().heartbeatSent >= 1);
    CHECK(ctx, rt.status().signalStrengthSent >= 1);
    CHECK(ctx, rt.status().temperatureSent >= 1);
    CHECK(ctx, rt.status().batterySent >= 1);
    CHECK(ctx, rt.status().batterySendFailures == 0);
    CHECK(ctx, rt.status().lastSignalStrengthDbm == -68);
    CHECK(ctx, rt.status().lastRssiDbm == -68);
    bool sawSignalStrength = false;
    for (const auto& sent : udp.sent) {
        if (sent.data.size() >= 14u &&
            sent.data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::SignalStrength)) {
            sawSignalStrength = true;
            CHECK(ctx, sent.data[12] == cfg.sensorId);
            CHECK(ctx, sent.data[13] == static_cast<uint8_t>(static_cast<int8_t>(-68)));
        }
    }
    CHECK(ctx, sawSignalStrength);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::BatteryLevel));
    CHECK_NEAR(ctx, readF32BeLocal(udp.sent.back().data.data() + 12), 3.80f, 1.0e-6f);
    CHECK_NEAR(ctx, readF32BeLocal(udp.sent.back().data.data() + 16), 0.55f, 1.0e-6f);

    udp.incoming = makeServerPacket(static_cast<uint8_t>(SlimeVRReceivePacketType::HeartBeat0), {});
    udp.incomingRemote = UdpEndpoint{0xC0A80001UL, 6969};
    udp.incomingPending = true;
    rt.update(6110);
    CHECK(ctx, rt.status().heartbeatReceived == 1);

    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::PingPong),
        {0x11, 0x22, 0x33, 0x44}
    );
    udp.incomingRemote = UdpEndpoint{0xC0A80001UL, 6969};
    udp.incomingPending = true;
    const size_t sentBeforePing = udp.sent.size();
    rt.update(6120);
    CHECK(ctx, rt.status().pingReceived == 1);
    CHECK(ctx, rt.status().pongSent == 1);
    CHECK(ctx, rt.status().lastPingId == 0x11223344u);
    CHECK(ctx, udp.sent.size() == sentBeforePing + 1u);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::PingPong));
    CHECK(ctx, udp.sent.back().data[12] == 0x11);
    CHECK(ctx, udp.sent.back().data[15] == 0x44);

    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags),
        {0x03}
    );
    udp.incomingPending = true;
    rt.update(6130);
    CHECK(ctx, rt.status().featureFlagsReceived == 3u);
    CHECK(ctx, rt.status().lastServerFeatureFlags == 0x03u);
    CHECK(ctx, rt.status().serverBundleSupported);
    CHECK(ctx, rt.status().serverCompactBundleSupported);
    CHECK(ctx, !rt.status().compactMotionEnabled);
    CHECK(ctx, rt.status().motionPacketMode == SlimeVRMotionPacketMode::Bundle100Rotation17Accel4);

    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::SetConfigFlag),
        {2, 0x00, 0x01, 0x00}
    );
    udp.incomingPending = true;
    const size_t sentBeforeConfig = udp.sent.size();
    rt.update(6140);
    SlimeVROutputRuntimeStatus afterConfig = rt.status();
    CHECK(ctx, configFlagSink.calls == 1);
    CHECK(ctx, configFlagSink.sensorId == 2);
    CHECK(ctx, configFlagSink.configType == SLIMEVR_CONFIG_TYPE_MAGNETOMETER);
    CHECK(ctx, !configFlagSink.enabled);
    CHECK(ctx, afterConfig.setConfigFlagReceived == 1);
    CHECK(ctx, afterConfig.setConfigFlagApplied == 1);
    CHECK(ctx, afterConfig.ackConfigSent == 1);
    CHECK(ctx, afterConfig.magSupportEnabled);
    CHECK(ctx, !afterConfig.magEnabled);
    CHECK(ctx, afterConfig.sensorConfig == SLIMEVR_SENSOR_CONFIG_MAG_SUPPORTED);
    CHECK(ctx, afterConfig.lastSetConfigType == SLIMEVR_CONFIG_TYPE_MAGNETOMETER);
    CHECK(ctx, !afterConfig.lastSetConfigState);
    CHECK(ctx, afterConfig.lastSetConfigApplied);
    CHECK(ctx, udp.sent.size() == sentBeforeConfig + 1u); // Ack now; SensorInfo refresh is bounded.
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::AcknowledgeConfigChange));
    CHECK(ctx, udp.sent.back().data[12] == 2);
    CHECK(ctx, udp.sent.back().data[13] == 0x00);
    CHECK(ctx, udp.sent.back().data[14] == 0x01);
    CHECK(ctx, afterConfig.sensorInfoSyncState == SlimeVRSensorInfoSyncState::Dirty);

    // A command for another sensor is tolerated but never applied or ACKed.
    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::SetConfigFlag),
        {9, 0x00, 0x01, 0x01}
    );
    udp.incomingPending = true;
    const size_t sentBeforeIgnoredConfig = udp.sent.size();
    rt.update(6145);
    CHECK(ctx, rt.status().setConfigFlagIgnored == 1u);
    CHECK(ctx, configFlagSink.calls == 1u);
    CHECK(ctx, udp.sent.size() == sentBeforeIgnoredConfig);

    // Apply/persist failure is visible and must not emit packet 24.
    configFlagSink.result = false;
    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::SetConfigFlag),
        {2, 0x00, 0x01, 0x01}
    );
    udp.incomingPending = true;
    const size_t sentBeforeFailedConfig = udp.sent.size();
    rt.update(6150);
    CHECK(ctx, rt.status().setConfigFlagApplyFailures == 1u);
    CHECK(ctx, rt.status().ackConfigSent == 1u);
    CHECK(ctx, udp.sent.size() == sentBeforeFailedConfig);
    configFlagSink.result = true;

    // Unknown config types are tolerated without invoking the persistence hook.
    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::SetConfigFlag),
        {2, 0x12, 0x34, 0x01}
    );
    udp.incomingPending = true;
    rt.update(6155);
    CHECK(ctx, rt.status().setConfigFlagIgnored == 2u);
    CHECK(ctx, configFlagSink.calls == 2u);

    // A repeated already-applied state is ACKed without another persistent apply.
    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::SetConfigFlag),
        {2, 0x00, 0x01, 0x00}
    );
    udp.incomingPending = true;
    const size_t sentBeforeIdempotentConfig = udp.sent.size();
    rt.update(6160);
    CHECK(ctx, configFlagSink.calls == 2u);
    CHECK(ctx, rt.status().ackConfigSent == 2u);
    CHECK(ctx, udp.sent.size() == sentBeforeIdempotentConfig + 1u);

    // Malformed control packets do not refresh liveness.
    const uint32_t incomingBeforeMalformed = rt.status().lastIncomingPacketMs;
    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::PingPong), {0x11, 0x22}
    );
    udp.incomingPending = true;
    rt.update(6165);
    CHECK(ctx, rt.status().malformedPing == 1u);
    CHECK(ctx, rt.status().lastIncomingPacketMs == incomingBeforeMalformed);

    // Headerless legacy control packets are malformed in the modern session
    // contract and cannot refresh liveness or invoke handlers.
    udp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::HeartBeat)
    };
    udp.incomingPending = true;
    rt.update(6170);
    CHECK(ctx, rt.status().malformedUnknownRaw == 1u);
    CHECK(ctx, rt.status().heartbeatReceived == 1u);
    CHECK(ctx, rt.status().lastIncomingPacketMs == incomingBeforeMalformed);

    // A datagram larger than the fixed RX buffer must not be interpreted from
    // its truncated prefix.
    udp.incoming.assign(600u, 0u);
    udp.incoming[3] = static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags);
    udp.incomingPending = true;
    rt.update(6175);
    CHECK(ctx, rt.status().malformedDatagramLength == 1u);
    CHECK(ctx, rt.status().featureFlagsReceived == 3u);
    CHECK(ctx, rt.status().lastIncomingPacketMs == incomingBeforeMalformed);

    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::ProtocolChange),
        {0x01, 0x13}
    );
    udp.incomingPending = true;
    rt.update(6180);
    CHECK(ctx, rt.status().protocolChangeReceived == 1);
    CHECK(ctx, rt.status().protocolChangeIgnored == 1);
    CHECK(ctx, rt.status().lastProtocolTarget == 1);
    CHECK(ctx, rt.status().lastProtocolVersion == 0x13);

    udp.incoming = makeServerPacket(0xFE, {});
    udp.incomingPending = true;
    rt.update(6185);
    CHECK(ctx, rt.status().unknownPacketsReceived == 1);
    CHECK(ctx, rt.status().lastUnknownPacketType == 0xFE);

    TrackerHealthState health;
    health.enterDegradedNoImu(TrackerHealthFaultCode::LsmInitFailed, "LSM6DSV init failed after retries");
    rt.setTrackerHealth(health.snapshot());
    const SlimeVROutputRuntimeStatus beforeFaultUpdate = rt.status();
    const size_t sentBeforeFaultUpdate = udp.sent.size();
    snapshots.snapshot.sequence = 99;
    rt.update(7200);
    const SlimeVROutputRuntimeStatus faultStatus = rt.status();
    CHECK(ctx, faultStatus.trackerErrorActive);
    CHECK(ctx, faultStatus.trackerDegradedNoImu);
    CHECK(ctx, faultStatus.trackerErrorCode == static_cast<uint8_t>(TrackerHealthFaultCode::LsmInitFailed));
    CHECK(ctx, std::strcmp(faultStatus.trackerErrorMessage, "LSM6DSV init failed after retries") == 0);
    CHECK(ctx, faultStatus.rotationSent == beforeFaultUpdate.rotationSent);
    CHECK(ctx, faultStatus.rotationSuppressedByError > beforeFaultUpdate.rotationSuppressedByError);
    CHECK(ctx, faultStatus.trackerErrorSent == beforeFaultUpdate.trackerErrorSent + 1u);

    bool sawOnlineSensorInfo = false;
    bool sawErrorPacket = false;
    for (size_t i = sentBeforeFaultUpdate; i < udp.sent.size(); ++i) {
        const auto& pkt = udp.sent[i].data;
        if (pkt.size() >= SLIMEVR_PACKET_HEADER_SIZE + 8u &&
            pkt[3] == static_cast<uint8_t>(SlimeVRSendPacketType::SensorInfo)) {
            sawOnlineSensorInfo = sawOnlineSensorInfo ||
                (pkt[12] == 2 && pkt[13] == static_cast<uint8_t>(SlimeVRSensorState::Online));
        }
        if (pkt.size() >= SLIMEVR_PACKET_HEADER_SIZE + 3u &&
            pkt[3] == static_cast<uint8_t>(SlimeVRSendPacketType::Error)) {
            sawErrorPacket = sawErrorPacket ||
                (pkt[12] == 2 && pkt[13] == static_cast<uint8_t>(TrackerHealthFaultCode::LsmInitFailed));
        }
    }
    CHECK(ctx, sawOnlineSensorInfo);
    CHECK(ctx, sawErrorPacket);

    // Scheduler phase-lock: a late update advances to the nearest future
    // deadline, records skipped periods, and still sends at most one current
    // snapshot instead of a catch-up burst.
    FakeUdp phaseUdp;
    FakeSnapshotSource phaseSnapshots;
    phaseSnapshots.snapshot.valid = true;
    phaseSnapshots.snapshot.sequence = 1u;
    phaseSnapshots.snapshot.q = Quat::identity();
    phaseSnapshots.snapshot.linearAccelerationValid = false;
    phaseSnapshots.snapshot.confidence = 1.0f;

    SlimeVROutputRuntime phaseRt;
    phaseRt.begin(phaseUdp, wifi, FakeSnapshotSource::copy, &phaseSnapshots);
    SlimeVROutputRuntimeConfig phaseCfg = cfg;
    phaseCfg.latestTemperatureValid = false;
    phaseCfg.latestBatteryValid = false;
    phaseCfg.signalTelemetryEnabled = false;
    phaseCfg.temperatureTelemetryEnabled = false;
    phaseCfg.batteryTelemetryEnabled = false;
    phaseRt.configure(phaseCfg);
    phaseRt.update(1u);

    phaseUdp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    phaseUdp.incomingRemote = UdpEndpoint{0xC0A80002UL, 6969};
    phaseUdp.incomingPending = true;
    phaseRt.update(100u); // Intentional send grace ends at 200 ms.

    phaseRt.update(201u);
    CHECK(ctx, phaseRt.status().rotationSent == 1u);
    CHECK(ctx, phaseRt.status().rotationMissedDeadlines == 0u);

    phaseSnapshots.snapshot.sequence = 2u;
    const size_t packetsBeforeLateTick = phaseUdp.sent.size();
    phaseRt.update(225u); // Deadlines 210 and 220 elapsed; send once.
    SlimeVROutputRuntimeStatus phaseStatus = phaseRt.status();
    CHECK(ctx, phaseStatus.rotationSent == 2u);
    CHECK(ctx, phaseStatus.rotationSendDue == 2u);
    CHECK(ctx, phaseStatus.rotationMissedDeadlines == 1u);
    CHECK(ctx, phaseStatus.rotationLateEvents == 2u);
    CHECK(ctx, phaseStatus.rotationLatenessMaxMs == 15u);
    CHECK(ctx, phaseUdp.sent.size() == packetsBeforeLateTick + 1u);

    phaseSnapshots.snapshot.sequence = 3u;
    phaseRt.update(226u);
    CHECK(ctx, phaseRt.status().rotationSent == 2u);
    phaseRt.update(230u);
    CHECK(ctx, phaseRt.status().rotationSent == 3u);
    CHECK(ctx, phaseRt.status().rotationMissedDeadlines == 1u);

    // Servers that do not answer FeatureFlags stay on packet 17 at the full
    // pose rate while coherent packet-4 acceleration is limited to 50 Hz.
    FakeUdp fallbackUdp;
    FakeSnapshotSource fallbackSnapshots;
    fallbackSnapshots.snapshot.valid = true;
    fallbackSnapshots.snapshot.sequence = 1u;
    fallbackSnapshots.snapshot.q = Quat::identity();
    fallbackSnapshots.snapshot.linearAccelerationValid = true;
    fallbackSnapshots.snapshot.linearAccelerationDeviceG = Vec3::zero();
    fallbackSnapshots.snapshot.confidence = 1.0f;

    SlimeVROutputRuntime fallbackRt;
    fallbackRt.begin(fallbackUdp, wifi, FakeSnapshotSource::copy, &fallbackSnapshots);
    fallbackRt.configure(phaseCfg);
    fallbackRt.update(1u);
    fallbackUdp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    fallbackUdp.incomingRemote = UdpEndpoint{0xC0A80004UL, 6969};
    fallbackUdp.incomingPending = true;
    fallbackRt.update(100u);
    fallbackRt.update(201u);
    CHECK(ctx, fallbackRt.status().motionPacketMode ==
               SlimeVRMotionPacketMode::SeparateRotation17Accel4);
    CHECK(ctx, fallbackRt.status().rotationSent == 1u);
    CHECK(ctx, fallbackRt.status().accelerationSent == 1u);

    fallbackSnapshots.snapshot.sequence = 2u;
    fallbackRt.update(211u);
    CHECK(ctx, fallbackRt.status().rotationSent == 2u);
    CHECK(ctx, fallbackRt.status().accelerationSent == 1u);
    CHECK(ctx, fallbackRt.status().accelerationRateLimited == 1u);

    fallbackSnapshots.snapshot.sequence = 3u;
    fallbackRt.update(221u);
    CHECK(ctx, fallbackRt.status().rotationSent == 3u);
    CHECK(ctx, fallbackRt.status().accelerationSent == 2u);

    for (uint8_t i = fallbackRt.status().featureFlagsRequestAttempts;
         i < TRACKER_SLIMEVR_FEATURE_FLAGS_REQUEST_ATTEMPTS; ++i) {
        const uint32_t now = 701u + static_cast<uint32_t>(i) * 500u;
        fallbackUdp.incoming = makeServerPacket(
            static_cast<uint8_t>(SlimeVRReceivePacketType::HeartBeat0), {}
        );
        fallbackUdp.incomingRemote = UdpEndpoint{0xC0A80004UL, 6969};
        fallbackUdp.incomingPending = true;
        fallbackSnapshots.snapshot.sequence += 1u;
        fallbackRt.update(now);
    }
    fallbackUdp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::HeartBeat0), {}
    );
    fallbackUdp.incomingRemote = UdpEndpoint{0xC0A80004UL, 6969};
    fallbackUdp.incomingPending = true;
    fallbackRt.update(9000u);
    CHECK(ctx, fallbackRt.status().featureFlagsRequestAttempts ==
               TRACKER_SLIMEVR_FEATURE_FLAGS_REQUEST_ATTEMPTS);
    CHECK(ctx, fallbackRt.status().featureNegotiationState ==
               SlimeVRFeatureNegotiationState::Unavailable);
    CHECK(ctx, fallbackRt.status().motionPacketMode ==
               SlimeVRMotionPacketMode::SeparateRotation17Accel4);

    // A recent inbound heartbeat/ping proves the server session is alive.
    // Sustained transient TX pressure must not reopen the UDP socket and add
    // a discovery/grace gap; stale pose datagrams are simply dropped.
    FakeUdp pressureUdp;
    FakeSnapshotSource pressureSnapshots;
    pressureSnapshots.snapshot.valid = true;
    pressureSnapshots.snapshot.sequence = 1u;
    pressureSnapshots.snapshot.q = Quat::identity();
    pressureSnapshots.snapshot.linearAccelerationValid = true;
    pressureSnapshots.snapshot.linearAccelerationDeviceG = Vec3::zero();
    pressureSnapshots.snapshot.confidence = 1.0f;

    SlimeVROutputRuntime pressureRt;
    pressureRt.begin(pressureUdp, wifi, FakeSnapshotSource::copy, &pressureSnapshots);
    pressureRt.configure(phaseCfg);
    pressureRt.update(1u);
    pressureUdp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    pressureUdp.incomingRemote = UdpEndpoint{0xC0A80003UL, 6969};
    pressureUdp.incomingPending = true;
    pressureRt.update(100u);
    pressureUdp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags), {0x01}
    );
    pressureUdp.incomingPending = true;
    pressureRt.update(110u);
    CHECK(ctx, pressureRt.status().motionPacketMode ==
               SlimeVRMotionPacketMode::Bundle100Rotation17Accel4);

    pressureUdp.failPacketType = static_cast<int>(SlimeVRSendPacketType::Bundle);
    for (uint32_t i = 0; i < TRACKER_SLIMEVR_SEND_FAILURE_REOPEN_THRESHOLD; ++i) {
        pressureSnapshots.snapshot.sequence = i + 2u;
        pressureRt.update(201u + i * 10u);
    }
    SlimeVROutputRuntimeStatus pressureStatus = pressureRt.status();
    CHECK(ctx, pressureStatus.bundledMotionSendFailures ==
               TRACKER_SLIMEVR_SEND_FAILURE_REOPEN_THRESHOLD);
    CHECK(ctx, pressureStatus.compactMotionSendFailures == 0u);
    CHECK(ctx, pressureStatus.udpReopenRequests == 0u);
    CHECK(ctx, pressureStatus.udpReopenSuppressedRecentRx == 1u);
    CHECK(ctx, pressureUdp.active());
    CHECK(ctx, pressureUdp.stopCalls == 0u);

    pressureUdp.failPacketType = -1;
    pressureSnapshots.snapshot.sequence += 1u;
    pressureRt.update(401u);
    CHECK(ctx, pressureRt.status().bundledMotionSent == 1u);
    CHECK(ctx, pressureRt.status().rotationSent == 1u);
    CHECK(ctx, pressureRt.status().accelerationSent == 1u);

    // Once inbound server activity is genuinely stale, the same sustained
    // transport failure must still request a socket reopen.
    pressureUdp.failPacketType = static_cast<int>(SlimeVRSendPacketType::Bundle);
    for (uint32_t i = 0; i < TRACKER_SLIMEVR_SEND_FAILURE_REOPEN_THRESHOLD; ++i) {
        pressureSnapshots.snapshot.sequence += 1u;
        pressureRt.update(3000u + i * 10u);
    }
    pressureRt.update(3210u);
    CHECK(ctx, pressureRt.status().udpReopenRequests == 1u);
    CHECK(ctx, !pressureRt.status().serverFound);
    CHECK(ctx, !pressureRt.status().serverFeatureFlagsAvailable);
    CHECK(ctx, pressureRt.status().motionPacketMode ==
               SlimeVRMotionPacketMode::SeparateRotation17Accel4);
    CHECK(ctx, pressureUdp.stopCalls == 1u);


    // Manual server mode resolves the configured hostname at a bounded rate,
    // sends a unicast handshake, and rejects discovery responses from every
    // other endpoint when broadcast discovery is disabled.
    FakeUdp manualUdp;
    FakeSnapshotSource manualSnapshots;
    manualSnapshots.snapshot.valid = true;
    manualSnapshots.snapshot.sequence = 1u;
    manualSnapshots.snapshot.q = Quat::identity();
    manualSnapshots.snapshot.linearAccelerationValid = true;
    manualSnapshots.snapshot.linearAccelerationDeviceG = Vec3::zero();
    manualSnapshots.snapshot.confidence = 1.0f;

    SlimeVROutputRuntime manualRt;
    manualRt.begin(manualUdp, wifi, FakeSnapshotSource::copy, &manualSnapshots);
    SlimeVROutputRuntimeConfig manualCfg = phaseCfg;
    manualCfg.discoveryEnabled = false;
    manualCfg.manualServerEnabled = true;
    manualCfg.manualServerHost = "slimevr.local";
    manualCfg.serverPort = 6969u;
    manualCfg.discoveryIntervalMs = 1000u;
    manualRt.configure(manualCfg);
    manualRt.update(1000u);
    CHECK(ctx, manualUdp.resolveCalls == 1u);
    CHECK(ctx, std::strcmp(manualUdp.lastResolvedHost, "slimevr.local") == 0);
    CHECK(ctx, manualUdp.sent.size() == 1u);
    const UdpEndpoint expectedManualEndpoint{manualUdp.resolvedIpv4, 6969u};
    CHECK(ctx, manualUdp.sent.back().endpoint == expectedManualEndpoint);
    CHECK(ctx, manualRt.status().manualServerResolved);
    CHECK(ctx, manualRt.status().manualServerResolveAttempts == 1u);
    CHECK(ctx, manualRt.status().manualServerResolveFailures == 0u);
    CHECK(ctx, manualRt.status().manualServerHandshakesSent == 1u);

    manualUdp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    manualUdp.incomingRemote = UdpEndpoint{0xC0A80063UL, 6969u};
    manualUdp.incomingPending = true;
    manualRt.update(1010u);
    CHECK(ctx, !manualRt.status().serverFound);
    CHECK(ctx, manualRt.status().foreignEndpointPacketsDropped == 1u);

    manualUdp.incomingRemote = UdpEndpoint{manualUdp.resolvedIpv4, 6969u};
    manualUdp.incomingPending = true;
    manualRt.update(1020u);
    CHECK(ctx, manualRt.status().serverFound);
    CHECK(ctx, manualRt.status().serverIpv4 == manualUdp.resolvedIpv4);

    FakeUdp failedResolveUdp;
    failedResolveUdp.resolveResult = false;
    SlimeVROutputRuntime failedResolveRt;
    failedResolveRt.begin(failedResolveUdp, wifi, FakeSnapshotSource::copy, &manualSnapshots);
    failedResolveRt.configure(manualCfg);
    failedResolveRt.update(1000u);
    failedResolveRt.update(1100u);
    CHECK(ctx, failedResolveUdp.resolveCalls == 1u); // bounded by discovery interval
    CHECK(ctx, failedResolveRt.status().manualServerResolveFailures == 1u);
    failedResolveRt.update(2000u);
    CHECK(ctx, failedResolveUdp.resolveCalls == 2u);
    CHECK(ctx, failedResolveRt.status().manualServerResolveFailures == 2u);

    return ctx.finish("slimevr_output_runtime");
}

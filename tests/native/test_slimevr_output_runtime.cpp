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

    udp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    udp.incomingRemote = UdpEndpoint{0xC0A80001UL, 6969};
    udp.incomingPending = true;
    rt.update(1100);

    {
        const SlimeVROutputRuntimeStatus st = rt.status();
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
    const SlimeVROutputRuntimeStatus st = rt.status();
    CHECK(ctx, st.sensorInfoSent == 1);
    CHECK(ctx, st.rotationSent == 1);
    CHECK(ctx, st.accelerationSent == 1);
    CHECK(ctx, st.accelerationSkippedInvalid == 0);
    CHECK(ctx, st.accelerationSendFailures == 0);
    CHECK(ctx, st.signalStrengthSent == 0);
    CHECK(ctx, st.temperatureSent == 0);
    CHECK(ctx, st.magnetometerAccuracySent == 0);
    CHECK(ctx, st.magSupportEnabled);
    CHECK(ctx, st.sensorConfig == SLIMEVR_SENSOR_CONFIG_MAG_SUPPORTED_AND_ENABLED);
    CHECK(ctx, st.lastTemperatureValid);
    CHECK_NEAR(ctx, st.lastTemperatureC, 42.5f, 1.0e-6f);
    CHECK(ctx, st.batteryTelemetryEnabled);
    CHECK(ctx, st.lastBatteryValid);
    CHECK_NEAR(ctx, st.lastBatteryVoltage, 3.80f, 1.0e-6f);
    CHECK_NEAR(ctx, st.lastBatteryPercentage, 55.0f, 1.0e-6f);
    CHECK(ctx, !st.hasCompletedRestCalibration);
    CHECK(ctx, st.rotationNoSnapshot == 0);
    CHECK(ctx, st.rotationDuplicateSnapshot == 0);
    CHECK(ctx, st.lastRotationSnapshotSequence == 1);
    CHECK(ctx, st.lastRotationRuntimeSample == 123);
    CHECK(ctx, st.lastRotationTimestampUs == 456789ULL);
    CHECK(ctx, st.lastRotationQualityFlags == 0x1234u);
    CHECK(ctx, st.lastRotationSnapshotAgeUs == 1500u);
    CHECK_NEAR(ctx, st.lastRotationConfidence, 0.99f, 1.0e-6f);
    CHECK(ctx, udp.sent.size() >= 3u);
    CHECK(ctx, udp.sent[1].endpoint.ipv4 == 0xC0A80001UL);
    CHECK(ctx, udp.sent[1].data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::SensorInfo));
    CHECK(ctx, udp.sent[1].data[17] == 0);
    CHECK(ctx, udp.sent.back().endpoint.ipv4 == 0xC0A80001UL);
    CHECK(ctx, udp.sent[udp.sent.size() - 2u].data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::RotationData));
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::Accel));
    CHECK(ctx, udp.sent.back().data.size() == SLIMEVR_PACKET_HEADER_SIZE + 13u);
    CHECK_NEAR(ctx, readF32BeLocal(udp.sent.back().data.data() + 12), 0.25f * 9.80665f, 1.0e-5f);
    CHECK_NEAR(ctx, readF32BeLocal(udp.sent.back().data.data() + 16), -0.5f * 9.80665f, 1.0e-5f);
    CHECK_NEAR(ctx, readF32BeLocal(udp.sent.back().data.data() + 20), 1.75f * 9.80665f, 1.0e-5f);
    CHECK(ctx, udp.sent.back().data[24] == cfg.sensorId);

    const size_t sentBeforeTap = udp.sent.size();
    CHECK(ctx, rt.sendTap(2));
    CHECK(ctx, rt.status().tapSent == 1);
    CHECK(ctx, rt.status().tapSendFailures == 0);
    CHECK(ctx, rt.status().lastTapValue == 2);
    CHECK(ctx, udp.sent.size() == sentBeforeTap + 1u);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::Tap));
    CHECK(ctx, udp.sent.back().data[12] == 2);
    CHECK(ctx, udp.sent.back().data[13] == 2);

    rt.update(1205);
    CHECK(ctx, rt.status().rotationSent == 1);

    snapshots.snapshot.sequence = 2;
    snapshots.snapshot.runtimeSample = 124;
    snapshots.snapshot.publishedAtMcuUs = 1210500u;
    snapshots.snapshot.linearAccelerationValid = false;
    snapshots.snapshot.linearAccelerationInvalidFlags =
        prepared_output_motion_flags::ACCEL_COMPONENT_MISSING |
        prepared_output_motion_flags::PAIR_COHERENCY_DEGRADED;
    const size_t sentBeforeInvalidAcceleration = udp.sent.size();
    rt.update(1211);
    CHECK(ctx, rt.status().rotationSent == 2);
    CHECK(ctx, rt.status().accelerationSent == 1);
    CHECK(ctx, rt.status().accelerationSkippedInvalid == 1);
    CHECK(ctx, rt.status().accelerationSkippedComponentMissing == 1);
    CHECK(ctx, rt.status().accelerationSkippedPairDegraded == 1);
    CHECK(ctx, rt.status().accelerationSkippedConfiguration == 0);
    CHECK(ctx, rt.status().accelerationSkippedSaturated == 0);
    CHECK(ctx, rt.status().accelerationSkippedNonFinite == 0);
    CHECK(ctx, rt.status().accelerationSkippedOther == 0);
    CHECK(ctx, rt.status().lastRotationSnapshotSequence == 2);
    CHECK(ctx, udp.sent.size() == sentBeforeInvalidAcceleration + 1u);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::RotationData));

    snapshots.snapshot.sequence = 3;
    snapshots.snapshot.publishedAtMcuUs = 1220500u;
    snapshots.snapshot.linearAccelerationValid = true;
    snapshots.snapshot.linearAccelerationInvalidFlags = prepared_output_motion_flags::NONE;
    udp.failPacketType = static_cast<int>(SlimeVRSendPacketType::Accel);
    const size_t sentBeforeAccelerationFailure = udp.sent.size();
    rt.update(1221);
    udp.failPacketType = -1;
    CHECK(ctx, rt.status().rotationSent == 3);
    CHECK(ctx, rt.status().accelerationSent == 1);
    CHECK(ctx, rt.status().accelerationSendFailures == 1);
    CHECK(ctx, udp.sent.size() == sentBeforeAccelerationFailure + 1u);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::RotationData));

    cfg.hasCompletedRestCalibration = true;
    rt.configure(cfg);
    const size_t sentBeforeRestRefresh = udp.sent.size();
    rt.update(1230);
    CHECK(ctx, rt.status().hasCompletedRestCalibration);
    CHECK(ctx, rt.status().sensorInfoSent == 2);
    CHECK(ctx, udp.sent.size() == sentBeforeRestRefresh + 1u);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::SensorInfo));
    CHECK(ctx, udp.sent.back().data[17] == 1);

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
        {0xAA, 0xBB, 0xCC, 0xDD}
    );
    udp.incomingPending = true;
    rt.update(6130);
    CHECK(ctx, rt.status().featureFlagsReceived == 1);
    CHECK(ctx, rt.status().lastServerFeatureFlags == 0xAABBCCDDu);

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
    CHECK(ctx, udp.sent.size() >= sentBeforeConfig + 2u); // SensorInfo + AckConfigChange.
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::AcknowledgeConfigChange));
    CHECK(ctx, udp.sent.back().data[12] == 2);
    CHECK(ctx, udp.sent.back().data[13] == 0x00);
    CHECK(ctx, udp.sent.back().data[14] == 0x01);

    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::ProtocolChange),
        {0x01, 0x13}
    );
    udp.incomingPending = true;
    rt.update(6150);
    CHECK(ctx, rt.status().protocolChangeReceived == 1);
    CHECK(ctx, rt.status().lastProtocolTarget == 1);
    CHECK(ctx, rt.status().lastProtocolVersion == 0x13);

    udp.incoming = makeServerPacket(0xFE, {});
    udp.incomingPending = true;
    rt.update(6160);
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

    return ctx.finish("slimevr_output_runtime");
}

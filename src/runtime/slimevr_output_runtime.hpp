#pragma once

#include <cstddef>
#include <cstdint>

#include "network/udp_transport.hpp"
#include "network/wifi_manager.hpp"
#include "output/slimevr_packet_writer.hpp"
#include "runtime/tracker_runtime_types.hpp"

namespace tracker {

class TrackerNetworkConfig;

using SlimeVRCopyOutputSnapshotFn = bool (*)(TrackerPreparedOutputSnapshot& out, void* user);

constexpr uint16_t SLIMEVR_DISCOVERY_LOCAL_PORT = 6969;
constexpr uint32_t SLIMEVR_DISCOVERY_BROADCAST_IPV4 = 0xffffffffUL;

// SlimeVR UDP output runtime: Wi-Fi/UDP discovery, SensorInfo/heartbeat and
// non-blocking RotationData emission from prepared quaternion snapshots.
enum class SlimeVROutputState : uint8_t {
    Disabled,
    WaitingForWifi,
    UdpStarting,
    Discovering,
    ServerFound,
    Error,
};

const char* slimevrOutputStateName(SlimeVROutputState state);

struct SlimeVROutputRuntimeConfig {
    bool enabled = false;
    bool discoveryEnabled = true;
    bool manualServerEnabled = false;
    const char* deviceName = nullptr;
    uint8_t sensorId = 0;
    uint16_t serverPort = SLIMEVR_DEFAULT_SERVER_PORT;
    uint16_t localPort = SLIMEVR_DISCOVERY_LOCAL_PORT;
    uint32_t discoveryIntervalMs = 1000;
    uint16_t rotationRateHz = 100;
    uint8_t incomingPacketsPerUpdate = 4;
};

struct SlimeVROutputRuntimeStatus {
    SlimeVROutputState state = SlimeVROutputState::Disabled;
    bool enabled = false;
    bool wifiConnected = false;
    bool udpReady = false;
    bool serverFound = false;
    bool discoveryEnabled = true;
    bool manualServerEnabled = false;

    uint16_t localPort = 0;
    uint32_t serverIpv4 = 0;
    uint16_t serverPort = 0;
    uint8_t sensorId = 0;
    uint8_t protocolVersion = SLIMEVR_PROTOCOL_VERSION;
    uint32_t boardType = 10;
    uint32_t imuType = 13;
    uint32_t mcuType = 6;

    uint32_t handshakesSent = 0;
    uint32_t heartbeatSent = 0;
    uint32_t sensorInfoSent = 0;
    uint32_t rotationSent = 0;
    uint32_t rotationNoSnapshot = 0;
    uint32_t rotationDuplicateSnapshot = 0;
    uint32_t packetsReceived = 0;
    uint32_t discoveryResponses = 0;
    uint32_t sendFailures = 0;
    uint32_t udpBeginFailures = 0;

    uint32_t nextPacketNumber = 0;
    uint16_t rotationRateHz = 0;
    bool preparedOutputAvailable = false;
    uint32_t lastHandshakeMs = 0;
    uint32_t lastIncomingPacketMs = 0;
    uint32_t lastStateChangeMs = 0;
    uint32_t lastRotationMs = 0;
    uint32_t lastRotationSnapshotSequence = 0;
    uint32_t lastRotationRuntimeSample = 0;
    uint64_t lastRotationTimestampUs = 0;
    uint32_t lastRotationQualityFlags = 0;
    float lastRotationConfidence = 0.0f;
};

class SlimeVROutputRuntime {
public:
    void begin(IUdpTransport& udp,
               const TrackerWifiManager& wifi,
               SlimeVRCopyOutputSnapshotFn copyOutputSnapshot = nullptr,
               void* copyOutputSnapshotUser = nullptr);
    void configure(const SlimeVROutputRuntimeConfig& config);
    void resetCounters();
    void stop();
    void restart();
    void update(uint32_t nowMs);

    bool enabled() const { return enabled_; }
    bool serverFound() const { return serverFound_; }
    SlimeVROutputState state() const { return state_; }
    SlimeVROutputRuntimeStatus status() const;

private:
    void transitionTo(SlimeVROutputState state, uint32_t nowMs);
    void resetConnectionState(bool keepCounters);
    void ensureUdp(uint32_t nowMs);
    void pollIncoming(uint32_t nowMs);
    void handleIncomingPacket(const uint8_t* data, size_t len, const UdpEndpoint& remote, uint32_t nowMs);
    void maybeSendDiscovery(uint32_t nowMs);
    void sendHandshakeTo(const UdpEndpoint& endpoint, uint32_t nowMs);
    void sendSensorInfo(uint32_t nowMs);
    void sendHeartbeat(uint32_t nowMs);
    void maybeSendRotation(uint32_t nowMs);
    void sendRotation(const TrackerPreparedOutputSnapshot& snapshot, uint32_t nowMs);
    void makeHandshakeInfo(SlimeVRHandshakeInfo& info) const;
    bool sendPacket(const SlimeVRPacketWriteResult& packet, const UdpEndpoint& endpoint);
    uint32_t rotationPeriodMs() const;
    static uint8_t accuracyFromConfidence(float confidence);

    IUdpTransport* udp_ = nullptr;
    const TrackerWifiManager* wifi_ = nullptr;
    SlimeVRCopyOutputSnapshotFn copyOutputSnapshot_ = nullptr;
    void* copyOutputSnapshotUser_ = nullptr;

    SlimeVRPacketWriter writer_;
    uint8_t packetBuffer_[512] = {};
    uint8_t incomingBuffer_[512] = {};

    SlimeVROutputState state_ = SlimeVROutputState::Disabled;
    bool enabled_ = false;
    bool discoveryEnabled_ = true;
    bool manualServerEnabled_ = false;
    bool serverFound_ = false;

    char deviceName_[32] = "c3-6dsv-tracker";
    uint8_t sensorId_ = 0;
    uint16_t serverPort_ = SLIMEVR_DEFAULT_SERVER_PORT;
    uint16_t localPort_ = SLIMEVR_DISCOVERY_LOCAL_PORT;
    uint32_t discoveryIntervalMs_ = 1000;
    uint16_t rotationRateHz_ = 100;
    uint8_t incomingPacketsPerUpdate_ = 4;

    UdpEndpoint serverEndpoint_;

    uint32_t handshakesSent_ = 0;
    uint32_t heartbeatSent_ = 0;
    uint32_t sensorInfoSent_ = 0;
    uint32_t rotationSent_ = 0;
    uint32_t rotationNoSnapshot_ = 0;
    uint32_t rotationDuplicateSnapshot_ = 0;
    uint32_t packetsReceived_ = 0;
    uint32_t discoveryResponses_ = 0;
    uint32_t sendFailures_ = 0;
    uint32_t udpBeginFailures_ = 0;

    uint32_t lastHandshakeMs_ = 0;
    uint32_t lastIncomingPacketMs_ = 0;
    uint32_t lastStateChangeMs_ = 0;
    uint32_t lastHeartbeatMs_ = 0;
    uint32_t lastSensorInfoMs_ = 0;
    uint32_t lastRotationAttemptMs_ = 0;
    uint32_t lastRotationMs_ = 0;
    uint32_t lastRotationSnapshotSequence_ = 0;
    uint32_t lastRotationRuntimeSample_ = 0;
    uint64_t lastRotationTimestampUs_ = 0;
    uint32_t lastRotationQualityFlags_ = 0;
    float lastRotationConfidence_ = 0.0f;
    uint32_t nextUdpBeginRetryMs_ = 0;
};

} // namespace tracker

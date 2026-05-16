#pragma once

#include <cstddef>
#include <cstdint>

#include "defines.h"
#include "network/udp_transport.hpp"
#include "network/wifi_manager.hpp"
#include "output/slimevr_packet_writer.hpp"
#include "runtime/tracker_runtime_types.hpp"

namespace tracker {

class TrackerNetworkConfig;

using SlimeVRCopyOutputSnapshotFn = bool (*)(TrackerPreparedOutputSnapshot& out, void* user);
using SlimeVRSetConfigFlagFn = bool (*)(uint8_t sensorId, uint16_t configType, bool enabled, void* user);

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

    // magSupported means the firmware can expose a magnetometer-backed yaw
    // path. magEnabled means the server/runtime currently allows using it.
    bool magSupportEnabled = false; // compatibility field: true means supported.
    bool magEnabled = false;
    SlimeVRSetConfigFlagFn setConfigFlag = nullptr;
    void* setConfigFlagUser = nullptr;
    bool signalTelemetryEnabled = TRACKER_SLIMEVR_ENABLE_SIGNAL_TELEMETRY != 0;
    bool temperatureTelemetryEnabled = TRACKER_SLIMEVR_ENABLE_TEMPERATURE_TELEMETRY != 0;
    uint32_t telemetryIntervalMs = TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS;
    bool latestTemperatureValid = false;
    float latestTemperatureC = 0.0f;
    bool hasCompletedRestCalibration = false;
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
    uint32_t signalStrengthSent = 0;
    uint32_t temperatureSent = 0;
    uint32_t magnetometerAccuracySent = 0;
    uint32_t rotationNoSnapshot = 0;
    uint32_t rotationDuplicateSnapshot = 0;
    uint32_t packetsReceived = 0;
    uint32_t discoveryResponses = 0;
    uint32_t heartbeatReceived = 0;
    uint32_t pingReceived = 0;
    uint32_t pongSent = 0;
    uint32_t featureFlagsReceived = 0;
    uint32_t setConfigFlagReceived = 0;
    uint32_t setConfigFlagApplied = 0;
    uint32_t setConfigFlagIgnored = 0;
    uint32_t ackConfigSent = 0;
    uint32_t protocolChangeReceived = 0;
    uint32_t unknownPacketsReceived = 0;
    uint32_t sendFailures = 0;
    uint32_t udpBeginFailures = 0;
    uint32_t serverSilenceResets = 0;
    uint32_t wifiLostResets = 0;
    uint32_t udpReopenRequests = 0;
    uint32_t consecutiveSendFailures = 0;

    uint32_t nextPacketNumber = 0;
    uint16_t rotationRateHz = 0;
    bool preparedOutputAvailable = false;
    bool magSupportEnabled = false;
    bool magEnabled = false;
    uint16_t sensorConfig = 0;
    bool signalTelemetryEnabled = false;
    bool temperatureTelemetryEnabled = false;
    uint32_t telemetryIntervalMs = 0;
    uint8_t lastSignalStrength = 0;
    int32_t lastRssiDbm = 0;
    float lastTemperatureC = 0.0f;
    bool lastTemperatureValid = false;
    bool hasCompletedRestCalibration = false;
    uint32_t lastPingId = 0;
    uint32_t lastServerFeatureFlags = 0;
    uint8_t lastSetConfigSensorId = 0;
    uint16_t lastSetConfigType = 0;
    bool lastSetConfigState = false;
    bool lastSetConfigApplied = false;
    uint8_t lastProtocolTarget = 0;
    uint8_t lastProtocolVersion = 0;
    uint8_t lastUnknownPacketType = 0;
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
    void requestSensorInfoRefresh();

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
    void handlePingPong(const uint8_t* data, size_t len);
    void handleFeatureFlags(const uint8_t* data, size_t len);
    void handleSetConfigFlag(const uint8_t* data, size_t len, uint32_t nowMs);
    void handleProtocolChange(const uint8_t* data, size_t len);
    void sendAckConfigChange(uint16_t configType);
    void maybeSendDiscovery(uint32_t nowMs);
    void sendHandshakeTo(const UdpEndpoint& endpoint, uint32_t nowMs);
    void sendSensorInfo(uint32_t nowMs);
    void sendHeartbeat(uint32_t nowMs);
    void maybeSendTelemetry(uint32_t nowMs);
    void sendSignalStrength(uint32_t nowMs);
    void sendTemperature(uint32_t nowMs);
    void sendMagnetometerAccuracy(uint32_t nowMs);
    void maybeSendRotation(uint32_t nowMs);
    void sendRotation(const TrackerPreparedOutputSnapshot& snapshot, uint32_t nowMs);
    void makeHandshakeInfo(SlimeVRHandshakeInfo& info) const;
    bool sendPacket(const SlimeVRPacketWriteResult& packet, const UdpEndpoint& endpoint);
    uint32_t rotationPeriodMs() const;
    uint16_t sensorConfigFlags() const;
    static uint8_t accuracyFromConfidence(float confidence);
    static uint8_t signalStrengthFromRssi(int32_t rssiDbm);

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

    bool magSupportEnabled_ = false;
    bool magEnabled_ = false;
    SlimeVRSetConfigFlagFn setConfigFlag_ = nullptr;
    void* setConfigFlagUser_ = nullptr;
    bool signalTelemetryEnabled_ = TRACKER_SLIMEVR_ENABLE_SIGNAL_TELEMETRY != 0;
    bool temperatureTelemetryEnabled_ = TRACKER_SLIMEVR_ENABLE_TEMPERATURE_TELEMETRY != 0;
    uint32_t telemetryIntervalMs_ = TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS;
    bool latestTemperatureValid_ = false;
    float latestTemperatureC_ = 0.0f;
    bool hasCompletedRestCalibration_ = false;

    uint32_t lastPingId_ = 0;
    uint32_t lastServerFeatureFlags_ = 0;
    uint8_t lastSetConfigSensorId_ = 0;
    uint16_t lastSetConfigType_ = 0;
    bool lastSetConfigState_ = false;
    bool lastSetConfigApplied_ = false;
    uint8_t lastProtocolTarget_ = 0;
    uint8_t lastProtocolVersion_ = 0;
    uint8_t lastUnknownPacketType_ = 0;

    UdpEndpoint serverEndpoint_;

    uint32_t handshakesSent_ = 0;
    uint32_t heartbeatSent_ = 0;
    uint32_t sensorInfoSent_ = 0;
    uint32_t rotationSent_ = 0;
    uint32_t signalStrengthSent_ = 0;
    uint32_t temperatureSent_ = 0;
    uint32_t magnetometerAccuracySent_ = 0;
    uint32_t rotationNoSnapshot_ = 0;
    uint32_t rotationDuplicateSnapshot_ = 0;
    uint32_t packetsReceived_ = 0;
    uint32_t discoveryResponses_ = 0;
    uint32_t heartbeatReceived_ = 0;
    uint32_t pingReceived_ = 0;
    uint32_t pongSent_ = 0;
    uint32_t featureFlagsReceived_ = 0;
    uint32_t setConfigFlagReceived_ = 0;
    uint32_t setConfigFlagApplied_ = 0;
    uint32_t setConfigFlagIgnored_ = 0;
    uint32_t ackConfigSent_ = 0;
    uint32_t protocolChangeReceived_ = 0;
    uint32_t unknownPacketsReceived_ = 0;
    uint32_t sendFailures_ = 0;
    uint32_t udpBeginFailures_ = 0;
    uint32_t serverSilenceResets_ = 0;
    uint32_t wifiLostResets_ = 0;
    uint32_t udpReopenRequests_ = 0;
    uint32_t consecutiveSendFailures_ = 0;
    bool udpReopenRequested_ = false;

    uint32_t lastHandshakeMs_ = 0;
    uint32_t lastIncomingPacketMs_ = 0;
    uint32_t lastStateChangeMs_ = 0;
    uint32_t lastHeartbeatMs_ = 0;
    uint32_t lastSensorInfoMs_ = 0;
    uint32_t lastRotationAttemptMs_ = 0;
    uint32_t lastRotationMs_ = 0;
    uint32_t lastTelemetryMs_ = 0;
    uint8_t lastSignalStrength_ = 0;
    int32_t lastRssiDbm_ = 0;
    uint32_t lastRotationSnapshotSequence_ = 0;
    uint32_t lastRotationRuntimeSample_ = 0;
    uint64_t lastRotationTimestampUs_ = 0;
    uint32_t lastRotationQualityFlags_ = 0;
    float lastRotationConfidence_ = 0.0f;
    uint32_t nextUdpBeginRetryMs_ = 0;
    uint32_t serverFoundSendGraceUntilMs_ = 0;
};

} // namespace tracker

#pragma once

#include <cstddef>
#include <cstdint>

#include "defines.h"
#include "network/udp_transport.hpp"
#include "network/wifi_manager.hpp"
#include "output/slimevr_packet_writer.hpp"
#include "runtime/tracker_runtime_types.hpp"
#include "runtime/tracker_health_state.hpp"

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

enum class SlimeVRMotionPacketMode : uint8_t {
    SeparateRotation17Accel4,
    Bundle100Rotation17Accel4,
    ExperimentalRotationAcceleration23,
};

const char* slimevrMotionPacketModeName(SlimeVRMotionPacketMode mode);

enum class SlimeVRSensorInfoSyncState : uint8_t {
    Dirty,
    WaitingForAck,
    Acknowledged,
};

const char* slimevrSensorInfoSyncStateName(SlimeVRSensorInfoSyncState state);

enum class SlimeVRFeatureNegotiationState : uint8_t {
    NotStarted,
    Waiting,
    Negotiated,
    Unavailable,
};

const char* slimevrFeatureNegotiationStateName(SlimeVRFeatureNegotiationState state);

struct SlimeVROutputRuntimeConfig {
    bool enabled = false;
    bool discoveryEnabled = true;
    bool manualServerEnabled = false;
    const char* manualServerHost = nullptr;
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
    bool batteryTelemetryEnabled = TRACKER_SLIMEVR_ENABLE_BATTERY_TELEMETRY != 0;
    uint32_t telemetryIntervalMs = TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS; // legacy/common fallback
    uint32_t signalTelemetryIntervalMs = TRACKER_SLIMEVR_SIGNAL_TELEMETRY_INTERVAL_MS;
    uint32_t temperatureTelemetryIntervalMs = TRACKER_SLIMEVR_TEMPERATURE_TELEMETRY_INTERVAL_MS;
    uint32_t batteryTelemetryIntervalMs = TRACKER_SLIMEVR_BATTERY_TELEMETRY_INTERVAL_MS;
    bool latestTemperatureValid = false;
    float latestTemperatureC = 0.0f;
    bool latestBatteryValid = false;
    float latestBatteryVoltage = 0.0f;
    float latestBatteryPercentage = 0.0f;
    bool hasCompletedRestCalibration = false;
};

struct SlimeVROutputHealthCounters {
    uint32_t sendFailures = 0;
    uint32_t rotationSendFailures = 0;
    uint32_t serverSilenceResets = 0;
    uint32_t wifiLostResets = 0;
};

struct SlimeVROutputRuntimeStatus {
    SlimeVROutputState state = SlimeVROutputState::Disabled;
    bool enabled = false;
    bool wifiConnected = false;
    bool udpReady = false;
    bool serverFound = false;
    bool discoveryEnabled = true;
    bool manualServerEnabled = false;
    char manualServerHost[64] = {};
    bool manualServerResolved = false;
    uint32_t manualServerIpv4 = 0;
    uint32_t manualServerResolveAttempts = 0;
    uint32_t manualServerResolveFailures = 0;
    uint32_t manualServerHandshakesSent = 0;

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
    uint32_t sensorInfoAckReceived = 0;
    uint32_t sensorInfoAckMalformed = 0;
    uint32_t sensorInfoAckMismatch = 0;
    SlimeVRSensorInfoSyncState sensorInfoSyncState = SlimeVRSensorInfoSyncState::Dirty;
    bool sensorInfoDirty = true;
    uint8_t sensorInfoLocalStatus = 0;
    uint16_t sensorInfoLocalConfig = 0;
    bool sensorInfoLocalRestCalibration = false;
    uint8_t sensorInfoAckStatus = 0;
    uint16_t sensorInfoAckConfig = 0;
    bool sensorInfoAckRestCalibration = false;
    uint32_t rotationSent = 0;
    uint32_t accelerationSent = 0;
    uint32_t compactMotionSent = 0;
    uint32_t compactMotionSendFailures = 0;
    uint32_t bundledMotionSent = 0;
    uint32_t bundledMotionSendFailures = 0;
    uint32_t accelerationRateLimited = 0;
    uint32_t featureFlagsSent = 0;
    uint32_t featureFlagsSendFailures = 0;
    bool compactMotionAvailable = true;
    bool compactMotionEnabled = TRACKER_SLIMEVR_USE_COMPACT_MOTION_PACKET != 0;
    bool bundleNegotiationEnabled = TRACKER_SLIMEVR_ENABLE_BUNDLE_NEGOTIATION != 0;
    bool serverFeatureFlagsAvailable = false;
    SlimeVRFeatureNegotiationState featureNegotiationState = SlimeVRFeatureNegotiationState::NotStarted;
    uint8_t featureFlagsRequestAttempts = 0;
    bool serverBundleSupported = false;
    bool serverCompactBundleSupported = false;
    bool bundledMotionEnabled = false;
    SlimeVRMotionPacketMode motionPacketMode = SlimeVRMotionPacketMode::SeparateRotation17Accel4;
    uint16_t fallbackAccelerationRateHz = TRACKER_SLIMEVR_FALLBACK_ACCEL_RATE_HZ;
    uint32_t accelerationSkippedInvalid = 0;
    uint32_t accelerationSkippedConfiguration = 0;
    uint32_t accelerationSkippedComponentMissing = 0;
    uint32_t accelerationSkippedPairDegraded = 0;
    uint32_t accelerationSkippedSaturated = 0;
    uint32_t accelerationSkippedNonFinite = 0;
    uint32_t accelerationSkippedOther = 0;
    uint32_t accelerationSendFailures = 0;
    uint32_t rotationSendDue = 0;
    uint32_t rotationRateLimited = 0;
    uint32_t rotationMissedDeadlines = 0;
    uint32_t rotationLateEvents = 0;
    uint32_t rotationLatenessSumMs = 0;
    uint32_t rotationLatenessMaxMs = 0;
    uint32_t serviceUpdates = 0;
    uint32_t serviceSkips = 0;
    uint32_t signalStrengthSent = 0;
    uint32_t temperatureSent = 0;
    uint32_t magnetometerAccuracySent = 0;
    uint32_t tapSent = 0;
    uint32_t tapSendFailures = 0;
    uint32_t trackerErrorSent = 0;
    uint32_t trackerErrorSendFailures = 0;
    uint8_t lastTapValue = 0;
    uint32_t rotationNoSnapshot = 0;
    uint32_t rotationDuplicateSnapshot = 0;
    uint32_t rotationSuppressedByError = 0;
    uint32_t packetsReceived = 0;
    uint32_t foreignEndpointPacketsDropped = 0;
    uint32_t preSessionPacketsDropped = 0;
    uint32_t malformedPackets = 0;
    uint32_t malformedDatagramLength = 0;
    uint32_t malformedHeartbeat = 0;
    uint32_t malformedPing = 0;
    uint32_t malformedFeatureFlags = 0;
    uint32_t malformedSetConfigFlag = 0;
    uint32_t malformedProtocolChange = 0;
    uint32_t malformedUnknownRaw = 0;
    uint32_t discoveryResponses = 0;
    uint32_t heartbeatReceived = 0;
    uint32_t pingReceived = 0;
    uint32_t pongSent = 0;
    uint32_t featureFlagsReceived = 0;
    uint32_t setConfigFlagReceived = 0;
    uint32_t setConfigFlagApplied = 0;
    uint32_t setConfigFlagApplyFailures = 0;
    uint32_t setConfigFlagIgnored = 0;
    uint32_t ackConfigSent = 0;
    uint32_t ackConfigSendFailures = 0;
    uint32_t userActionSent = 0;
    uint32_t userActionSendFailures = 0;
    SlimeVRUserAction lastUserAction = SlimeVRUserAction::None;
    uint32_t protocolChangeReceived = 0;
    uint32_t protocolChangeIgnored = 0;
    uint32_t unknownPacketsReceived = 0;
    uint32_t sendFailures = 0;
    uint32_t rotationSendFailures = 0;
    uint32_t controlSendFailures = 0;
    uint32_t telemetrySendFailures = 0;
    uint32_t discoverySendFailures = 0;
    uint32_t tapTransportSendFailures = 0;
    uint32_t udpBeginFailures = 0;
    uint32_t serverSilenceResets = 0;
    uint32_t wifiLostResets = 0;
    uint32_t udpReopenRequests = 0;
    uint32_t udpReopenSuppressedRecentRx = 0;
    uint32_t consecutiveSendFailures = 0;

    uint32_t nextPacketNumber = 0;
    uint16_t rotationRateHz = 0;
    bool preparedOutputAvailable = false;
    bool magSupportEnabled = false;
    bool magEnabled = false;
    uint16_t sensorConfig = 0;
    bool signalTelemetryEnabled = false;
    bool temperatureTelemetryEnabled = false;
    bool batteryTelemetryEnabled = false;
    uint32_t telemetryIntervalMs = 0;
    uint32_t signalTelemetryIntervalMs = 0;
    uint32_t temperatureTelemetryIntervalMs = 0;
    uint32_t batteryTelemetryIntervalMs = 0;
    int8_t lastSignalStrengthDbm = 0;
    int32_t lastRssiDbm = 0;
    float lastTemperatureC = 0.0f;
    bool lastTemperatureValid = false;
    uint32_t batterySent = 0;
    uint32_t batterySendFailures = 0;
    float lastBatteryVoltage = 0.0f;
    float lastBatteryPercentage = 0.0f;
    bool lastBatteryValid = false;
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
    uint32_t lastRotationSnapshotAgeUs = 0;
    bool trackerErrorActive = false;
    bool trackerDegradedNoImu = false;
    uint8_t trackerErrorCode = 0;
    uint32_t trackerHealthRevision = 0;
    char trackerErrorMessage[64] = {};
};

class SlimeVROutputRuntime {
public:
    void begin(IUdpTransport& udp,
               const TrackerWifiManager& wifi,
               SlimeVRCopyOutputSnapshotFn copyOutputSnapshot = nullptr,
               void* copyOutputSnapshotUser = nullptr);
    void configure(const SlimeVROutputRuntimeConfig& config);
    void updateLiveState(bool latestTemperatureValid,
                         float latestTemperatureC,
                         bool latestBatteryValid,
                         float latestBatteryVoltage,
                         float latestBatteryPercentage,
                         bool hasCompletedRestCalibration);
    void setTrackerHealth(const TrackerHealthSnapshot& health);
    void resetCounters();
    void stop();
    void restart();
    bool update(uint32_t nowMs);
    void requestSensorInfoRefresh();
    bool sendTap(uint8_t value);
    bool sendUserAction(SlimeVRUserAction action);

    bool enabled() const { return enabled_; }
    bool serverFound() const { return serverFound_; }
    SlimeVROutputState state() const { return state_; }
    SlimeVROutputRuntimeStatus status() const;
    void healthCounters(SlimeVROutputHealthCounters& out) const;
    // Returns true only when a live rotation deadline is armed.  outSlackMs is
    // zero when already due/late and otherwise the wrap-safe time remaining.
    bool rotationDeadlineSlackMs(uint32_t nowMs, uint32_t& outSlackMs) const;

private:
    void transitionTo(SlimeVROutputState state, uint32_t nowMs);
    void resetConnectionState(bool keepCounters);
    void resetServerFeatureNegotiation();
    void ensureUdp(uint32_t nowMs);
    void pollIncoming(uint32_t nowMs);
    bool handleIncomingPacket(const uint8_t* data, size_t len, const UdpEndpoint& remote, uint32_t nowMs);
    bool handleSensorInfoAck(const uint8_t* data, size_t len);
    bool handleHeartbeat(const uint8_t* data, size_t len, uint32_t nowMs);
    bool handlePingPong(const uint8_t* data, size_t len);
    bool handleFeatureFlags(const uint8_t* data, size_t len);
    bool handleSetConfigFlag(const uint8_t* data, size_t len, uint32_t nowMs);
    bool handleProtocolChange(const uint8_t* data, size_t len);
    bool sendAckConfigChange(uint8_t targetSensorId, uint16_t configType);
    void maybeSendDiscovery(uint32_t nowMs);
    void sendHandshakeTo(const UdpEndpoint& endpoint, uint32_t nowMs);
    void sendSensorInfo(uint32_t nowMs);
    SlimeVRSensorInfo desiredSensorInfo() const;
    void markSensorInfoDirty();
    static bool sensorInfoEqual(const SlimeVRSensorInfo& a, const SlimeVRSensorInfo& b);
    void sendHeartbeat(uint32_t nowMs);
    void maybeSendFeatureFlags(uint32_t nowMs);
    void maybeSendTelemetry(uint32_t nowMs);
    bool serviceDue(uint32_t nowMs) const;
    uint32_t activitySignature() const;
    bool activityChanged(SlimeVROutputState previousState, uint32_t previousSignature) const;
    void sendSignalStrength(uint32_t nowMs);
    void sendTemperature(uint32_t nowMs);
    void sendBatteryLevel(uint32_t nowMs);
    void sendMagnetometerAccuracy(uint32_t nowMs);
    void maybeSendTrackerError(uint32_t nowMs);
    void maybeRecordRotationSuppressedByError(uint32_t nowMs);
    void maybeSendRotation(uint32_t nowMs);
    bool consumeRotationDeadline(uint32_t nowMs);
    bool consumeFallbackAccelerationDeadline(uint32_t nowMs);
    void resetRotationDeadline();
    void resetFallbackAccelerationDeadline();
    void armRotationDeadline(uint32_t deadlineMs);
    SlimeVRMotionPacketMode motionPacketMode() const;
    bool serverFeatureEnabled(uint8_t bit) const;
    void sendRotation(const TrackerPreparedOutputSnapshot& snapshot, uint32_t nowMs);
    void makeHandshakeInfo(SlimeVRHandshakeInfo& info) const;
    enum class PacketPurpose : uint8_t {
        Discovery,
        Control,
        Telemetry,
        Rotation,
        Acceleration,
        MotionCombined,
        Tap,
        UserAction,
        ErrorReport,
    };

    bool sendPacket(const SlimeVRPacketWriteResult& packet, const UdpEndpoint& endpoint, PacketPurpose purpose);
    void recordSendFailure(PacketPurpose purpose);
    uint32_t rotationPeriodMs() const;
    uint16_t sensorConfigFlags() const;
    static uint8_t accuracyFromConfidence(float confidence);
    static int8_t signalStrengthFromRssi(int32_t rssiDbm);

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
    char manualServerHost_[64] = {};
    UdpEndpoint manualServerEndpoint_;

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
    bool batteryTelemetryEnabled_ = TRACKER_SLIMEVR_ENABLE_BATTERY_TELEMETRY != 0;
    uint32_t telemetryIntervalMs_ = TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS; // legacy/common fallback
    uint32_t signalTelemetryIntervalMs_ = TRACKER_SLIMEVR_SIGNAL_TELEMETRY_INTERVAL_MS;
    uint32_t temperatureTelemetryIntervalMs_ = TRACKER_SLIMEVR_TEMPERATURE_TELEMETRY_INTERVAL_MS;
    uint32_t batteryTelemetryIntervalMs_ = TRACKER_SLIMEVR_BATTERY_TELEMETRY_INTERVAL_MS;
    bool latestTemperatureValid_ = false;
    float latestTemperatureC_ = 0.0f;
    bool latestBatteryValid_ = false;
    float latestBatteryVoltage_ = 0.0f;
    float latestBatteryPercentage_ = 0.0f;
    bool hasCompletedRestCalibration_ = false;
    SlimeVRSensorInfo desiredSensorInfo_;
    SlimeVRSensorInfo lastSentSensorInfo_;
    SlimeVRSensorInfo acknowledgedSensorInfo_;
    SlimeVRSensorInfoSyncState sensorInfoSyncState_ = SlimeVRSensorInfoSyncState::Dirty;
    bool sensorInfoDirty_ = true;
    TrackerHealthSnapshot trackerHealth_;
    uint32_t lastTrackerErrorMs_ = 0;

    uint32_t lastPingId_ = 0;
    uint32_t lastServerFeatureFlags_ = 0;
    uint8_t serverFeatureFlags_[4] = {};
    uint8_t serverFeatureFlagsLength_ = 0;
    bool serverFeatureFlagsAvailable_ = false;
    SlimeVRFeatureNegotiationState featureNegotiationState_ = SlimeVRFeatureNegotiationState::NotStarted;
    uint8_t featureFlagsRequestAttempts_ = 0;
    uint32_t lastFeatureFlagsRequestMs_ = 0;
    uint8_t lastSetConfigSensorId_ = 0;
    uint16_t lastSetConfigType_ = 0;
    bool lastSetConfigState_ = false;
    bool lastSetConfigApplied_ = false;
    uint8_t lastProtocolTarget_ = 0;
    uint8_t lastProtocolVersion_ = 0;
    uint8_t lastUnknownPacketType_ = 0;

    UdpEndpoint serverEndpoint_;

    uint32_t handshakesSent_ = 0;
    uint32_t manualServerResolveAttempts_ = 0;
    uint32_t manualServerResolveFailures_ = 0;
    uint32_t manualServerHandshakesSent_ = 0;
    uint32_t heartbeatSent_ = 0;
    uint32_t sensorInfoSent_ = 0;
    uint32_t sensorInfoAckReceived_ = 0;
    uint32_t sensorInfoAckMalformed_ = 0;
    uint32_t sensorInfoAckMismatch_ = 0;
    uint32_t rotationSent_ = 0;
    uint32_t accelerationSent_ = 0;
    uint32_t compactMotionSent_ = 0;
    uint32_t compactMotionSendFailures_ = 0;
    uint32_t bundledMotionSent_ = 0;
    uint32_t bundledMotionSendFailures_ = 0;
    uint32_t accelerationRateLimited_ = 0;
    uint32_t featureFlagsSent_ = 0;
    uint32_t featureFlagsSendFailures_ = 0;
    uint32_t accelerationSkippedInvalid_ = 0;
    uint32_t accelerationSkippedConfiguration_ = 0;
    uint32_t accelerationSkippedComponentMissing_ = 0;
    uint32_t accelerationSkippedPairDegraded_ = 0;
    uint32_t accelerationSkippedSaturated_ = 0;
    uint32_t accelerationSkippedNonFinite_ = 0;
    uint32_t accelerationSkippedOther_ = 0;
    uint32_t accelerationSendFailures_ = 0;
    uint32_t rotationSendDue_ = 0;
    uint32_t rotationRateLimited_ = 0;
    uint32_t rotationMissedDeadlines_ = 0;
    uint32_t rotationLateEvents_ = 0;
    uint32_t rotationLatenessSumMs_ = 0;
    uint32_t rotationLatenessMaxMs_ = 0;
    uint32_t serviceUpdates_ = 0;
    uint32_t serviceSkips_ = 0;
    uint32_t signalStrengthSent_ = 0;
    uint32_t temperatureSent_ = 0;
    uint32_t batterySent_ = 0;
    uint32_t batterySendFailures_ = 0;
    uint32_t magnetometerAccuracySent_ = 0;
    uint32_t tapSent_ = 0;
    uint32_t tapSendFailures_ = 0;
    uint32_t trackerErrorSent_ = 0;
    uint32_t trackerErrorSendFailures_ = 0;
    uint8_t lastTapValue_ = 0;
    uint32_t rotationNoSnapshot_ = 0;
    uint32_t rotationDuplicateSnapshot_ = 0;
    uint32_t rotationSuppressedByError_ = 0;
    uint32_t packetsReceived_ = 0;
    uint32_t foreignEndpointPacketsDropped_ = 0;
    uint32_t preSessionPacketsDropped_ = 0;
    uint32_t malformedPackets_ = 0;
    uint32_t malformedDatagramLength_ = 0;
    uint32_t malformedHeartbeat_ = 0;
    uint32_t malformedPing_ = 0;
    uint32_t malformedFeatureFlags_ = 0;
    uint32_t malformedSetConfigFlag_ = 0;
    uint32_t malformedProtocolChange_ = 0;
    uint32_t malformedUnknownRaw_ = 0;
    uint32_t discoveryResponses_ = 0;
    uint32_t heartbeatReceived_ = 0;
    uint32_t pingReceived_ = 0;
    uint32_t pongSent_ = 0;
    uint32_t featureFlagsReceived_ = 0;
    uint32_t setConfigFlagReceived_ = 0;
    uint32_t setConfigFlagApplied_ = 0;
    uint32_t setConfigFlagApplyFailures_ = 0;
    uint32_t setConfigFlagIgnored_ = 0;
    uint32_t ackConfigSent_ = 0;
    uint32_t ackConfigSendFailures_ = 0;
    uint32_t userActionSent_ = 0;
    uint32_t userActionSendFailures_ = 0;
    SlimeVRUserAction lastUserAction_ = SlimeVRUserAction::None;
    uint32_t protocolChangeReceived_ = 0;
    uint32_t protocolChangeIgnored_ = 0;
    uint32_t unknownPacketsReceived_ = 0;
    uint32_t sendFailures_ = 0;
    uint32_t rotationSendFailures_ = 0;
    uint32_t controlSendFailures_ = 0;
    uint32_t telemetrySendFailures_ = 0;
    uint32_t discoverySendFailures_ = 0;
    uint32_t tapTransportSendFailures_ = 0;
    uint32_t udpBeginFailures_ = 0;
    uint32_t serverSilenceResets_ = 0;
    uint32_t wifiLostResets_ = 0;
    uint32_t udpReopenRequests_ = 0;
    uint32_t udpReopenSuppressedRecentRx_ = 0;
    uint32_t consecutiveSendFailures_ = 0;
    bool udpReopenRequested_ = false;

    uint32_t lastHandshakeMs_ = 0;
    uint32_t lastDiscoveryAttemptMs_ = 0;
    uint32_t lastIncomingPacketMs_ = 0;
    uint32_t lastStateChangeMs_ = 0;
    uint32_t lastHeartbeatMs_ = 0;
    uint32_t lastSensorInfoMs_ = 0;
    uint32_t nextRotationDeadlineMs_ = 0;
    bool rotationDeadlineArmed_ = false;
    uint32_t nextFallbackAccelerationDeadlineMs_ = 0;
    bool fallbackAccelerationDeadlineArmed_ = false;
    uint32_t lastRotationMs_ = 0;
    uint32_t lastTelemetryMs_ = 0; // legacy/status only after split intervals
    uint32_t lastServiceUpdateMs_ = 0;
    uint32_t lastSignalTelemetryMs_ = 0;
    uint32_t lastTemperatureTelemetryMs_ = 0;
    uint32_t lastBatteryTelemetryMs_ = 0;
    int8_t lastSignalStrengthDbm_ = 0;
    int32_t lastRssiDbm_ = 0;
    uint32_t lastRotationSnapshotSequence_ = 0;
    uint32_t lastRotationRuntimeSample_ = 0;
    uint64_t lastRotationTimestampUs_ = 0;
    uint32_t lastRotationQualityFlags_ = 0;
    float lastRotationConfidence_ = 0.0f;
    uint32_t lastRotationSnapshotAgeUs_ = 0;
    uint32_t nextUdpBeginRetryMs_ = 0;
    uint32_t serverFoundSendGraceUntilMs_ = 0;
    uint32_t lastRuntimeNowMs_ = 0;
};

} // namespace tracker

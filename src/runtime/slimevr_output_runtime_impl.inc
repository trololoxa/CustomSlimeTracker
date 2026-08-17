#include "runtime/slimevr_output_runtime.hpp"

#include <Arduino.h>

#include "build_config/build_identity.hpp"

#include <cstring>
#include <cmath>
#include <cerrno>

namespace tracker {

namespace {

constexpr uint32_t UDP_BEGIN_RETRY_MS = 1000;
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5000;
constexpr uint32_t SENSOR_INFO_INTERVAL_MS = 1000;
constexpr uint32_t TRACKER_ERROR_INTERVAL_MS = 30000;
constexpr uint32_t SERVER_SILENCE_TIMEOUT_MS = TRACKER_SLIMEVR_SERVER_SILENCE_TIMEOUT_MS;
constexpr uint32_t SERVER_FOUND_SEND_GRACE_MS = 100;
constexpr uint16_t ROTATION_RATE_HZ_DEFAULT = 100;
constexpr uint16_t ROTATION_RATE_HZ_MAX = TRACKER_SLIMEVR_OUTPUT_RATE_HZ_MAX;
static_assert(TRACKER_SLIMEVR_TX_FAILURE_WINDOW_ATTEMPTS == 32U,
              "TX failure outcome window currently uses one uint32_t bitmap");
static_assert(TRACKER_SLIMEVR_TX_FAILURE_WINDOW_FAILURES > 0U &&
              TRACKER_SLIMEVR_TX_FAILURE_WINDOW_FAILURES <= TRACKER_SLIMEVR_TX_FAILURE_WINDOW_ATTEMPTS,
              "TX failure window threshold must fit the outcome window");
static_assert(TRACKER_SLIMEVR_TX_OTHER_REBIND_CONSECUTIVE_FAILURES > 0U,
              "non-pressure TX consecutive-failure threshold must be nonzero");
static_assert(TRACKER_SLIMEVR_TX_PRESSURE_REBIND_STALL_MS > TRACKER_SLIMEVR_TX_BACKOFF_INITIAL_MS,
              "pressure rebind stall must exceed the initial backoff");
static_assert(TRACKER_SLIMEVR_TX_POST_REBIND_REOPEN_STALL_MS > TRACKER_SLIMEVR_TX_REBIND_SEND_GRACE_MS,
              "post-rebind reopen stall must exceed the send grace");

void copyCString(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    if (!src) src = "";
    const size_t len = std::strlen(src);
    const size_t n = len < (dstSize - 1u) ? len : (dstSize - 1u);
    if (n != 0u) std::memcpy(dst, src, n);
    dst[n] = '\0';
}


uint16_t readU16Be(const uint8_t* p) {
    return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) |
                                 static_cast<uint16_t>(p[1]));
}

uint32_t readU32Be(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
}

struct SlimeVRIncomingPacketView {
    uint8_t type = 0;
    const uint8_t* payload = nullptr;
    size_t payloadLen = 0;
    bool headered = false;
};

bool parseIncomingPacketView(const uint8_t* data, size_t len, SlimeVRIncomingPacketView& out) {
    if (!data || len < SLIMEVR_PACKET_HEADER_SIZE) return false;

    // Normal SlimeVR server packets use the 12-byte header:
    // type:u32be + packetNumber:u64be + payload. Discovery and the six-byte
    // SensorInfo acknowledgement are the only headerless exceptions and are
    // handled before this helper.
    const uint32_t packetType = readU32Be(data);
    if (packetType > 0xffu) return false;

    out.type = static_cast<uint8_t>(packetType);
    out.payload = data + SLIMEVR_PACKET_HEADER_SIZE;
    out.payloadLen = len - SLIMEVR_PACKET_HEADER_SIZE;
    out.headered = true;
    return true;
}

bool parseShortSensorInfoAck(const uint8_t* data, size_t len, uint8_t& sensorId, uint8_t& status) {
    // Server SensorInfo acknowledgement is the protocol exception: u32 type
    // followed by sensor id and status, without the normal u64 packet number.
    if (!data || len != 6u) return false;
    if (readU32Be(data) != static_cast<uint32_t>(SlimeVRReceivePacketType::SensorInfo)) return false;
    sensorId = data[4];
    status = data[5];
    return true;
}

} // namespace

const char* slimevrMotionPacketModeName(SlimeVRMotionPacketMode mode) {
    switch (mode) {
        case SlimeVRMotionPacketMode::Rotation17Only: return "rotation_17_only";
        case SlimeVRMotionPacketMode::SeparateRotation17Accel4: return "rotation_17_plus_accel_4_fallback";
        case SlimeVRMotionPacketMode::Bundle100Rotation17Accel4: return "bundle_100_rotation_17_accel_4";
        case SlimeVRMotionPacketMode::ExperimentalRotationAcceleration23: return "rotation_acceleration_23";
    }
    return "unknown";
}

const char* slimevrSensorInfoSyncStateName(SlimeVRSensorInfoSyncState state) {
    switch (state) {
        case SlimeVRSensorInfoSyncState::Dirty: return "dirty";
        case SlimeVRSensorInfoSyncState::WaitingForAck: return "waiting_for_ack";
        case SlimeVRSensorInfoSyncState::Acknowledged: return "acknowledged";
    }
    return "unknown";
}

const char* slimevrFeatureNegotiationStateName(SlimeVRFeatureNegotiationState state) {
    switch (state) {
        case SlimeVRFeatureNegotiationState::NotStarted: return "not_started";
        case SlimeVRFeatureNegotiationState::Waiting: return "waiting";
        case SlimeVRFeatureNegotiationState::Negotiated: return "negotiated";
        case SlimeVRFeatureNegotiationState::Unavailable: return "unavailable";
    }
    return "unknown";
}

const char* slimevrTxPressureStateName(SlimeVRTxPressureState state) {
    switch (state) {
        case SlimeVRTxPressureState::Healthy: return "healthy";
        case SlimeVRTxPressureState::TransientPressure: return "transient_pressure";
        case SlimeVRTxPressureState::SustainedPressure: return "sustained_pressure";
        case SlimeVRTxPressureState::AwaitingPostRebindSuccess: return "awaiting_post_rebind_success";
        case SlimeVRTxPressureState::FullReopenCooldown: return "full_reopen_cooldown";
    }
    return "unknown";
}

const char* slimevrTxRecoveryReasonName(SlimeVRTxRecoveryReason reason) {
    switch (reason) {
        case SlimeVRTxRecoveryReason::None: return "none";
        case SlimeVRTxRecoveryReason::PressureMotionStall: return "pressure_motion_stall";
        case SlimeVRTxRecoveryReason::NonPressureTransportFailure: return "nonpressure_transport_failure";
        case SlimeVRTxRecoveryReason::StaleServerRx: return "stale_server_rx";
        case SlimeVRTxRecoveryReason::RebindFailure: return "rebind_failure";
        case SlimeVRTxRecoveryReason::PostRebindMotionStall: return "post_rebind_motion_stall";
    }
    return "unknown";
}

const char* slimevrOutputStateName(SlimeVROutputState state) {
    switch (state) {
        case SlimeVROutputState::Disabled: return "disabled";
        case SlimeVROutputState::WaitingForWifi: return "waiting_for_wifi";
        case SlimeVROutputState::UdpStarting: return "udp_starting";
        case SlimeVROutputState::Discovering: return "discovering";
        case SlimeVROutputState::ServerFound: return "server_found";
        case SlimeVROutputState::Error: return "error";
    }
    return "unknown";
}

void SlimeVROutputRuntime::begin(IUdpTransport& udp,
                                    const TrackerWifiManager& wifi,
                                    SlimeVRCopyOutputSnapshotFn copyOutputSnapshot,
                                    void* copyOutputSnapshotUser,
                                    SlimeVRRotationSoftwareAgeSinkFn rotationSoftwareAgeSink,
                                    void* rotationSoftwareAgeSinkUser) {
    udp_ = &udp;
    wifi_ = &wifi;
    copyOutputSnapshot_ = copyOutputSnapshot;
    copyOutputSnapshotUser_ = copyOutputSnapshotUser;
    rotationSoftwareAgeSink_ = rotationSoftwareAgeSink;
    rotationSoftwareAgeSinkUser_ = rotationSoftwareAgeSinkUser;
    resetConnectionState(true);
}

void SlimeVROutputRuntime::configure(const SlimeVROutputRuntimeConfig& config) {
    const bool incomingRestComplete = config.hasCompletedRestCalibration;
    const bool restCalibrationChanged = hasCompletedRestCalibration_ != incomingRestComplete;
    const uint16_t incomingServerPort = config.serverPort == 0 ? SLIMEVR_DEFAULT_SERVER_PORT : config.serverPort;
    const uint16_t incomingLocalPort = config.localPort == 0 ? SLIMEVR_DISCOVERY_LOCAL_PORT : config.localPort;
    const char* incomingManualServerHost = config.manualServerHost ? config.manualServerHost : "";
    const bool incomingMagEnabled = config.magSupportEnabled && config.magEnabled;
    const bool configChanged = enabled_ != config.enabled ||
                               discoveryEnabled_ != config.discoveryEnabled ||
                               manualServerEnabled_ != config.manualServerEnabled ||
                               std::strncmp(manualServerHost_, incomingManualServerHost, sizeof(manualServerHost_)) != 0 ||
                               serverPort_ != incomingServerPort ||
                               localPort_ != incomingLocalPort ||
                               sensorId_ != config.sensorId ||
                               magSupportEnabled_ != config.magSupportEnabled ||
                               magEnabled_ != incomingMagEnabled ||
                               batteryTelemetryEnabled_ != config.batteryTelemetryEnabled ||
                               std::strncmp(deviceName_, config.deviceName ? config.deviceName : "", sizeof(deviceName_)) != 0;

    const bool rotationRateChanged = rotationRateHz_ != config.rotationRateHz;
    const bool motionPacketPolicyChanged = motionPacketPolicy_ != config.motionPacketPolicy;

    enabled_ = config.enabled;
    discoveryEnabled_ = config.discoveryEnabled;
    manualServerEnabled_ = config.manualServerEnabled;
    copyCString(manualServerHost_, sizeof(manualServerHost_), incomingManualServerHost);
    sensorId_ = config.sensorId;
    serverPort_ = config.serverPort == 0 ? SLIMEVR_DEFAULT_SERVER_PORT : config.serverPort;
    localPort_ = config.localPort == 0 ? SLIMEVR_DISCOVERY_LOCAL_PORT : config.localPort;
    discoveryIntervalMs_ = config.discoveryIntervalMs == 0 ? 1u : config.discoveryIntervalMs;
    rotationRateHz_ = config.rotationRateHz == 0 ? ROTATION_RATE_HZ_DEFAULT : config.rotationRateHz;
    if (rotationRateHz_ > ROTATION_RATE_HZ_MAX) rotationRateHz_ = ROTATION_RATE_HZ_MAX;
    incomingPacketsPerUpdate_ = config.incomingPacketsPerUpdate == 0 ? 1u : config.incomingPacketsPerUpdate;
    motionPacketPolicy_ = config.motionPacketPolicy;
    magSupportEnabled_ = config.magSupportEnabled;
    magEnabled_ = config.magEnabled;
    if (!magSupportEnabled_) magEnabled_ = false;
    setConfigFlag_ = config.setConfigFlag;
    setConfigFlagUser_ = config.setConfigFlagUser;
    signalTelemetryEnabled_ = config.signalTelemetryEnabled;
    temperatureTelemetryEnabled_ = config.temperatureTelemetryEnabled;
    batteryTelemetryEnabled_ = config.batteryTelemetryEnabled;
    telemetryIntervalMs_ = config.telemetryIntervalMs == 0 ? TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS : config.telemetryIntervalMs;
    signalTelemetryIntervalMs_ = config.signalTelemetryIntervalMs == 0 ? telemetryIntervalMs_ : config.signalTelemetryIntervalMs;
    temperatureTelemetryIntervalMs_ = config.temperatureTelemetryIntervalMs == 0 ? telemetryIntervalMs_ : config.temperatureTelemetryIntervalMs;
    batteryTelemetryIntervalMs_ = config.batteryTelemetryIntervalMs == 0 ? telemetryIntervalMs_ : config.batteryTelemetryIntervalMs;
    updateLiveState(
        config.latestTemperatureValid,
        config.latestTemperatureC,
        config.latestBatteryValid,
        config.latestBatteryVoltage,
        config.latestBatteryPercentage,
        incomingRestComplete
    );
    copyCString(deviceName_, sizeof(deviceName_), config.deviceName && config.deviceName[0] ? config.deviceName : "c3-6dsv-tracker");

    if (!enabled_) {
        stop();
        return;
    }

    if (configChanged) {
        resetConnectionState(true);
    } else {
        if (rotationRateChanged || motionPacketPolicyChanged) {
            rotationPhaseOffsetValid_ = false;
            resetRotationDeadline();
            resetFallbackAccelerationDeadline();
        }
        if (restCalibrationChanged) {
            requestSensorInfoRefresh();
        }
    }
}


void SlimeVROutputRuntime::setMotionPacketPolicy(SlimeVRMotionPacketPolicy policy) {
    if (!slimevrMotionPacketPolicyValid(policy)) {
        policy = SlimeVRMotionPacketPolicy::QuaternionOnly;
    }
    if (motionPacketPolicy_ == policy) return;

    motionPacketPolicy_ = policy;
    rotationPhaseOffsetValid_ = false;
    resetRotationDeadline();
    resetFallbackAccelerationDeadline();
}

void SlimeVROutputRuntime::updateLiveState(bool latestTemperatureValid,
                                           float latestTemperatureC,
                                           bool latestBatteryValid,
                                           float latestBatteryVoltage,
                                           float latestBatteryPercentage,
                                           bool hasCompletedRestCalibration) {
    const bool restCalibrationChanged = hasCompletedRestCalibration_ != hasCompletedRestCalibration;

    latestTemperatureValid_ = latestTemperatureValid && std::isfinite(latestTemperatureC);
    latestTemperatureC_ = latestTemperatureValid_ ? latestTemperatureC : 0.0f;

    latestBatteryValid_ = latestBatteryValid &&
                          std::isfinite(latestBatteryVoltage) &&
                          std::isfinite(latestBatteryPercentage);
    latestBatteryVoltage_ = latestBatteryValid_ ? latestBatteryVoltage : 0.0f;
    latestBatteryPercentage_ = latestBatteryValid_ ? latestBatteryPercentage : 0.0f;
    if (latestBatteryPercentage_ < 0.0f) latestBatteryPercentage_ = 0.0f;
    if (latestBatteryPercentage_ > 100.0f) latestBatteryPercentage_ = 100.0f;

    hasCompletedRestCalibration_ = hasCompletedRestCalibration;
    if (restCalibrationChanged) {
        requestSensorInfoRefresh();
    }
}

SlimeVRSensorInfo SlimeVROutputRuntime::desiredSensorInfo() const {
    SlimeVRSensorInfo info;
    info.sensorId = sensorId_;
    // Keep SensorInfo online even in firmware-fault/degraded mode. Firmware
    // faults are reported through packet 14 while pose output is suppressed.
    info.sensorState = SlimeVRSensorState::Online;
    info.sensorConfig = sensorConfigFlags();
    info.hasCompletedRestCalibration = !trackerHealth_.degradedNoImu && hasCompletedRestCalibration_;
    return info;
}

bool SlimeVROutputRuntime::sensorInfoEqual(const SlimeVRSensorInfo& a, const SlimeVRSensorInfo& b) {
    return a.sensorId == b.sensorId &&
           a.sensorState == b.sensorState &&
           a.imuType == b.imuType &&
           a.sensorConfig == b.sensorConfig &&
           a.hasCompletedRestCalibration == b.hasCompletedRestCalibration &&
           a.sensorPosition == b.sensorPosition &&
           a.sensorDataType == b.sensorDataType;
}

void SlimeVROutputRuntime::markSensorInfoDirty() {
    desiredSensorInfo_ = desiredSensorInfo();
    sensorInfoDirty_ = true;
    sensorInfoSyncState_ = SlimeVRSensorInfoSyncState::Dirty;
    lastSensorInfoMs_ = 0;
}

void SlimeVROutputRuntime::requestSensorInfoRefresh() {
    markSensorInfoDirty();
}

void SlimeVROutputRuntime::setTrackerHealth(const TrackerHealthSnapshot& health) {
    const bool changed = trackerHealth_.revision != health.revision ||
                         trackerHealth_.fatalActive != health.fatalActive ||
                         trackerHealth_.degradedNoImu != health.degradedNoImu ||
                         trackerHealth_.faultCode != health.faultCode ||
                         std::strncmp(trackerHealth_.message, health.message, sizeof(trackerHealth_.message)) != 0;
    trackerHealth_ = health;
    trackerHealth_.message[sizeof(trackerHealth_.message) - 1u] = '\0';
    if (changed) {
        requestSensorInfoRefresh();
        lastTrackerErrorMs_ = 0;
        resetRotationDeadline();
    }
}

void SlimeVROutputRuntime::resetCounters() {
    handshakesSent_ = 0;
    manualServerResolveAttempts_ = 0;
    manualServerResolveFailures_ = 0;
    manualServerHandshakesSent_ = 0;
    heartbeatSent_ = 0;
    sensorInfoSent_ = 0;
    sensorInfoAckReceived_ = 0;
    sensorInfoAckMalformed_ = 0;
    sensorInfoAckMismatch_ = 0;
    rotationSent_ = 0;
    accelerationSent_ = 0;
    compactMotionSent_ = 0;
    compactMotionSendFailures_ = 0;
    bundledMotionSent_ = 0;
    bundledMotionSendFailures_ = 0;
    accelerationRateLimited_ = 0;
    featureFlagsSent_ = 0;
    featureFlagsSendFailures_ = 0;
    accelerationSkippedInvalid_ = 0;
    accelerationSkippedConfiguration_ = 0;
    accelerationSkippedComponentMissing_ = 0;
    accelerationSkippedPairDegraded_ = 0;
    accelerationSkippedSaturated_ = 0;
    accelerationSkippedNonFinite_ = 0;
    accelerationSkippedOther_ = 0;
    accelerationSendFailures_ = 0;
    rotationSendDue_ = 0;
    rotationRateLimited_ = 0;
    rotationMissedDeadlines_ = 0;
    rotationLateEvents_ = 0;
    rotationLatenessSumMs_ = 0;
    rotationLatenessMaxMs_ = 0;
    serviceUpdates_ = 0;
    serviceSkips_ = 0;
    signalStrengthSent_ = 0;
    temperatureSent_ = 0;
    batterySent_ = 0;
    batterySendFailures_ = 0;
    magnetometerAccuracySent_ = 0;
    tapSent_ = 0;
    tapSendFailures_ = 0;
    trackerErrorSent_ = 0;
    trackerErrorSendFailures_ = 0;
    lastTapValue_ = 0;
    rotationNoSnapshot_ = 0;
    rotationDuplicateSnapshot_ = 0;
    rotationSuppressedByError_ = 0;
    packetsReceived_ = 0;
    foreignEndpointPacketsDropped_ = 0;
    preSessionPacketsDropped_ = 0;
    malformedPackets_ = 0;
    malformedDatagramLength_ = 0;
    malformedHeartbeat_ = 0;
    malformedPing_ = 0;
    malformedFeatureFlags_ = 0;
    malformedSetConfigFlag_ = 0;
    malformedProtocolChange_ = 0;
    malformedUnknownRaw_ = 0;
    discoveryResponses_ = 0;
    heartbeatReceived_ = 0;
    pingReceived_ = 0;
    pongSent_ = 0;
    featureFlagsReceived_ = 0;
    setConfigFlagReceived_ = 0;
    setConfigFlagApplied_ = 0;
    setConfigFlagApplyFailures_ = 0;
    setConfigFlagIgnored_ = 0;
    ackConfigSent_ = 0;
    ackConfigSendFailures_ = 0;
    userActionSent_ = 0;
    userActionSendFailures_ = 0;
    lastUserAction_ = SlimeVRUserAction::None;
    protocolChangeReceived_ = 0;
    protocolChangeIgnored_ = 0;
    unknownPacketsReceived_ = 0;
    sendFailures_ = 0;
    rotationSendFailures_ = 0;
    controlSendFailures_ = 0;
    telemetrySendFailures_ = 0;
    discoverySendFailures_ = 0;
    tapTransportSendFailures_ = 0;
    udpBeginFailures_ = 0;
    serverSilenceResets_ = 0;
    wifiLostResets_ = 0;
    udpReopenRequests_ = 0;
    udpTransportRebindRequests_ = 0;
    udpTransportRebindSuccesses_ = 0;
    udpTransportRebindFailures_ = 0;
    udpFullReopenEscalations_ = 0;
    txBackoffDrops_ = 0;
    txPressureFailures_ = 0;
    txOtherFailures_ = 0;
    txFailureWindowTrips_ = 0;
    lastUdpSendError_ = 0;
    txPressureEpisodeCount_ = 0;
    txPressureEpisodeMaxMs_ = 0;
    txPressureStableResets_ = 0;
    udpRebindSuppressedCooldown_ = 0;
    udpFullReopenSuppressedCooldown_ = 0;
    physicalDatagramsSent_ = 0;
    motionDatagramsSent_ = 0;
    separateRotationDatagramsSent_ = 0;
    separateAccelerationDatagramsSent_ = 0;
    backgroundControlDatagramsSent_ = 0;
    criticalControlDatagramsSent_ = 0;
    motionPacketModeTransitions_ = 0;
    bundleToSeparateTransitions_ = 0;
    separateToBundleTransitions_ = 0;
    accelerationSuppressedDuringNegotiation_ = 0;
    resetTxRecoveryState();
    lastUdpTransportRebindMs_ = 0;
    lastUdpTransportRebindValid_ = false;
    lastUdpFullReopenMs_ = 0;
    lastUdpFullReopenValid_ = false;
    udpTransportRebindRequested_ = false;
    udpReopenRequested_ = false;
}

void SlimeVROutputRuntime::stop() {
    enabled_ = false;
    if (udp_) udp_->stop();
    resetConnectionState(true);
}

void SlimeVROutputRuntime::restart() {
    if (udp_) udp_->stop();
    resetConnectionState(true);
    enabled_ = true;
}

bool SlimeVROutputRuntime::criticalRotationServiceDue(uint32_t nowMs) const {
    if (!enabled_ || !udp_ || !wifi_ || !udp_->active() || !serverFound_ ||
        !wifi_->connected() || trackerHealth_.fatalActive ||
        udpTransportRebindRequested_ || udpReopenRequested_) {
        return false;
    }
    if (serverFoundSendGraceUntilMs_ != 0u &&
        static_cast<int32_t>(nowMs - serverFoundSendGraceUntilMs_) < 0) {
        return false;
    }
    // The first fresh snapshot after a reset is intentionally due immediately.
    return !rotationDeadlineArmed_ ||
           static_cast<int32_t>(nowMs - nextRotationDeadlineMs_) >= 0;
}

bool SlimeVROutputRuntime::updateCritical(uint32_t nowMs) {
    lastRuntimeNowMs_ = nowMs;
    if (!criticalRotationServiceDue(nowMs)) return false;

    // Rotation is the only work permitted here. All connection lifecycle, RX,
    // heartbeat, feature/config, telemetry and error service stays in update().
    // Avoid the full status-signature reduction on this high-frequency path:
    // every due attempt increments rotationSendDue_, including no-snapshot,
    // duplicate-snapshot and transport-failure outcomes.
    const uint32_t dueBefore = rotationSendDue_;
    maybeSendRotation(nowMs);
    return rotationSendDue_ != dueBefore;
}

bool SlimeVROutputRuntime::update(uint32_t nowMs) {
    lastRuntimeNowMs_ = nowMs;
    const SlimeVROutputState stateBefore = state_;
    const uint32_t activityBefore = activitySignature();

    if (!udp_ || !wifi_) return activityChanged(stateBefore, activityBefore);

    if (!enabled_) {
        if (udp_->active()) udp_->stop();
        transitionTo(SlimeVROutputState::Disabled, nowMs);
        return activityChanged(stateBefore, activityBefore);
    }

    const bool wifiConnected = wifi_->connected();
    if (!wifiConnected) {
        if (udp_->active() || serverFound_) ++wifiLostResets_;
        if (udp_->active()) udp_->stop();
        serverFound_ = false;
        serverEndpoint_ = UdpEndpoint{};
        manualServerEndpoint_ = UdpEndpoint{};
        lastDiscoveryAttemptMs_ = 0;
        resetServerFeatureNegotiation();
        acknowledgedSensorInfo_ = SlimeVRSensorInfo{};
        lastSentSensorInfo_ = SlimeVRSensorInfo{};
        markSensorInfoDirty();
        resetTxRecoveryState();
        lastUdpTransportRebindMs_ = 0;
        lastUdpTransportRebindValid_ = false;
        lastUdpFullReopenMs_ = 0;
        lastUdpFullReopenValid_ = false;
        udpTransportRebindRequested_ = false;
        udpReopenRequested_ = false;
        transitionTo(SlimeVROutputState::WaitingForWifi, nowMs);
        return activityChanged(stateBefore, activityBefore);
    }

    ensureUdp(nowMs);
    if (!udp_->active()) return activityChanged(stateBefore, activityBefore);

    serviceTxRecovery(nowMs);

    if (udpTransportRebindRequested_) {
        udpTransportRebindRequested_ = false;
        if (!rebindUdpPreservingSession(nowMs)) {
            requestUdpFullReopen(nowMs, SlimeVRTxRecoveryReason::RebindFailure);
        } else {
            return activityChanged(stateBefore, activityBefore);
        }
    }

    if (udpReopenRequested_) {
        ++udpReopenRequests_;
        udp_->stop();
        serverFound_ = false;
        serverEndpoint_ = UdpEndpoint{};
        manualServerEndpoint_ = UdpEndpoint{};
        lastDiscoveryAttemptMs_ = 0;
        resetServerFeatureNegotiation();
        acknowledgedSensorInfo_ = SlimeVRSensorInfo{};
        lastSentSensorInfo_ = SlimeVRSensorInfo{};
        markSensorInfoDirty();
        resetTxRecoveryState();
        lastUdpTransportRebindMs_ = 0;
        lastUdpTransportRebindValid_ = false;
        lastUdpFullReopenMs_ = nowMs;
        lastUdpFullReopenValid_ = true;
        txPressureState_ = SlimeVRTxPressureState::FullReopenCooldown;
        udpTransportRebindRequested_ = false;
        udpReopenRequested_ = false;
        lastHeartbeatMs_ = 0;
        lastTelemetryMs_ = 0;
        lastServiceUpdateMs_ = 0;
        transitionTo(SlimeVROutputState::UdpStarting, nowMs);
        return activityChanged(stateBefore, activityBefore);
    }

    const bool runService = serviceDue(nowMs);
    if (runService) {
        lastServiceUpdateMs_ = nowMs;
        ++serviceUpdates_;
        pollIncoming(nowMs);
    } else {
        ++serviceSkips_;
    }

    if (serverFound_) {
        transitionTo(SlimeVROutputState::ServerFound, nowMs);
        if (runService && lastIncomingPacketMs_ != 0 && nowMs - lastIncomingPacketMs_ >= SERVER_SILENCE_TIMEOUT_MS) {
            // UDP sends can continue to succeed while the server process was
            // restarted or the old association disappeared. Drop back to
            // discovery when the server has been silent long enough.
            ++serverSilenceResets_;
            serverFound_ = false;
            serverEndpoint_ = UdpEndpoint{};
            manualServerEndpoint_ = UdpEndpoint{};
            lastDiscoveryAttemptMs_ = 0;
            resetServerFeatureNegotiation();
            acknowledgedSensorInfo_ = SlimeVRSensorInfo{};
            lastSentSensorInfo_ = SlimeVRSensorInfo{};
            markSensorInfoDirty();
            lastHeartbeatMs_ = 0;
            lastTelemetryMs_ = 0;
            lastServiceUpdateMs_ = 0;
            transitionTo(SlimeVROutputState::Discovering, nowMs);
            maybeSendDiscovery(nowMs);
            return activityChanged(stateBefore, activityBefore);
        }
        if (serverFoundSendGraceUntilMs_ != 0 && static_cast<int32_t>(nowMs - serverFoundSendGraceUntilMs_) < 0) {
            return activityChanged(stateBefore, activityBefore);
        }
        serverFoundSendGraceUntilMs_ = 0;

        if (runService) {
            if (nowMs - lastHeartbeatMs_ >= HEARTBEAT_INTERVAL_MS) sendHeartbeat(nowMs);
            maybeSendFeatureFlags(nowMs);
            if (sensorInfoSyncState_ != SlimeVRSensorInfoSyncState::Acknowledged &&
                (lastSensorInfoMs_ == 0u || nowMs - lastSensorInfoMs_ >= SENSOR_INFO_INTERVAL_MS)) {
                sendSensorInfo(nowMs);
            }
            maybeSendTelemetry(nowMs);
            maybeSendTrackerError(nowMs);
        }
        if (trackerHealth_.fatalActive) {
            maybeRecordRotationSuppressedByError(nowMs);
        } else {
            maybeSendRotation(nowMs);
        }
        return activityChanged(stateBefore, activityBefore);
    }

    if (runService) {
        transitionTo(SlimeVROutputState::Discovering, nowMs);
        maybeSendDiscovery(nowMs);
    }
    return activityChanged(stateBefore, activityBefore);
}

void SlimeVROutputRuntime::healthCounters(SlimeVROutputHealthCounters& out) const {
    out.sendFailures = sendFailures_;
    out.rotationSendFailures = rotationSendFailures_;
    out.serverSilenceResets = serverSilenceResets_;
    out.wifiLostResets = wifiLostResets_;
}

SlimeVROutputRuntimeStatus SlimeVROutputRuntime::status() const {
    SlimeVROutputRuntimeStatus s;
    s.state = state_;
    s.enabled = enabled_;
    s.wifiConnected = wifi_ && wifi_->connected();
    s.udpReady = udp_ && udp_->active();
    s.serverFound = serverFound_;
    s.discoveryEnabled = discoveryEnabled_;
    s.manualServerEnabled = manualServerEnabled_;
    copyCString(s.manualServerHost, sizeof(s.manualServerHost), manualServerHost_);
    s.manualServerResolved = manualServerEndpoint_.valid();
    s.manualServerIpv4 = manualServerEndpoint_.ipv4;
    s.manualServerResolveAttempts = manualServerResolveAttempts_;
    s.manualServerResolveFailures = manualServerResolveFailures_;
    s.manualServerHandshakesSent = manualServerHandshakesSent_;
    s.localPort = udp_ ? udp_->localPort() : 0;
    s.serverIpv4 = serverEndpoint_.ipv4;
    s.serverPort = serverEndpoint_.port;
    s.sensorId = sensorId_;
    {
        SlimeVRHandshakeInfo info;
        makeHandshakeInfo(info);
        s.protocolVersion = static_cast<uint8_t>(info.protocolVersion);
        s.boardType = info.boardType;
        s.imuType = info.imuType;
        s.mcuType = info.mcuType;
    }
    s.handshakesSent = handshakesSent_;
    s.heartbeatSent = heartbeatSent_;
    s.sensorInfoSent = sensorInfoSent_;
    s.sensorInfoAckReceived = sensorInfoAckReceived_;
    s.sensorInfoAckMalformed = sensorInfoAckMalformed_;
    s.sensorInfoAckMismatch = sensorInfoAckMismatch_;
    s.sensorInfoSyncState = sensorInfoSyncState_;
    s.sensorInfoDirty = sensorInfoDirty_;
    s.sensorInfoLocalStatus = static_cast<uint8_t>(desiredSensorInfo_.sensorState);
    s.sensorInfoLocalConfig = desiredSensorInfo_.sensorConfig;
    s.sensorInfoLocalRestCalibration = desiredSensorInfo_.hasCompletedRestCalibration;
    s.sensorInfoAckStatus = static_cast<uint8_t>(acknowledgedSensorInfo_.sensorState);
    s.sensorInfoAckConfig = acknowledgedSensorInfo_.sensorConfig;
    s.sensorInfoAckRestCalibration = acknowledgedSensorInfo_.hasCompletedRestCalibration;
    s.rotationSent = rotationSent_;
    s.accelerationSent = accelerationSent_;
    s.compactMotionSent = compactMotionSent_;
    s.compactMotionSendFailures = compactMotionSendFailures_;
    s.bundledMotionSent = bundledMotionSent_;
    s.bundledMotionSendFailures = bundledMotionSendFailures_;
    s.accelerationRateLimited = accelerationRateLimited_;
    s.featureFlagsSent = featureFlagsSent_;
    s.featureFlagsSendFailures = featureFlagsSendFailures_;
    s.compactMotionAvailable = true;
    s.compactMotionEnabled = motionPacketPolicy_ ==
        SlimeVRMotionPacketPolicy::RotationAcceleration23;
    s.bundleNegotiationEnabled = TRACKER_SLIMEVR_ENABLE_BUNDLE_NEGOTIATION != 0;
    s.serverFeatureFlagsAvailable = serverFeatureFlagsAvailable_;
    s.featureNegotiationState = featureNegotiationState_;
    s.featureFlagsRequestAttempts = featureFlagsRequestAttempts_;
    s.serverBundleSupported = serverFeatureEnabled(SLIMEVR_SERVER_FEATURE_PROTOCOL_BUNDLE_SUPPORT);
    s.serverCompactBundleSupported = serverFeatureEnabled(SLIMEVR_SERVER_FEATURE_PROTOCOL_BUNDLE_COMPACT_SUPPORT);
    s.motionPacketPolicy = motionPacketPolicy_;
    s.motionPacketMode = motionPacketMode();
    s.bundledMotionEnabled = s.motionPacketMode == SlimeVRMotionPacketMode::Bundle100Rotation17Accel4;
    s.fallbackAccelerationRateHz = TRACKER_SLIMEVR_FALLBACK_ACCEL_RATE_HZ;
    s.accelerationSkippedInvalid = accelerationSkippedInvalid_;
    s.accelerationSkippedConfiguration = accelerationSkippedConfiguration_;
    s.accelerationSkippedComponentMissing = accelerationSkippedComponentMissing_;
    s.accelerationSkippedPairDegraded = accelerationSkippedPairDegraded_;
    s.accelerationSkippedSaturated = accelerationSkippedSaturated_;
    s.accelerationSkippedNonFinite = accelerationSkippedNonFinite_;
    s.accelerationSkippedOther = accelerationSkippedOther_;
    s.accelerationSendFailures = accelerationSendFailures_;
    s.rotationSendDue = rotationSendDue_;
    s.rotationRateLimited = rotationRateLimited_;
    s.rotationMissedDeadlines = rotationMissedDeadlines_;
    s.rotationLateEvents = rotationLateEvents_;
    s.rotationLatenessSumMs = rotationLatenessSumMs_;
    s.rotationLatenessMaxMs = rotationLatenessMaxMs_;
    s.serviceUpdates = serviceUpdates_;
    s.serviceSkips = serviceSkips_;
    s.signalStrengthSent = signalStrengthSent_;
    s.temperatureSent = temperatureSent_;
    s.batterySent = batterySent_;
    s.batterySendFailures = batterySendFailures_;
    s.magnetometerAccuracySent = magnetometerAccuracySent_;
    s.tapSent = tapSent_;
    s.tapSendFailures = tapSendFailures_;
    s.trackerErrorSent = trackerErrorSent_;
    s.trackerErrorSendFailures = trackerErrorSendFailures_;
    s.lastTapValue = lastTapValue_;
    s.rotationNoSnapshot = rotationNoSnapshot_;
    s.rotationDuplicateSnapshot = rotationDuplicateSnapshot_;
    s.rotationSuppressedByError = rotationSuppressedByError_;
    s.packetsReceived = packetsReceived_;
    s.foreignEndpointPacketsDropped = foreignEndpointPacketsDropped_;
    s.preSessionPacketsDropped = preSessionPacketsDropped_;
    s.malformedPackets = malformedPackets_;
    s.malformedDatagramLength = malformedDatagramLength_;
    s.malformedHeartbeat = malformedHeartbeat_;
    s.malformedPing = malformedPing_;
    s.malformedFeatureFlags = malformedFeatureFlags_;
    s.malformedSetConfigFlag = malformedSetConfigFlag_;
    s.malformedProtocolChange = malformedProtocolChange_;
    s.malformedUnknownRaw = malformedUnknownRaw_;
    s.discoveryResponses = discoveryResponses_;
    s.heartbeatReceived = heartbeatReceived_;
    s.pingReceived = pingReceived_;
    s.pongSent = pongSent_;
    s.featureFlagsReceived = featureFlagsReceived_;
    s.setConfigFlagReceived = setConfigFlagReceived_;
    s.setConfigFlagApplied = setConfigFlagApplied_;
    s.setConfigFlagApplyFailures = setConfigFlagApplyFailures_;
    s.setConfigFlagIgnored = setConfigFlagIgnored_;
    s.ackConfigSent = ackConfigSent_;
    s.ackConfigSendFailures = ackConfigSendFailures_;
    s.userActionSent = userActionSent_;
    s.userActionSendFailures = userActionSendFailures_;
    s.lastUserAction = lastUserAction_;
    s.protocolChangeReceived = protocolChangeReceived_;
    s.protocolChangeIgnored = protocolChangeIgnored_;
    s.unknownPacketsReceived = unknownPacketsReceived_;
    s.sendFailures = sendFailures_;
    s.rotationSendFailures = rotationSendFailures_;
    s.controlSendFailures = controlSendFailures_;
    s.telemetrySendFailures = telemetrySendFailures_;
    s.discoverySendFailures = discoverySendFailures_;
    s.tapTransportSendFailures = tapTransportSendFailures_;
    s.udpBeginFailures = udpBeginFailures_;
    s.serverSilenceResets = serverSilenceResets_;
    s.wifiLostResets = wifiLostResets_;
    s.udpReopenRequests = udpReopenRequests_;
    s.udpTransportRebindRequests = udpTransportRebindRequests_;
    s.udpTransportRebindSuccesses = udpTransportRebindSuccesses_;
    s.udpTransportRebindFailures = udpTransportRebindFailures_;
    s.udpFullReopenEscalations = udpFullReopenEscalations_;
    s.txBackoffDrops = txBackoffDrops_;
    s.txPressureFailures = txPressureFailures_;
    s.txOtherFailures = txOtherFailures_;
    s.txFailureWindowTrips = txFailureWindowTrips_;
    s.consecutiveSendFailures = consecutiveSendFailures_;
    s.lastUdpSendError = lastUdpSendError_;
    s.txPressureState = txPressureState_;
    s.txRecoveryReason = txRecoveryReason_;
    s.txPressureEpisodeActive = txPressureEpisodeActive_;
    s.txPressureEpisodeCount = txPressureEpisodeCount_;
    s.txPressureEpisodeDurationMs = txPressureEpisodeActive_
        ? static_cast<uint32_t>(lastRuntimeNowMs_ - txPressureEpisodeStartMs_) : 0u;
    s.txPressureEpisodeMaxMs = txPressureEpisodeMaxMs_;
    s.txPressureStableResets = txPressureStableResets_;
    s.lastSuccessfulMotionTxAgeMs = lastSuccessfulMotionTxValid_
        ? static_cast<uint32_t>(lastRuntimeNowMs_ - lastSuccessfulMotionTxMs_) : 0xFFFFFFFFUL;
    s.successfulMotionTxStreak = successfulMotionTxStreak_;
    s.udpRebindSuppressedCooldown = udpRebindSuppressedCooldown_;
    s.udpFullReopenSuppressedCooldown = udpFullReopenSuppressedCooldown_;
    s.physicalDatagramsSent = physicalDatagramsSent_;
    s.motionDatagramsSent = motionDatagramsSent_;
    s.separateRotationDatagramsSent = separateRotationDatagramsSent_;
    s.separateAccelerationDatagramsSent = separateAccelerationDatagramsSent_;
    s.backgroundControlDatagramsSent = backgroundControlDatagramsSent_;
    s.criticalControlDatagramsSent = criticalControlDatagramsSent_;
    s.motionPacketModeTransitions = motionPacketModeTransitions_;
    s.bundleToSeparateTransitions = bundleToSeparateTransitions_;
    s.separateToBundleTransitions = separateToBundleTransitions_;
    s.accelerationSuppressedDuringNegotiation = accelerationSuppressedDuringNegotiation_;
    s.rotationPhaseOffsetMs = rotationPhaseOffsetMs_;
    s.nextPacketNumber = static_cast<uint32_t>(writer_.packetNumber());
    s.rotationRateHz = rotationRateHz_;
    s.preparedOutputAvailable = copyOutputSnapshot_ != nullptr;
    s.magSupportEnabled = magSupportEnabled_;
    s.magEnabled = magEnabled_;
    s.sensorConfig = sensorConfigFlags();
    s.signalTelemetryEnabled = signalTelemetryEnabled_;
    s.temperatureTelemetryEnabled = temperatureTelemetryEnabled_;
    s.batteryTelemetryEnabled = batteryTelemetryEnabled_;
    s.telemetryIntervalMs = telemetryIntervalMs_;
    s.signalTelemetryIntervalMs = signalTelemetryIntervalMs_;
    s.temperatureTelemetryIntervalMs = temperatureTelemetryIntervalMs_;
    s.batteryTelemetryIntervalMs = batteryTelemetryIntervalMs_;
    s.lastSignalStrengthDbm = lastSignalStrengthDbm_;
    s.lastRssiDbm = lastRssiDbm_;
    s.lastTemperatureC = latestTemperatureC_;
    s.lastTemperatureValid = latestTemperatureValid_;
    s.lastBatteryVoltage = latestBatteryVoltage_;
    s.lastBatteryPercentage = latestBatteryPercentage_;
    s.lastBatteryValid = latestBatteryValid_;
    s.hasCompletedRestCalibration = hasCompletedRestCalibration_;
    s.lastPingId = lastPingId_;
    s.lastServerFeatureFlags = lastServerFeatureFlags_;
    s.lastSetConfigSensorId = lastSetConfigSensorId_;
    s.lastSetConfigType = lastSetConfigType_;
    s.lastSetConfigState = lastSetConfigState_;
    s.lastSetConfigApplied = lastSetConfigApplied_;
    s.lastProtocolTarget = lastProtocolTarget_;
    s.lastProtocolVersion = lastProtocolVersion_;
    s.lastUnknownPacketType = lastUnknownPacketType_;
    s.lastHandshakeMs = lastHandshakeMs_;
    s.lastIncomingPacketMs = lastIncomingPacketMs_;
    s.lastStateChangeMs = lastStateChangeMs_;
    s.lastRotationMs = lastRotationMs_;
    s.lastRotationSnapshotSequence = lastRotationSnapshotSequence_;
    s.lastRotationRuntimeSample = lastRotationRuntimeSample_;
    s.lastRotationTimestampUs = lastRotationTimestampUs_;
    s.lastRotationQualityFlags = lastRotationQualityFlags_;
    s.lastRotationConfidence = lastRotationConfidence_;
    s.lastRotationSnapshotAgeUs = lastRotationSnapshotAgeUs_;
    s.lastRotationSoftwareAgeUs = lastRotationSoftwareAgeUs_;
    s.trackerErrorActive = trackerHealth_.fatalActive;
    s.trackerDegradedNoImu = trackerHealth_.degradedNoImu;
    s.trackerErrorCode = static_cast<uint8_t>(trackerHealth_.faultCode);
    s.trackerHealthRevision = trackerHealth_.revision;
    copyCString(s.trackerErrorMessage, sizeof(s.trackerErrorMessage), trackerHealth_.message);
    return s;
}

bool SlimeVROutputRuntime::rotationDeadlineSlackMs(uint32_t nowMs,
                                                   uint32_t& outSlackMs) const {
    if (!enabled_ || !serverFound_ || !rotationDeadlineArmed_) {
        outSlackMs = 0xFFFFFFFFUL;
        return false;
    }
    const int32_t remaining = static_cast<int32_t>(nextRotationDeadlineMs_ - nowMs);
    outSlackMs = remaining > 0 ? static_cast<uint32_t>(remaining) : 0u;
    return true;
}

void SlimeVROutputRuntime::transitionTo(SlimeVROutputState state, uint32_t nowMs) {
    if (state_ == state) return;
    state_ = state;
    lastStateChangeMs_ = nowMs;
}

void SlimeVROutputRuntime::resetConnectionState(bool keepCounters) {
    state_ = enabled_ ? SlimeVROutputState::WaitingForWifi : SlimeVROutputState::Disabled;
    serverFound_ = false;
    serverEndpoint_ = UdpEndpoint{};
    manualServerEndpoint_ = UdpEndpoint{};
    lastHandshakeMs_ = 0;
    lastDiscoveryAttemptMs_ = 0;
    lastIncomingPacketMs_ = 0;
    lastHeartbeatMs_ = 0;
    lastSensorInfoMs_ = 0;
    desiredSensorInfo_ = desiredSensorInfo();
    lastSentSensorInfo_ = SlimeVRSensorInfo{};
    acknowledgedSensorInfo_ = SlimeVRSensorInfo{};
    sensorInfoSyncState_ = SlimeVRSensorInfoSyncState::Dirty;
    sensorInfoDirty_ = true;
    resetRotationDeadline();
    lastRotationMs_ = 0;
    lastTelemetryMs_ = 0;
    lastServiceUpdateMs_ = 0;
    lastSignalTelemetryMs_ = 0;
    lastTemperatureTelemetryMs_ = 0;
    lastBatteryTelemetryMs_ = 0;
    lastTrackerErrorMs_ = 0;
    lastSignalStrengthDbm_ = 0;
    lastRssiDbm_ = 0;
    lastRotationSnapshotSequence_ = 0;
    lastRotationRuntimeSample_ = 0;
    lastRotationTimestampUs_ = 0;
    lastRotationQualityFlags_ = 0;
    lastRotationConfidence_ = 0.0f;
    lastRotationSnapshotAgeUs_ = 0;
    lastRotationSoftwareAgeUs_ = 0;
    lastPingId_ = 0;
    resetServerFeatureNegotiation();
    lastSetConfigSensorId_ = 0;
    lastSetConfigType_ = 0;
    lastSetConfigState_ = false;
    lastSetConfigApplied_ = false;
    lastProtocolTarget_ = 0;
    lastProtocolVersion_ = 0;
    lastUnknownPacketType_ = 0;
    nextUdpBeginRetryMs_ = 0;
    serverFoundSendGraceUntilMs_ = 0;
    resetTxRecoveryState();
    lastUdpTransportRebindMs_ = 0;
    lastUdpTransportRebindValid_ = false;
    lastUdpFullReopenMs_ = 0;
    lastUdpFullReopenValid_ = false;
    rotationPhaseOffsetValid_ = false;
    lastObservedMotionPacketModeValid_ = false;
    udpReopenRequested_ = false;
    writer_.resetPacketNumber(0);
    if (!keepCounters) resetCounters();
}

void SlimeVROutputRuntime::resetServerFeatureNegotiation() {
    lastServerFeatureFlags_ = 0;
    std::memset(serverFeatureFlags_, 0, sizeof(serverFeatureFlags_));
    serverFeatureFlagsLength_ = 0;
    serverFeatureFlagsAvailable_ = false;
    featureNegotiationState_ = SlimeVRFeatureNegotiationState::NotStarted;
    featureFlagsRequestAttempts_ = 0;
    lastFeatureFlagsRequestMs_ = 0;
    resetFallbackAccelerationDeadline();
}

void SlimeVROutputRuntime::ensureUdp(uint32_t nowMs) {
    if (!udp_) return;
    if (udp_->active()) return;
    transitionTo(SlimeVROutputState::UdpStarting, nowMs);
    if (static_cast<int32_t>(nowMs - nextUdpBeginRetryMs_) < 0) return;
    if (!udp_->begin(localPort_)) {
        ++udpBeginFailures_;
        nextUdpBeginRetryMs_ = nowMs + UDP_BEGIN_RETRY_MS;
        transitionTo(SlimeVROutputState::Error, nowMs);
    }
}

void SlimeVROutputRuntime::pollIncoming(uint32_t nowMs) {
    if (!udp_) return;
    for (uint8_t i = 0; i < incomingPacketsPerUpdate_; ++i) {
        const int packetSize = udp_->parsePacket();
        if (packetSize <= 0) return;

        const int len = udp_->read(incomingBuffer_, sizeof(incomingBuffer_));
        const UdpEndpoint remote = udp_->remoteEndpoint();
        ++packetsReceived_;
        if (len <= 0 || len != packetSize ||
            static_cast<size_t>(packetSize) > sizeof(incomingBuffer_)) {
            ++malformedDatagramLength_;
            ++malformedPackets_;
            continue;
        }

        (void)handleIncomingPacket(incomingBuffer_, static_cast<size_t>(len), remote, nowMs);
    }
}

bool SlimeVROutputRuntime::handleIncomingPacket(const uint8_t* data,
                                                size_t len,
                                                const UdpEndpoint& remote,
                                                uint32_t nowMs) {
    if (SlimeVRPacketWriter::isServerHandshakeResponse(data, len)) {
        // With broadcast discovery disabled, a manual endpoint is exclusive:
        // only the resolved host/port may establish the session.
        if (!serverFound_ && manualServerEnabled_ && !discoveryEnabled_) {
            if (!manualServerEndpoint_.valid() || remote != manualServerEndpoint_) {
                ++foreignEndpointPacketsDropped_;
                return false;
            }
        }

        // Discovery may select a server only while no live endpoint exists. A
        // second server must not hijack an established session.
        if (serverFound_ && serverEndpoint_.valid() && remote != serverEndpoint_) {
            ++foreignEndpointPacketsDropped_;
            return false;
        }

        const bool newSession = !serverFound_ || !serverEndpoint_.valid() || remote != serverEndpoint_;
        if (newSession) {
            resetServerFeatureNegotiation();
            acknowledgedSensorInfo_ = SlimeVRSensorInfo{};
            lastSentSensorInfo_ = SlimeVRSensorInfo{};
            markSensorInfoDirty();
        }
        serverFound_ = true;
        serverEndpoint_ = remote;
        if (serverEndpoint_.port == 0) serverEndpoint_.port = serverPort_;
        ++discoveryResponses_;
        transitionTo(SlimeVROutputState::ServerFound, nowMs);
        serverFoundSendGraceUntilMs_ = nowMs + SERVER_FOUND_SEND_GRACE_MS;
        lastHeartbeatMs_ = nowMs;
        lastTelemetryMs_ = nowMs;
        lastSignalTelemetryMs_ = nowMs;
        lastTemperatureTelemetryMs_ = nowMs;
        lastBatteryTelemetryMs_ = nowMs;
        armRotationDeadline(serverFoundSendGraceUntilMs_);
        lastTrackerErrorMs_ = 0;
        lastIncomingPacketMs_ = nowMs;
        return true;
    }

    if (!data || len == 0) return false;
    if (!serverFound_ || !serverEndpoint_.valid()) {
        ++preSessionPacketsDropped_;
        return false;
    }
    if (remote != serverEndpoint_) {
        ++foreignEndpointPacketsDropped_;
        return false;
    }

    uint8_t ackSensorId = 0;
    uint8_t ackStatus = 0;
    if (parseShortSensorInfoAck(data, len, ackSensorId, ackStatus)) {
        const bool valid = handleSensorInfoAck(data, len);
        if (valid) lastIncomingPacketMs_ = nowMs;
        return valid;
    }

    SlimeVRIncomingPacketView packet;
    if (!parseIncomingPacketView(data, len, packet)) {
        ++malformedUnknownRaw_;
        ++malformedPackets_;
        return false;
    }

    bool valid = false;
    switch (static_cast<SlimeVRReceivePacketType>(packet.type)) {
        case SlimeVRReceivePacketType::HeartBeat0:
        case SlimeVRReceivePacketType::HeartBeat:
            valid = handleHeartbeat(packet.payload, packet.payloadLen, nowMs);
            break;
        case SlimeVRReceivePacketType::PingPong:
            valid = handlePingPong(packet.payload, packet.payloadLen);
            break;
        case SlimeVRReceivePacketType::FeatureFlags:
            valid = handleFeatureFlags(packet.payload, packet.payloadLen);
            break;
        case SlimeVRReceivePacketType::SetConfigFlag:
            valid = handleSetConfigFlag(packet.payload, packet.payloadLen, nowMs);
            break;
        case SlimeVRReceivePacketType::ProtocolChange:
            valid = handleProtocolChange(packet.payload, packet.payloadLen);
            break;
        case SlimeVRReceivePacketType::Handshake:
            // A headered handshake from the selected endpoint is tolerated but
            // does not replace discovery/rebind semantics.
            valid = packet.headered && packet.payloadLen == 0u;
            break;
        case SlimeVRReceivePacketType::SensorInfo:
            // SensorInfo ACK must use the special short packet.
            ++sensorInfoAckMalformed_;
            ++malformedPackets_;
            valid = false;
            break;
        default:
            if (packet.headered) {
                ++unknownPacketsReceived_;
                lastUnknownPacketType_ = packet.type;
                valid = true;
            } else {
                ++malformedUnknownRaw_;
                ++malformedPackets_;
            }
            break;
    }

    if (valid) lastIncomingPacketMs_ = nowMs;
    return valid;
}

bool SlimeVROutputRuntime::handleSensorInfoAck(const uint8_t* data, size_t len) {
    uint8_t ackSensorId = 0;
    uint8_t ackStatus = 0;
    if (!parseShortSensorInfoAck(data, len, ackSensorId, ackStatus)) {
        ++sensorInfoAckMalformed_;
        ++malformedPackets_;
        return false;
    }

    ++sensorInfoAckReceived_;
    const uint8_t sentStatus = static_cast<uint8_t>(lastSentSensorInfo_.sensorState);
    if (sensorInfoSyncState_ != SlimeVRSensorInfoSyncState::WaitingForAck ||
        ackSensorId != lastSentSensorInfo_.sensorId ||
        ackStatus != sentStatus) {
        ++sensorInfoAckMismatch_;
        return true;
    }

    acknowledgedSensorInfo_ = lastSentSensorInfo_;
    desiredSensorInfo_ = desiredSensorInfo();
    if (sensorInfoEqual(desiredSensorInfo_, acknowledgedSensorInfo_)) {
        sensorInfoDirty_ = false;
        sensorInfoSyncState_ = SlimeVRSensorInfoSyncState::Acknowledged;
    } else {
        sensorInfoDirty_ = true;
        sensorInfoSyncState_ = SlimeVRSensorInfoSyncState::Dirty;
        lastSensorInfoMs_ = 0;
    }
    return true;
}

bool SlimeVROutputRuntime::handleHeartbeat(const uint8_t* data, size_t len, uint32_t nowMs) {
    (void)data;
    if (len != 0u) {
        ++malformedHeartbeat_;
        ++malformedPackets_;
        return false;
    }
    ++heartbeatReceived_;
    sendHeartbeat(nowMs);
    return true;
}

bool SlimeVROutputRuntime::handlePingPong(const uint8_t* data, size_t len) {
    if (!serverEndpoint_.valid() || !data || len < 4u) {
        ++malformedPing_;
        ++malformedPackets_;
        return false;
    }
    const uint32_t pingId = readU32Be(data);
    const SlimeVRPacketWriteResult packet = writer_.writePingPong(packetBuffer_, sizeof(packetBuffer_), pingId);
    ++pingReceived_;
    lastPingId_ = pingId;
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Control) == PacketSendResult::Sent) {
        ++pongSent_;
    }
    return true;
}

bool SlimeVROutputRuntime::handleFeatureFlags(const uint8_t* data, size_t len) {
    ++featureFlagsReceived_;
    if (!data || len == 0u) {
        ++malformedFeatureFlags_;
        ++malformedPackets_;
        return false;
    }

    const SlimeVRMotionPacketMode previousMode = motionPacketMode();
    std::memset(serverFeatureFlags_, 0, sizeof(serverFeatureFlags_));
    serverFeatureFlagsLength_ = static_cast<uint8_t>(len < sizeof(serverFeatureFlags_) ? len : sizeof(serverFeatureFlags_));
    std::memcpy(serverFeatureFlags_, data, serverFeatureFlagsLength_);
    serverFeatureFlagsAvailable_ = true;
    featureNegotiationState_ = SlimeVRFeatureNegotiationState::Negotiated;

    uint32_t packed = 0;
    for (uint8_t i = 0; i < serverFeatureFlagsLength_; ++i) {
        packed = (packed << 8) | static_cast<uint32_t>(serverFeatureFlags_[i]);
    }
    lastServerFeatureFlags_ = packed;
    if (motionPacketMode() != previousMode) resetFallbackAccelerationDeadline();
    return true;
}

bool SlimeVROutputRuntime::handleSetConfigFlag(const uint8_t* data, size_t len, uint32_t nowMs) {
    if (!data || len < 4u) {
        ++malformedSetConfigFlag_;
        ++malformedPackets_;
        return false;
    }

    const uint8_t targetSensorId = data[0];
    const uint16_t configType = readU16Be(data + 1);
    const bool requestedState = data[3] != 0;

    ++setConfigFlagReceived_;
    lastSetConfigSensorId_ = targetSensorId;
    lastSetConfigType_ = configType;
    lastSetConfigState_ = requestedState;
    lastSetConfigApplied_ = false;

    const bool targetsThisSensor = targetSensorId == sensorId_ || targetSensorId == SLIMEVR_SENSOR_ID_GLOBAL;
    if (!targetsThisSensor || configType != SLIMEVR_CONFIG_TYPE_MAGNETOMETER) {
        ++setConfigFlagIgnored_;
        return true;
    }

    // A server repeats packet 25 until packet 24 arrives. If the persistent
    // transition succeeded but the ACK itself was lost, acknowledge the retry
    // without rewriting NVS or restarting the magnetic runtime.
    const bool alreadyApplied = magSupportEnabled_ && magEnabled_ == requestedState;
    if (!alreadyApplied) {
        if (!setConfigFlag_) {
            ++setConfigFlagApplyFailures_;
            return true;
        }
        if (!setConfigFlag_(sensorId_, configType, requestedState, setConfigFlagUser_)) {
            ++setConfigFlagApplyFailures_;
            return true;
        }

        magSupportEnabled_ = true;
        magEnabled_ = requestedState;
        markSensorInfoDirty();
        // Defer the state refresh to the bounded retry interval so RX handling
        // emits only the required acknowledgement in this service slice.
        lastSensorInfoMs_ = nowMs;
        ++setConfigFlagApplied_;
    }

    lastSetConfigApplied_ = true;
    (void)sendAckConfigChange(targetSensorId, configType);
    // Do not force a burst in the same RX callback. The service scheduler sends
    // the dirty SensorInfo on its normal bounded path.
    (void)nowMs;
    return true;
}

bool SlimeVROutputRuntime::handleProtocolChange(const uint8_t* data, size_t len) {
    if (!data || len < 2u) {
        ++malformedProtocolChange_;
        ++malformedPackets_;
        return false;
    }
    ++protocolChangeReceived_;
    ++protocolChangeIgnored_;
    lastProtocolTarget_ = data[0];
    lastProtocolVersion_ = data[1];
    return true;
}

bool SlimeVROutputRuntime::sendAckConfigChange(uint8_t targetSensorId, uint16_t configType) {
    if (!serverEndpoint_.valid()) return false;
    const SlimeVRPacketWriteResult packet = writer_.writeAcknowledgeConfigChange(
        packetBuffer_, sizeof(packetBuffer_), targetSensorId, configType
    );
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Control) == PacketSendResult::Sent) {
        ++ackConfigSent_;
        return true;
    }
    ++ackConfigSendFailures_;
    return false;
}

void SlimeVROutputRuntime::maybeSendDiscovery(uint32_t nowMs) {
    if (!discoveryEnabled_ && !manualServerEnabled_) return;
    if (lastDiscoveryAttemptMs_ != 0 &&
        static_cast<uint32_t>(nowMs - lastDiscoveryAttemptMs_) < discoveryIntervalMs_) return;
    lastDiscoveryAttemptMs_ = nowMs;

    if (manualServerEnabled_ && manualServerHost_[0] != '\0') {
        if (!manualServerEndpoint_.valid()) {
            ++manualServerResolveAttempts_;
            uint32_t resolvedIpv4 = 0;
            if (udp_ && udp_->resolveHost(manualServerHost_, resolvedIpv4)) {
                manualServerEndpoint_.ipv4 = resolvedIpv4;
                manualServerEndpoint_.port = serverPort_;
            } else {
                ++manualServerResolveFailures_;
            }
        }
        if (manualServerEndpoint_.valid()) {
            const uint32_t handshakesBefore = handshakesSent_;
            sendHandshakeTo(manualServerEndpoint_, nowMs);
            if (handshakesSent_ != handshakesBefore) ++manualServerHandshakesSent_;
        }
    }

    if (discoveryEnabled_) {
        UdpEndpoint broadcast;
        broadcast.ipv4 = SLIMEVR_DISCOVERY_BROADCAST_IPV4;
        broadcast.port = serverPort_;
        sendHandshakeTo(broadcast, nowMs);
    }
}

void SlimeVROutputRuntime::sendHandshakeTo(const UdpEndpoint& endpoint, uint32_t nowMs) {
    SlimeVRHandshakeInfo info;
    makeHandshakeInfo(info);
    const SlimeVRPacketWriteResult packet = writer_.writeHandshake(packetBuffer_, sizeof(packetBuffer_), info);
    if (sendPacket(packet, endpoint, PacketPurpose::Discovery) == PacketSendResult::Sent) {
        ++handshakesSent_;
        lastHandshakeMs_ = nowMs;
    }
}

void SlimeVROutputRuntime::sendSensorInfo(uint32_t nowMs) {
    if (!serverEndpoint_.valid()) return;
    desiredSensorInfo_ = desiredSensorInfo();
    if (!sensorInfoDirty_ && sensorInfoSyncState_ == SlimeVRSensorInfoSyncState::Acknowledged) return;

    const SlimeVRPacketWriteResult packet = writer_.writeSensorInfo(
        packetBuffer_, sizeof(packetBuffer_), desiredSensorInfo_
    );
    const PacketSendResult result = sendPacket(
        packet, serverEndpoint_, PacketPurpose::ControlBackground
    );
    if (result == PacketSendResult::BackoffSuppressed) return;
    lastSensorInfoMs_ = nowMs;
    if (result == PacketSendResult::Sent) {
        ++sensorInfoSent_;
        lastSentSensorInfo_ = desiredSensorInfo_;
        sensorInfoSyncState_ = SlimeVRSensorInfoSyncState::WaitingForAck;
        sensorInfoDirty_ = true;
    }
}

void SlimeVROutputRuntime::sendHeartbeat(uint32_t nowMs) {
    if (!serverEndpoint_.valid()) return;
    const SlimeVRPacketWriteResult packet = writer_.writeHeartbeat(packetBuffer_, sizeof(packetBuffer_));
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::ControlBackground) == PacketSendResult::Sent) {
        ++heartbeatSent_;
        lastHeartbeatMs_ = nowMs;
    }
}

void SlimeVROutputRuntime::maybeSendFeatureFlags(uint32_t nowMs) {
#if TRACKER_SLIMEVR_ENABLE_BUNDLE_NEGOTIATION
    if (!serverEndpoint_.valid()) return;
    if (featureNegotiationState_ == SlimeVRFeatureNegotiationState::Negotiated ||
        featureNegotiationState_ == SlimeVRFeatureNegotiationState::Unavailable) return;

    if (featureFlagsRequestAttempts_ >= TRACKER_SLIMEVR_FEATURE_FLAGS_REQUEST_ATTEMPTS) {
        featureNegotiationState_ = SlimeVRFeatureNegotiationState::Unavailable;
        return;
    }
    if (lastFeatureFlagsRequestMs_ != 0u &&
        nowMs - lastFeatureFlagsRequestMs_ < TRACKER_SLIMEVR_FEATURE_FLAGS_REQUEST_INTERVAL_MS) return;

    const uint8_t flags[] = {SLIMEVR_FIRMWARE_FEATURE_FLAGS};
    const SlimeVRPacketWriteResult packet = writer_.writeFeatureFlags(
        packetBuffer_, sizeof(packetBuffer_), flags, sizeof(flags)
    );
    const PacketSendResult result = sendPacket(
        packet, serverEndpoint_, PacketPurpose::ControlBackground
    );
    if (result == PacketSendResult::BackoffSuppressed) return;
    lastFeatureFlagsRequestMs_ = nowMs;
    if (result == PacketSendResult::Sent) {
        ++featureFlagsRequestAttempts_;
        ++featureFlagsSent_;
        featureNegotiationState_ = SlimeVRFeatureNegotiationState::Waiting;
    } else {
        ++featureFlagsSendFailures_;
    }
#else
    (void)nowMs;
    featureNegotiationState_ = SlimeVRFeatureNegotiationState::Unavailable;
#endif
}

void SlimeVROutputRuntime::maybeSendTelemetry(uint32_t nowMs) {
    if (!serverEndpoint_.valid()) return;

    if (signalTelemetryEnabled_ && signalTelemetryIntervalMs_ != 0 &&
        (lastSignalTelemetryMs_ == 0 || nowMs - lastSignalTelemetryMs_ >= signalTelemetryIntervalMs_)) {
        lastSignalTelemetryMs_ = nowMs;
        sendSignalStrength(nowMs);
    }

    if (temperatureTelemetryEnabled_ && temperatureTelemetryIntervalMs_ != 0 &&
        (lastTemperatureTelemetryMs_ == 0 || nowMs - lastTemperatureTelemetryMs_ >= temperatureTelemetryIntervalMs_)) {
        lastTemperatureTelemetryMs_ = nowMs;
        sendTemperature(nowMs);
    }

    if (batteryTelemetryEnabled_ && batteryTelemetryIntervalMs_ != 0 &&
        (lastBatteryTelemetryMs_ == 0 || nowMs - lastBatteryTelemetryMs_ >= batteryTelemetryIntervalMs_)) {
        lastBatteryTelemetryMs_ = nowMs;
        sendBatteryLevel(nowMs);
    }
    // Do not periodically send MagnetometerAccuracy as generic telemetry. The
    // server learns magnetometer support from SensorInfo.sensorConfig. Some
    // server builds treat packet 18 as active mag-calibration feedback, so
    // sending dummy accuracy values can disturb preview/tracker state.
}

bool SlimeVROutputRuntime::serviceDue(uint32_t nowMs) const {
#if TRACKER_SLIMEVR_SERVICE_UPDATE_INTERVAL_MS == 0
    (void)nowMs;
    return true;
#else
    return lastServiceUpdateMs_ == 0 ||
           static_cast<uint32_t>(nowMs - lastServiceUpdateMs_) >= TRACKER_SLIMEVR_SERVICE_UPDATE_INTERVAL_MS;
#endif
}

uint32_t SlimeVROutputRuntime::activitySignature() const {
    // This intentionally excludes rotationRateLimited_ and serviceSkips_: they
    // are useful diagnostics, but they represent no packet/state work and would
    // make every scheduler tick look busy to the app-level idle detector.
    return handshakesSent_ + heartbeatSent_ + sensorInfoSent_ + rotationSent_ +
           accelerationSent_ + compactMotionSent_ + compactMotionSendFailures_ +
           bundledMotionSent_ + bundledMotionSendFailures_ + accelerationRateLimited_ +
           featureFlagsSent_ + featureFlagsSendFailures_ +
           accelerationSkippedInvalid_ + accelerationSendFailures_ +
           rotationSendDue_ + signalStrengthSent_ + temperatureSent_ + batterySent_ +
           batterySendFailures_ + magnetometerAccuracySent_ + tapSent_ + tapSendFailures_ +
           trackerErrorSent_ + trackerErrorSendFailures_ +
           rotationNoSnapshot_ + rotationDuplicateSnapshot_ + packetsReceived_ +
           discoveryResponses_ + heartbeatReceived_ + pingReceived_ + pongSent_ +
           featureFlagsReceived_ + setConfigFlagReceived_ + setConfigFlagApplied_ +
           setConfigFlagApplyFailures_ + setConfigFlagIgnored_ + ackConfigSent_ +
           ackConfigSendFailures_ + userActionSent_ + userActionSendFailures_ +
           protocolChangeReceived_ + protocolChangeIgnored_ + unknownPacketsReceived_ +
           malformedPackets_ + malformedDatagramLength_ + sensorInfoAckReceived_ + sensorInfoAckMalformed_ +
           sensorInfoAckMismatch_ + sendFailures_ + rotationSendFailures_ +
           controlSendFailures_ + telemetrySendFailures_ + discoverySendFailures_ +
           tapTransportSendFailures_ + udpBeginFailures_ + serverSilenceResets_ +
           wifiLostResets_ + udpReopenRequests_ + udpTransportRebindRequests_ +
           udpTransportRebindSuccesses_ + udpTransportRebindFailures_ +
           udpFullReopenEscalations_ + txBackoffDrops_ + txPressureFailures_ +
           txOtherFailures_ + txFailureWindowTrips_ + consecutiveSendFailures_ +
           txPressureStableResets_ + udpRebindSuppressedCooldown_ +
           udpFullReopenSuppressedCooldown_ + serviceUpdates_;
}

bool SlimeVROutputRuntime::activityChanged(SlimeVROutputState previousState,
                                           uint32_t previousSignature) const {
    return state_ != previousState || activitySignature() != previousSignature;
}

bool SlimeVROutputRuntime::sendTap(uint8_t value) {
    if (!serverFound_ || !serverEndpoint_.valid()) {
        ++tapSendFailures_;
        return false;
    }
    const SlimeVRPacketWriteResult packet = writer_.writeTap(
        packetBuffer_,
        sizeof(packetBuffer_),
        sensorId_,
        value
    );
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Tap) == PacketSendResult::Sent) {
        ++tapSent_;
        lastTapValue_ = value;
        return true;
    }
    ++tapSendFailures_;
    return false;
}

bool SlimeVROutputRuntime::sendUserAction(SlimeVRUserAction action) {
    lastUserAction_ = action;
    if (action == SlimeVRUserAction::None || !serverFound_ || !serverEndpoint_.valid()) {
        ++userActionSendFailures_;
        return false;
    }
    const SlimeVRPacketWriteResult packet = writer_.writeUserAction(
        packetBuffer_, sizeof(packetBuffer_), action
    );
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::UserAction) == PacketSendResult::Sent) {
        ++userActionSent_;
        return true;
    }
    ++userActionSendFailures_;
    return false;
}

void SlimeVROutputRuntime::sendSignalStrength(uint32_t nowMs) {
    (void)nowMs;
    if (!serverEndpoint_.valid() || !wifi_) return;
    const TrackerWifiManagerStatus ws = wifi_->status();
    const int8_t signalDbm = signalStrengthFromRssi(ws.rssiDbm);
    const SlimeVRPacketWriteResult packet = writer_.writeSignalStrength(
        packetBuffer_,
        sizeof(packetBuffer_),
        sensorId_,
        signalDbm
    );
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Telemetry) == PacketSendResult::Sent) {
        ++signalStrengthSent_;
        lastSignalStrengthDbm_ = signalDbm;
        lastRssiDbm_ = ws.rssiDbm;
    }
}

void SlimeVROutputRuntime::sendTemperature(uint32_t nowMs) {
    (void)nowMs;
    if (!serverEndpoint_.valid() || !latestTemperatureValid_) return;
    const SlimeVRPacketWriteResult packet = writer_.writeTemperature(
        packetBuffer_,
        sizeof(packetBuffer_),
        sensorId_,
        latestTemperatureC_
    );
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Telemetry) == PacketSendResult::Sent) {
        ++temperatureSent_;
    }
}

void SlimeVROutputRuntime::sendBatteryLevel(uint32_t nowMs) {
    (void)nowMs;
    if (!serverEndpoint_.valid()) return;
    if (!latestBatteryValid_) return;
    // SlimeVR BatteryLevel uses voltage in volts and battery level as a
    // normalized fraction (0.0-1.0). Keep internal/CLI state as human-readable
    // percent (0-100) and only convert at the protocol boundary.
    float protocolBatteryFraction = latestBatteryPercentage_ / 100.0f;
    if (!std::isfinite(protocolBatteryFraction) || protocolBatteryFraction < 0.0f) protocolBatteryFraction = 0.0f;
    if (protocolBatteryFraction > 1.0f) protocolBatteryFraction = 1.0f;
    const SlimeVRPacketWriteResult packet = writer_.writeBatteryLevel(
        packetBuffer_,
        sizeof(packetBuffer_),
        latestBatteryVoltage_,
        protocolBatteryFraction
    );
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Telemetry) == PacketSendResult::Sent) {
        ++batterySent_;
        return;
    }
    ++batterySendFailures_;
}

void SlimeVROutputRuntime::sendMagnetometerAccuracy(uint32_t nowMs) {
    (void)nowMs;
    if (!serverEndpoint_.valid()) return;
    const SlimeVRPacketWriteResult packet = writer_.writeMagnetometerAccuracy(
        packetBuffer_,
        sizeof(packetBuffer_),
        sensorId_,
        0.0f
    );
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Telemetry) == PacketSendResult::Sent) {
        ++magnetometerAccuracySent_;
    }
}

void SlimeVROutputRuntime::maybeSendTrackerError(uint32_t nowMs) {
    if (!trackerHealth_.fatalActive) return;
    if (!serverEndpoint_.valid()) return;
    if (lastTrackerErrorMs_ != 0 && nowMs - lastTrackerErrorMs_ < TRACKER_ERROR_INTERVAL_MS) return;

    const SlimeVRPacketWriteResult packet = writer_.writeError(
        packetBuffer_,
        sizeof(packetBuffer_),
        sensorId_,
        static_cast<uint8_t>(trackerHealth_.faultCode),
        trackerHealth_.message
    );
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::ErrorReport) == PacketSendResult::Sent) {
        ++trackerErrorSent_;
        lastTrackerErrorMs_ = nowMs;
        return;
    }
    ++trackerErrorSendFailures_;
}

void SlimeVROutputRuntime::maybeRecordRotationSuppressedByError(uint32_t nowMs) {
    if (!consumeRotationDeadline(nowMs)) return;
    ++rotationSuppressedByError_;
}

SlimeVRMotionPacketMode SlimeVROutputRuntime::motionPacketMode() const {
    switch (motionPacketPolicy_) {
        case SlimeVRMotionPacketPolicy::QuaternionOnly:
            return SlimeVRMotionPacketMode::Rotation17Only;
        case SlimeVRMotionPacketPolicy::RotationAcceleration23:
            return SlimeVRMotionPacketMode::ExperimentalRotationAcceleration23;
        case SlimeVRMotionPacketPolicy::BundleRotation17Acceleration4:
            if (TRACKER_SLIMEVR_ENABLE_BUNDLE_NEGOTIATION != 0 &&
                serverFeatureEnabled(SLIMEVR_SERVER_FEATURE_PROTOCOL_BUNDLE_SUPPORT)) {
                return SlimeVRMotionPacketMode::Bundle100Rotation17Accel4;
            }
            return SlimeVRMotionPacketMode::SeparateRotation17Accel4;
    }
    return SlimeVRMotionPacketMode::Rotation17Only;
}

bool SlimeVROutputRuntime::serverFeatureEnabled(uint8_t bit) const {
    if (!serverFeatureFlagsAvailable_) return false;
    const uint8_t byteIndex = static_cast<uint8_t>(bit / 8u);
    if (byteIndex >= serverFeatureFlagsLength_) return false;
    return (serverFeatureFlags_[byteIndex] & static_cast<uint8_t>(1u << (bit % 8u))) != 0u;
}

bool SlimeVROutputRuntime::fallbackAccelerationAllowed() const {
#if TRACKER_SLIMEVR_ENABLE_BUNDLE_NEGOTIATION
    return featureNegotiationState_ == SlimeVRFeatureNegotiationState::Negotiated ||
           featureNegotiationState_ == SlimeVRFeatureNegotiationState::Unavailable;
#else
    return true;
#endif
}

void SlimeVROutputRuntime::noteMotionPacketMode(SlimeVRMotionPacketMode mode) {
    if (!lastObservedMotionPacketModeValid_) {
        lastObservedMotionPacketMode_ = mode;
        lastObservedMotionPacketModeValid_ = true;
        return;
    }
    if (lastObservedMotionPacketMode_ == mode) return;
    ++motionPacketModeTransitions_;
    if (lastObservedMotionPacketMode_ == SlimeVRMotionPacketMode::Bundle100Rotation17Accel4 &&
        mode == SlimeVRMotionPacketMode::SeparateRotation17Accel4) {
        ++bundleToSeparateTransitions_;
    } else if (lastObservedMotionPacketMode_ == SlimeVRMotionPacketMode::SeparateRotation17Accel4 &&
               mode == SlimeVRMotionPacketMode::Bundle100Rotation17Accel4) {
        ++separateToBundleTransitions_;
    }
    lastObservedMotionPacketMode_ = mode;
}

void SlimeVROutputRuntime::refreshRotationPhaseOffset() {
    const uint32_t periodMs = rotationPeriodMs();
    if (rotationPhaseOffsetValid_ || periodMs == 0u) return;
    uint32_t hash = 2166136261UL;
    if (wifi_) {
        const TrackerWifiManagerStatus ws = wifi_->status();
        for (uint8_t byte : ws.mac) {
            hash ^= byte;
            hash *= 16777619UL;
        }
    }
    rotationPhaseOffsetMs_ = static_cast<uint16_t>(hash % periodMs);
    rotationPhaseOffsetValid_ = true;
}

void SlimeVROutputRuntime::resetFallbackAccelerationDeadline() {
    nextFallbackAccelerationDeadlineMs_ = 0u;
    fallbackAccelerationDeadlineArmed_ = false;
}

bool SlimeVROutputRuntime::consumeFallbackAccelerationDeadline(uint32_t nowMs) {
    constexpr uint16_t configuredHz = TRACKER_SLIMEVR_FALLBACK_ACCEL_RATE_HZ;
    if (configuredHz == 0u) return false;
    const uint16_t effectiveHz = configuredHz < rotationRateHz_ ? configuredHz : rotationRateHz_;
    if (effectiveHz == 0u) return false;
    const uint32_t periodMs = (1000UL + effectiveHz - 1u) / effectiveHz;
    if (!fallbackAccelerationDeadlineArmed_) {
        nextFallbackAccelerationDeadlineMs_ = nowMs + periodMs;
        fallbackAccelerationDeadlineArmed_ = true;
        return true;
    }
    if (static_cast<int32_t>(nowMs - nextFallbackAccelerationDeadlineMs_) < 0) return false;
    const uint32_t latenessMs = nowMs - nextFallbackAccelerationDeadlineMs_;
    nextFallbackAccelerationDeadlineMs_ += (1u + latenessMs / periodMs) * periodMs;
    return true;
}

void SlimeVROutputRuntime::resetRotationDeadline() {
    nextRotationDeadlineMs_ = 0u;
    rotationDeadlineArmed_ = false;
}

void SlimeVROutputRuntime::armRotationDeadline(uint32_t deadlineMs) {
    nextRotationDeadlineMs_ = deadlineMs;
    rotationDeadlineArmed_ = true;
}

bool SlimeVROutputRuntime::consumeRotationDeadline(uint32_t nowMs) {
    const uint32_t periodMs = rotationPeriodMs();
    if (!rotationDeadlineArmed_) {
        // Preserve immediate first-pose delivery, then distribute the steady
        // datagram phase by MAC so simultaneously started trackers do not all
        // refill the AP/TX queues in the same millisecond.
        refreshRotationPhaseOffset();
        const uint32_t baseDeadline = nowMs + periodMs;
        const uint32_t basePhase = baseDeadline % periodMs;
        const uint32_t phaseAdjust =
            (static_cast<uint32_t>(rotationPhaseOffsetMs_) + periodMs - basePhase) % periodMs;
        armRotationDeadline(baseDeadline + phaseAdjust);
        return true;
    }

    if (static_cast<int32_t>(nowMs - nextRotationDeadlineMs_) < 0) {
        ++rotationRateLimited_;
        return false;
    }

    const uint32_t latenessMs = nowMs - nextRotationDeadlineMs_;
    uint32_t elapsedPeriods = 1u;
    if (latenessMs >= periodMs) {
        elapsedPeriods += latenessMs / periodMs;
    }
    if (elapsedPeriods > 1u) {
        rotationMissedDeadlines_ += elapsedPeriods - 1u;
    }
    if (latenessMs > 0u) {
        ++rotationLateEvents_;
        rotationLatenessSumMs_ += latenessMs;
        if (latenessMs > rotationLatenessMaxMs_) rotationLatenessMaxMs_ = latenessMs;
    }

    // Preserve phase and jump directly to the nearest future deadline. Never
    // emit a burst of stale packets to catch up missed periods.
    nextRotationDeadlineMs_ += elapsedPeriods * periodMs;
    return true;
}

uint32_t SlimeVROutputRuntime::rotationPeriodMs() const {
    const uint16_t hz = rotationRateHz_ == 0 ? ROTATION_RATE_HZ_DEFAULT : rotationRateHz_;
    const uint32_t period = 1000UL / hz;
    return period == 0 ? 1UL : period;
}

uint16_t SlimeVROutputRuntime::sensorConfigFlags() const {
    uint16_t flags = 0;
    if (magSupportEnabled_) {
        flags |= SLIMEVR_SENSOR_CONFIG_MAG_SUPPORTED;
        if (magEnabled_) flags |= SLIMEVR_SENSOR_CONFIG_MAG_ENABLED;
    }
    return flags;
}

uint8_t SlimeVROutputRuntime::accuracyFromConfidence(float confidence) {
    if (confidence >= 0.90f) return 3;
    if (confidence >= 0.65f) return 2;
    if (confidence >= 0.35f) return 1;
    return 0;
}

int8_t SlimeVROutputRuntime::signalStrengthFromRssi(int32_t rssiDbm) {
    if (rssiDbm < -128) rssiDbm = -128;
    if (rssiDbm > 127) rssiDbm = 127;
    return static_cast<int8_t>(rssiDbm);
}

void SlimeVROutputRuntime::maybeSendRotation(uint32_t nowMs) {
    if (!serverEndpoint_.valid()) return;
    if (!copyOutputSnapshot_) return;
    if (!consumeRotationDeadline(nowMs)) return;
    ++rotationSendDue_;

    TrackerPreparedOutputSnapshot snapshot;
    if (!copyOutputSnapshot_(snapshot, copyOutputSnapshotUser_) || !snapshot.valid) {
        ++rotationNoSnapshot_;
        return;
    }

    if (snapshot.sequence == lastRotationSnapshotSequence_) {
        ++rotationDuplicateSnapshot_;
        return;
    }

    sendRotation(snapshot, nowMs);
}

void SlimeVROutputRuntime::sendRotation(const TrackerPreparedOutputSnapshot& snapshot, uint32_t nowMs) {
    const auto recordDeliveredRotation = [&]() {
        ++rotationSent_;
        lastRotationMs_ = nowMs;
        lastRotationSnapshotSequence_ = snapshot.sequence;
        lastRotationRuntimeSample_ = snapshot.runtimeSample;
        lastRotationTimestampUs_ = snapshot.timestampUs;
        lastRotationQualityFlags_ = snapshot.qualityFlags;
        lastRotationConfidence_ = snapshot.confidence;
        lastRotationSnapshotAgeUs_ = micros() - snapshot.publishedAtMcuUs;
        const uint64_t softwareAge = static_cast<uint64_t>(snapshot.softwareQueueAgeUs) +
                                   lastRotationSnapshotAgeUs_;
        lastRotationSoftwareAgeUs_ = softwareAge > 0xffffffffULL
            ? 0xffffffffu
            : static_cast<uint32_t>(softwareAge);
        if (rotationSoftwareAgeSink_ != nullptr) {
            rotationSoftwareAgeSink_(lastRotationSoftwareAgeUs_, rotationSoftwareAgeSinkUser_);
        }
    };

    const uint8_t accuracy = accuracyFromConfidence(snapshot.confidence);
    const SlimeVRMotionPacketMode mode = motionPacketMode();
    noteMotionPacketMode(mode);

    // Quaternion-only and invalid-acceleration paths never need an SI accel
    // conversion. Keeping this branch before accelerationWireMps2FromDeviceG()
    // makes the new default mode strictly cheaper than the old bundle path.
    if (mode == SlimeVRMotionPacketMode::Rotation17Only ||
        !snapshot.linearAccelerationValid) {
        const SlimeVRPacketWriteResult rotationPacket = writer_.writeRotationData(
            packetBuffer_, sizeof(packetBuffer_), sensorId_, snapshot.q, accuracy,
            SlimeVRRotationDataType::Normal
        );
        if (sendPacket(rotationPacket, serverEndpoint_, PacketPurpose::Rotation) != PacketSendResult::Sent) return;
        recordDeliveredRotation();

        if (mode == SlimeVRMotionPacketMode::Rotation17Only) return;

        ++accelerationSkippedInvalid_;
        const uint8_t reasons = snapshot.linearAccelerationInvalidFlags;
        if ((reasons & prepared_output_motion_flags::CONFIGURATION_NOT_READY) != 0u) ++accelerationSkippedConfiguration_;
        if ((reasons & prepared_output_motion_flags::ACCEL_COMPONENT_MISSING) != 0u) ++accelerationSkippedComponentMissing_;
        if ((reasons & prepared_output_motion_flags::PAIR_COHERENCY_DEGRADED) != 0u) ++accelerationSkippedPairDegraded_;
        if ((reasons & prepared_output_motion_flags::ACCEL_SATURATED) != 0u) ++accelerationSkippedSaturated_;
        if ((reasons & prepared_output_motion_flags::NON_FINITE) != 0u) ++accelerationSkippedNonFinite_;
        if (reasons == prepared_output_motion_flags::NONE) ++accelerationSkippedOther_;
        return;
    }

    const Vec3 accelerationMps2 = slimevr_motion_frame::accelerationWireMps2FromDeviceG(
        snapshot.linearAccelerationDeviceG
    );

    if (mode == SlimeVRMotionPacketMode::ExperimentalRotationAcceleration23) {
        const SlimeVRPacketWriteResult motionPacket = writer_.writeRotationAndAcceleration(
            packetBuffer_, sizeof(packetBuffer_), sensorId_, snapshot.q, accelerationMps2
        );
        if (sendPacket(motionPacket, serverEndpoint_, PacketPurpose::MotionCombined) != PacketSendResult::Sent) return;
        recordDeliveredRotation();
        ++accelerationSent_;
        ++compactMotionSent_;
        return;
    }

    if (mode == SlimeVRMotionPacketMode::Bundle100Rotation17Accel4) {
        const SlimeVRPacketWriteResult bundlePacket = writer_.writeRotationAccelerationBundle(
            packetBuffer_, sizeof(packetBuffer_), sensorId_, snapshot.q, accuracy,
            accelerationMps2, SlimeVRRotationDataType::Normal
        );
        if (sendPacket(bundlePacket, serverEndpoint_, PacketPurpose::MotionCombined) != PacketSendResult::Sent) return;
        recordDeliveredRotation();
        ++accelerationSent_;
        ++bundledMotionSent_;
        return;
    }

    // Negotiation fallback preserves packet 17 pose delivery and rate-limits
    // the paired packet 4 acceleration from the same prepared snapshot.
    const SlimeVRPacketWriteResult rotationPacket = writer_.writeRotationData(
        packetBuffer_, sizeof(packetBuffer_), sensorId_, snapshot.q, accuracy,
        SlimeVRRotationDataType::Normal
    );
    if (sendPacket(rotationPacket, serverEndpoint_, PacketPurpose::Rotation) != PacketSendResult::Sent) return;
    recordDeliveredRotation();

    if (!fallbackAccelerationAllowed()) {
        ++accelerationSuppressedDuringNegotiation_;
        return;
    }

    if (!consumeFallbackAccelerationDeadline(nowMs)) {
        ++accelerationRateLimited_;
        return;
    }

    const SlimeVRPacketWriteResult accelerationPacket = writer_.writeAcceleration(
        packetBuffer_, sizeof(packetBuffer_), sensorId_, accelerationMps2
    );
    if (sendPacket(accelerationPacket, serverEndpoint_, PacketPurpose::Acceleration) == PacketSendResult::Sent) {
        ++accelerationSent_;
    }
}

void SlimeVROutputRuntime::makeHandshakeInfo(SlimeVRHandshakeInfo& info) const {
    info.firmwareVersion = trackerBuildSlimeVRFirmwareVersion();
    info.vendorName = "CustomSlime";
    info.vendorUrl = "https://github.com";
    info.productName = deviceName_;
    info.updateAddress = "https://github.com";
    info.updateName = deviceName_;

    if (wifi_) {
        const TrackerWifiManagerStatus ws = wifi_->status();
        std::memcpy(info.mac, ws.mac, sizeof(info.mac));
    }
}

void SlimeVROutputRuntime::recordSendFailure(PacketPurpose purpose) {
    ++sendFailures_;
    switch (purpose) {
        case PacketPurpose::Discovery:
            ++discoverySendFailures_;
            break;
        case PacketPurpose::Control:
        case PacketPurpose::ControlBackground:
            ++controlSendFailures_;
            break;
        case PacketPurpose::Telemetry:
            ++telemetrySendFailures_;
            break;
        case PacketPurpose::Rotation:
            ++rotationSendFailures_;
            break;
        case PacketPurpose::Acceleration:
            ++accelerationSendFailures_;
            break;
        case PacketPurpose::MotionCombined:
            // One failed bundle or packet 23 loses both logical motion
            // channels, while sendFailures_ remains the physical datagram
            // failure count. Mode-specific counters distinguish the encoder.
            if (motionPacketMode() == SlimeVRMotionPacketMode::ExperimentalRotationAcceleration23) {
                ++compactMotionSendFailures_;
            } else if (motionPacketMode() == SlimeVRMotionPacketMode::Bundle100Rotation17Accel4) {
                ++bundledMotionSendFailures_;
            }
            ++rotationSendFailures_;
            ++accelerationSendFailures_;
            break;
        case PacketPurpose::Tap:
            ++tapTransportSendFailures_;
            break;
        case PacketPurpose::UserAction:
            ++controlSendFailures_;
            break;
        case PacketPurpose::ErrorReport:
            ++controlSendFailures_;
            break;
    }
}

SlimeVROutputRuntime::PacketSendResult SlimeVROutputRuntime::sendPacket(
    const SlimeVRPacketWriteResult& packet,
    const UdpEndpoint& endpoint,
    PacketPurpose purpose) {
    const uint32_t nowMs = lastRuntimeNowMs_ != 0u ? lastRuntimeNowMs_ : millis();
    if (txBackoffActive(nowMs) && packetMayBeDroppedDuringTxBackoff(purpose)) {
        recordSendBackoffDrop(purpose);
        return PacketSendResult::BackoffSuppressed;
    }

    if (!udp_ || !packet.ok || packet.size == 0) {
        lastUdpSendError_ = EINVAL;
        recordSendFailure(purpose);
        ++txOtherFailures_;
        return PacketSendResult::Failed;
    }

    if (!udp_->send(endpoint, packetBuffer_, packet.size)) {
        lastUdpSendError_ = udp_->lastSendError();
        recordSendFailure(purpose);
        const bool pressureFailure = isUdpTxPressureError(lastUdpSendError_);
        if (pressureFailure) {
            ++txPressureFailures_;
            beginTxPressureEpisode(nowMs);
        } else {
            ++txOtherFailures_;
        }
        ++consecutiveSendFailures_;
        const bool denseBefore = txFailureWindowExceeded();
        recordPhysicalSendOutcome(false, purpose, pressureFailure, nowMs);
        const bool denseAfter = txFailureWindowExceeded();
        if (!denseBefore && denseAfter) {
            ++txFailureWindowTrips_;
        }
        armTxBackoff(nowMs);

        // Pressure is recovered by elapsed motion-success age, not by a dense
        // intermittent ENOMEM bitmap. As long as fresh motion still passes,
        // the socket and negotiated session remain intact.
        if (!pressureFailure &&
            consecutiveSendFailures_ >= TRACKER_SLIMEVR_TX_OTHER_REBIND_CONSECUTIVE_FAILURES) {
            const bool recentServerRx = serverFound_ && lastIncomingPacketMs_ != 0u &&
                static_cast<uint32_t>(nowMs - lastIncomingPacketMs_) < TRACKER_SLIMEVR_TX_RECENT_RX_MS;
            if (recentServerRx) {
                requestUdpTransportRebind(nowMs, SlimeVRTxRecoveryReason::NonPressureTransportFailure);
            } else {
                requestUdpFullReopen(nowMs, SlimeVRTxRecoveryReason::NonPressureTransportFailure);
            }
        }
        return PacketSendResult::Failed;
    }

    consecutiveSendFailures_ = 0;
    txBackoffUntilMs_ = 0;
    txBackoffLevel_ = 0;
    recordPhysicalSendOutcome(true, purpose, false, nowMs);
    return PacketSendResult::Sent;
}

void SlimeVROutputRuntime::recordSendBackoffDrop(PacketPurpose purpose) {
    (void)purpose;
    ++txBackoffDrops_;
}

bool SlimeVROutputRuntime::packetPurposeIsMotion(PacketPurpose purpose) {
    return purpose == PacketPurpose::Rotation ||
           purpose == PacketPurpose::Acceleration ||
           purpose == PacketPurpose::MotionCombined;
}

void SlimeVROutputRuntime::recordPhysicalSendOutcome(bool success,
                                                      PacketPurpose purpose,
                                                      bool pressureFailure,
                                                      uint32_t nowMs) {
    constexpr uint32_t oldestBit = 0x80000000UL;
    if (txOutcomeWindowCount_ >= TRACKER_SLIMEVR_TX_FAILURE_WINDOW_ATTEMPTS) {
        if ((txOutcomeWindowBits_ & oldestBit) != 0u && txOutcomeFailureCount_ > 0u) {
            --txOutcomeFailureCount_;
        }
    } else {
        ++txOutcomeWindowCount_;
    }

    txOutcomeWindowBits_ <<= 1u;
    if (!success) {
        txOutcomeWindowBits_ |= 1u;
        ++txOutcomeFailureCount_;
    }

    if (success) {
        ++physicalDatagramsSent_;
        switch (purpose) {
            case PacketPurpose::Rotation:
                ++motionDatagramsSent_;
                ++separateRotationDatagramsSent_;
                break;
            case PacketPurpose::Acceleration:
                ++motionDatagramsSent_;
                ++separateAccelerationDatagramsSent_;
                break;
            case PacketPurpose::MotionCombined:
                ++motionDatagramsSent_;
                break;
            case PacketPurpose::ControlBackground:
                ++backgroundControlDatagramsSent_;
                break;
            case PacketPurpose::Control:
            case PacketPurpose::UserAction:
            case PacketPurpose::ErrorReport:
                ++criticalControlDatagramsSent_;
                break;
            case PacketPurpose::Discovery:
            case PacketPurpose::Telemetry:
            case PacketPurpose::Tap:
                break;
        }
    }

    if (!packetPurposeIsMotion(purpose)) return;
    lastMotionTxAttemptMs_ = nowMs;
    lastMotionTxAttemptValid_ = true;
    lastMotionTxAttemptSucceeded_ = success;
    if (success) {
        lastSuccessfulMotionTxMs_ = nowMs;
        lastSuccessfulMotionTxValid_ = true;
        if (successfulMotionTxStreak_ != 0xFFFFFFFFUL) ++successfulMotionTxStreak_;
        if (awaitingPostRebindSuccess_ && lastUdpTransportRebindValid_ &&
            static_cast<int32_t>(nowMs - lastUdpTransportRebindMs_) >= 0) {
            awaitingPostRebindSuccess_ = false;
            txPressureState_ = SlimeVRTxPressureState::TransientPressure;
        }
        if (txPressureEpisodeActive_ && !txPressureStableActive_) {
            txPressureStableSinceMs_ = nowMs;
            txPressureStableActive_ = true;
        }
        return;
    }

    successfulMotionTxStreak_ = 0;
    txPressureStableSinceMs_ = 0;
    txPressureStableActive_ = false;
    if (pressureFailure) lastTxPressureFailureMs_ = nowMs;
}

bool SlimeVROutputRuntime::txFailureWindowExceeded() const {
    return txOutcomeWindowCount_ >= TRACKER_SLIMEVR_TX_FAILURE_WINDOW_ATTEMPTS &&
           txOutcomeFailureCount_ >= TRACKER_SLIMEVR_TX_FAILURE_WINDOW_FAILURES;
}

bool SlimeVROutputRuntime::packetMayBeDroppedDuringTxBackoff(PacketPurpose purpose) {
    switch (purpose) {
        case PacketPurpose::Rotation:
        case PacketPurpose::Acceleration:
        case PacketPurpose::MotionCombined:
        case PacketPurpose::Telemetry:
        case PacketPurpose::Tap:
            return true;
        case PacketPurpose::ControlBackground:
            return true;
        case PacketPurpose::Discovery:
        case PacketPurpose::Control:
        case PacketPurpose::UserAction:
        case PacketPurpose::ErrorReport:
            return false;
    }
    return false;
}

void SlimeVROutputRuntime::armTxBackoff(uint32_t nowMs) {
    uint32_t delayMs = TRACKER_SLIMEVR_TX_BACKOFF_INITIAL_MS;
    for (uint8_t i = 0; i < txBackoffLevel_; ++i) {
        if (delayMs >= TRACKER_SLIMEVR_TX_BACKOFF_MAX_MS / 2u) {
            delayMs = TRACKER_SLIMEVR_TX_BACKOFF_MAX_MS;
            break;
        }
        delayMs *= 2u;
    }
    if (delayMs > TRACKER_SLIMEVR_TX_BACKOFF_MAX_MS) {
        delayMs = TRACKER_SLIMEVR_TX_BACKOFF_MAX_MS;
    }
    txBackoffUntilMs_ = nowMs + delayMs;
    if (delayMs < TRACKER_SLIMEVR_TX_BACKOFF_MAX_MS && txBackoffLevel_ < 31u) {
        ++txBackoffLevel_;
    }
}

bool SlimeVROutputRuntime::txBackoffActive(uint32_t nowMs) const {
    return txBackoffUntilMs_ != 0u &&
           static_cast<int32_t>(nowMs - txBackoffUntilMs_) < 0;
}

void SlimeVROutputRuntime::resetTxAttemptState() {
    consecutiveSendFailures_ = 0;
    txBackoffUntilMs_ = 0;
    txBackoffLevel_ = 0;
    txOutcomeWindowBits_ = 0;
    txOutcomeWindowCount_ = 0;
    txOutcomeFailureCount_ = 0;
}

void SlimeVROutputRuntime::resetTxRecoveryState() {
    resetTxAttemptState();
    txPressureState_ = SlimeVRTxPressureState::Healthy;
    txRecoveryReason_ = SlimeVRTxRecoveryReason::None;
    txPressureEpisodeActive_ = false;
    txPressureEpisodeStartMs_ = 0;
    txPressureStableSinceMs_ = 0;
    txPressureStableActive_ = false;
    lastTxPressureFailureMs_ = 0;
    lastSuccessfulMotionTxMs_ = 0;
    lastSuccessfulMotionTxValid_ = false;
    lastMotionTxAttemptMs_ = 0;
    lastMotionTxAttemptValid_ = false;
    lastMotionTxAttemptSucceeded_ = false;
    successfulMotionTxStreak_ = 0;
    postRebindStartedMs_ = 0;
    awaitingPostRebindSuccess_ = false;
    txRecoveryRetryAfterMs_ = 0;
    txRecoveryRetryAfterValid_ = false;
}

void SlimeVROutputRuntime::beginTxPressureEpisode(uint32_t nowMs) {
    if (!txPressureEpisodeActive_) {
        txPressureEpisodeActive_ = true;
        txPressureEpisodeStartMs_ = nowMs;
        ++txPressureEpisodeCount_;
    }
    txPressureState_ = awaitingPostRebindSuccess_
        ? SlimeVRTxPressureState::AwaitingPostRebindSuccess
        : SlimeVRTxPressureState::TransientPressure;
    txPressureStableSinceMs_ = 0;
    txPressureStableActive_ = false;
    lastTxPressureFailureMs_ = nowMs;
}

void SlimeVROutputRuntime::closeTxPressureEpisode(uint32_t nowMs) {
    if (!txPressureEpisodeActive_) return;
    const uint32_t durationMs = static_cast<uint32_t>(nowMs - txPressureEpisodeStartMs_);
    if (durationMs > txPressureEpisodeMaxMs_) txPressureEpisodeMaxMs_ = durationMs;
    ++txPressureStableResets_;
    txPressureEpisodeActive_ = false;
    txPressureEpisodeStartMs_ = 0;
    txPressureStableSinceMs_ = 0;
    txPressureStableActive_ = false;
    lastTxPressureFailureMs_ = 0;
    successfulMotionTxStreak_ = 0;
    awaitingPostRebindSuccess_ = false;
    postRebindStartedMs_ = 0;
    txRecoveryRetryAfterMs_ = 0;
    txRecoveryRetryAfterValid_ = false;
    txPressureState_ = SlimeVRTxPressureState::Healthy;
    txRecoveryReason_ = SlimeVRTxRecoveryReason::None;
    resetTxAttemptState();
}

void SlimeVROutputRuntime::serviceTxRecovery(uint32_t nowMs) {
    if (!txPressureEpisodeActive_) {
        if (txPressureState_ == SlimeVRTxPressureState::FullReopenCooldown &&
            (!lastUdpFullReopenValid_ ||
             static_cast<uint32_t>(nowMs - lastUdpFullReopenMs_) >= TRACKER_SLIMEVR_TX_FULL_REOPEN_COOLDOWN_MS)) {
            txPressureState_ = SlimeVRTxPressureState::Healthy;
            txRecoveryReason_ = SlimeVRTxRecoveryReason::None;
        }
        return;
    }

    const uint32_t episodeDurationMs = static_cast<uint32_t>(nowMs - txPressureEpisodeStartMs_);
    if (episodeDurationMs > txPressureEpisodeMaxMs_) txPressureEpisodeMaxMs_ = episodeDurationMs;

    if (txRecoveryRetryAfterValid_) {
        if (static_cast<int32_t>(nowMs - txRecoveryRetryAfterMs_) < 0) return;
        txRecoveryRetryAfterValid_ = false;
    }

    if (txPressureStableActive_ &&
        static_cast<uint32_t>(nowMs - txPressureStableSinceMs_) >= TRACKER_SLIMEVR_TX_PRESSURE_STABLE_RESET_MS &&
        static_cast<uint32_t>(nowMs - lastTxPressureFailureMs_) >= TRACKER_SLIMEVR_TX_PRESSURE_STABLE_RESET_MS) {
        closeTxPressureEpisode(nowMs);
        return;
    }

    if (!serverFound_ || udpTransportRebindRequested_ || udpReopenRequested_) return;

    if (awaitingPostRebindSuccess_) {
        txPressureState_ = SlimeVRTxPressureState::AwaitingPostRebindSuccess;
        if (lastSuccessfulMotionTxValid_ && lastUdpTransportRebindValid_ &&
            static_cast<int32_t>(lastSuccessfulMotionTxMs_ - lastUdpTransportRebindMs_) >= 0) {
            awaitingPostRebindSuccess_ = false;
            txPressureState_ = SlimeVRTxPressureState::TransientPressure;
            return;
        }
        if (static_cast<uint32_t>(nowMs - postRebindStartedMs_) >=
                TRACKER_SLIMEVR_TX_POST_REBIND_REOPEN_STALL_MS) {
            requestUdpFullReopen(nowMs, SlimeVRTxRecoveryReason::PostRebindMotionStall);
        }
        return;
    }

    const bool activelyFailingMotion = lastMotionTxAttemptValid_ &&
        !lastMotionTxAttemptSucceeded_ &&
        static_cast<uint32_t>(nowMs - lastMotionTxAttemptMs_) <=
            TRACKER_SLIMEVR_TX_PRESSURE_ACTIVE_ATTEMPT_MS;
    if (!activelyFailingMotion) return;

    uint32_t stallOriginMs = txPressureEpisodeStartMs_;
    if (lastSuccessfulMotionTxValid_ &&
        static_cast<int32_t>(lastSuccessfulMotionTxMs_ - txPressureEpisodeStartMs_) >= 0) {
        stallOriginMs = lastSuccessfulMotionTxMs_;
    }
    const uint32_t motionStallMs = static_cast<uint32_t>(nowMs - stallOriginMs);
    if (motionStallMs < TRACKER_SLIMEVR_TX_PRESSURE_REBIND_STALL_MS) return;

    txPressureState_ = SlimeVRTxPressureState::SustainedPressure;
    const bool recentServerRx = lastIncomingPacketMs_ != 0u &&
        static_cast<uint32_t>(nowMs - lastIncomingPacketMs_) < TRACKER_SLIMEVR_TX_RECENT_RX_MS;
    if (recentServerRx) {
        requestUdpTransportRebind(nowMs, SlimeVRTxRecoveryReason::PressureMotionStall);
        return;
    }

    if (motionStallMs >= TRACKER_SLIMEVR_TX_POST_REBIND_REOPEN_STALL_MS) {
        requestUdpFullReopen(nowMs, SlimeVRTxRecoveryReason::StaleServerRx);
    }
}

void SlimeVROutputRuntime::requestUdpTransportRebind(uint32_t nowMs,
                                                     SlimeVRTxRecoveryReason reason) {
    if (udpReopenRequested_ || udpTransportRebindRequested_) return;
    if (lastUdpTransportRebindValid_ &&
        static_cast<uint32_t>(nowMs - lastUdpTransportRebindMs_) < TRACKER_SLIMEVR_TX_REBIND_COOLDOWN_MS) {
        ++udpRebindSuppressedCooldown_;
        txPressureState_ = SlimeVRTxPressureState::SustainedPressure;
        txRecoveryRetryAfterMs_ = lastUdpTransportRebindMs_ + TRACKER_SLIMEVR_TX_REBIND_COOLDOWN_MS;
        txRecoveryRetryAfterValid_ = true;
        return;
    }
    txRecoveryReason_ = reason;
    udpTransportRebindRequested_ = true;
}

void SlimeVROutputRuntime::requestUdpFullReopen(uint32_t nowMs,
                                                SlimeVRTxRecoveryReason reason) {
    if (udpReopenRequested_) return;
    if (lastUdpFullReopenValid_ &&
        static_cast<uint32_t>(nowMs - lastUdpFullReopenMs_) < TRACKER_SLIMEVR_TX_FULL_REOPEN_COOLDOWN_MS) {
        ++udpFullReopenSuppressedCooldown_;
        txPressureState_ = SlimeVRTxPressureState::FullReopenCooldown;
        txRecoveryRetryAfterMs_ = lastUdpFullReopenMs_ + TRACKER_SLIMEVR_TX_FULL_REOPEN_COOLDOWN_MS;
        txRecoveryRetryAfterValid_ = true;
        return;
    }
    ++udpFullReopenEscalations_;
    txRecoveryReason_ = reason;
    udpTransportRebindRequested_ = false;
    udpReopenRequested_ = true;
}

bool SlimeVROutputRuntime::rebindUdpPreservingSession(uint32_t nowMs) {
    if (!udp_) return false;
    ++udpTransportRebindRequests_;
    udp_->stop();
    if (!udp_->begin(localPort_)) {
        ++udpTransportRebindFailures_;
        ++udpBeginFailures_;
        return false;
    }

    ++udpTransportRebindSuccesses_;
    lastUdpTransportRebindMs_ = nowMs;
    lastUdpTransportRebindValid_ = true;
    postRebindStartedMs_ = nowMs;
    awaitingPostRebindSuccess_ = true;
    txPressureState_ = SlimeVRTxPressureState::AwaitingPostRebindSuccess;
    resetTxAttemptState();
    serverFoundSendGraceUntilMs_ = nowMs + TRACKER_SLIMEVR_TX_REBIND_SEND_GRACE_MS;
    // Preserve the same grace for stale motion and optional background
    // control traffic. Discovery and request/response control remain
    // exempt so a genuinely lost session can still recover.
    txBackoffUntilMs_ = serverFoundSendGraceUntilMs_;
    lastServiceUpdateMs_ = 0;
    return true;
}

bool SlimeVROutputRuntime::isUdpTxPressureError(int errorCode) {
    return errorCode == ENOMEM || errorCode == ENOBUFS ||
           errorCode == EAGAIN
#if defined(EWOULDBLOCK) && EWOULDBLOCK != EAGAIN
           || errorCode == EWOULDBLOCK
#endif
           ;
}

} // namespace tracker

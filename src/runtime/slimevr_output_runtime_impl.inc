#include "runtime/slimevr_output_runtime.hpp"

#include <cstring>
#include <cmath>

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
    if (!data || len == 0) return false;

    // Normal SlimeVR UDP packets use the same 12-byte header as tracker ->
    // server packets: type:u32be + packetNumber:u64be + payload. The raw
    // discovery response is the exception and is handled before this helper.
    if (len >= SLIMEVR_PACKET_HEADER_SIZE) {
        const uint32_t packetType = readU32Be(data);
        if (packetType <= 0xffu) {
            out.type = static_cast<uint8_t>(packetType);
            out.payload = data + SLIMEVR_PACKET_HEADER_SIZE;
            out.payloadLen = len - SLIMEVR_PACKET_HEADER_SIZE;
            out.headered = true;
            return true;
        }
    }

    // Keep a fallback for legacy/raw test packets and the old one-byte parser.
    out.type = data[0];
    out.payload = data + 1;
    out.payloadLen = len - 1;
    out.headered = false;
    return true;
}

} // namespace

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
                                    void* copyOutputSnapshotUser) {
    udp_ = &udp;
    wifi_ = &wifi;
    copyOutputSnapshot_ = copyOutputSnapshot;
    copyOutputSnapshotUser_ = copyOutputSnapshotUser;
    resetConnectionState(true);
}

void SlimeVROutputRuntime::configure(const SlimeVROutputRuntimeConfig& config) {
    const bool incomingRestComplete = config.hasCompletedRestCalibration;
    const bool restCalibrationChanged = hasCompletedRestCalibration_ != incomingRestComplete;
    const uint16_t incomingServerPort = config.serverPort == 0 ? SLIMEVR_DEFAULT_SERVER_PORT : config.serverPort;
    const uint16_t incomingLocalPort = config.localPort == 0 ? SLIMEVR_DISCOVERY_LOCAL_PORT : config.localPort;
    const bool incomingMagEnabled = config.magSupportEnabled && config.magEnabled;
    const bool configChanged = enabled_ != config.enabled ||
                               discoveryEnabled_ != config.discoveryEnabled ||
                               manualServerEnabled_ != config.manualServerEnabled ||
                               serverPort_ != incomingServerPort ||
                               localPort_ != incomingLocalPort ||
                               sensorId_ != config.sensorId ||
                               magSupportEnabled_ != config.magSupportEnabled ||
                               magEnabled_ != incomingMagEnabled ||
                               batteryTelemetryEnabled_ != config.batteryTelemetryEnabled ||
                               std::strncmp(deviceName_, config.deviceName ? config.deviceName : "", sizeof(deviceName_)) != 0;

    const bool rotationRateChanged = rotationRateHz_ != config.rotationRateHz;

    enabled_ = config.enabled;
    discoveryEnabled_ = config.discoveryEnabled;
    manualServerEnabled_ = config.manualServerEnabled;
    sensorId_ = config.sensorId;
    serverPort_ = config.serverPort == 0 ? SLIMEVR_DEFAULT_SERVER_PORT : config.serverPort;
    localPort_ = config.localPort == 0 ? SLIMEVR_DISCOVERY_LOCAL_PORT : config.localPort;
    discoveryIntervalMs_ = config.discoveryIntervalMs == 0 ? 1u : config.discoveryIntervalMs;
    rotationRateHz_ = config.rotationRateHz == 0 ? ROTATION_RATE_HZ_DEFAULT : config.rotationRateHz;
    if (rotationRateHz_ > ROTATION_RATE_HZ_MAX) rotationRateHz_ = ROTATION_RATE_HZ_MAX;
    incomingPacketsPerUpdate_ = config.incomingPacketsPerUpdate == 0 ? 1u : config.incomingPacketsPerUpdate;
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
        if (rotationRateChanged) {
            lastRotationAttemptMs_ = 0;
        }
        if (restCalibrationChanged) {
            requestSensorInfoRefresh();
        }
    }
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

void SlimeVROutputRuntime::requestSensorInfoRefresh() {
    lastSensorInfoMs_ = 0;
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
        lastRotationAttemptMs_ = 0;
    }
}

void SlimeVROutputRuntime::resetCounters() {
    handshakesSent_ = 0;
    heartbeatSent_ = 0;
    sensorInfoSent_ = 0;
    rotationSent_ = 0;
    rotationSendDue_ = 0;
    rotationRateLimited_ = 0;
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
    discoveryResponses_ = 0;
    heartbeatReceived_ = 0;
    pingReceived_ = 0;
    pongSent_ = 0;
    featureFlagsReceived_ = 0;
    setConfigFlagReceived_ = 0;
    setConfigFlagApplied_ = 0;
    setConfigFlagIgnored_ = 0;
    ackConfigSent_ = 0;
    protocolChangeReceived_ = 0;
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
    consecutiveSendFailures_ = 0;
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

bool SlimeVROutputRuntime::update(uint32_t nowMs) {
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
        consecutiveSendFailures_ = 0;
        udpReopenRequested_ = false;
        transitionTo(SlimeVROutputState::WaitingForWifi, nowMs);
        return activityChanged(stateBefore, activityBefore);
    }

    ensureUdp(nowMs);
    if (!udp_->active()) return activityChanged(stateBefore, activityBefore);

    if (udpReopenRequested_) {
        ++udpReopenRequests_;
        udp_->stop();
        serverFound_ = false;
        serverEndpoint_ = UdpEndpoint{};
        consecutiveSendFailures_ = 0;
        udpReopenRequested_ = false;
        lastSensorInfoMs_ = 0;
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
            lastSensorInfoMs_ = 0;
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
            if (nowMs - lastSensorInfoMs_ >= SENSOR_INFO_INTERVAL_MS) sendSensorInfo(nowMs);
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

SlimeVROutputRuntimeStatus SlimeVROutputRuntime::status() const {
    SlimeVROutputRuntimeStatus s;
    s.state = state_;
    s.enabled = enabled_;
    s.wifiConnected = wifi_ && wifi_->connected();
    s.udpReady = udp_ && udp_->active();
    s.serverFound = serverFound_;
    s.discoveryEnabled = discoveryEnabled_;
    s.manualServerEnabled = manualServerEnabled_;
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
    s.rotationSent = rotationSent_;
    s.rotationSendDue = rotationSendDue_;
    s.rotationRateLimited = rotationRateLimited_;
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
    s.discoveryResponses = discoveryResponses_;
    s.heartbeatReceived = heartbeatReceived_;
    s.pingReceived = pingReceived_;
    s.pongSent = pongSent_;
    s.featureFlagsReceived = featureFlagsReceived_;
    s.setConfigFlagReceived = setConfigFlagReceived_;
    s.setConfigFlagApplied = setConfigFlagApplied_;
    s.setConfigFlagIgnored = setConfigFlagIgnored_;
    s.ackConfigSent = ackConfigSent_;
    s.protocolChangeReceived = protocolChangeReceived_;
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
    s.consecutiveSendFailures = consecutiveSendFailures_;
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
    s.trackerErrorActive = trackerHealth_.fatalActive;
    s.trackerDegradedNoImu = trackerHealth_.degradedNoImu;
    s.trackerErrorCode = static_cast<uint8_t>(trackerHealth_.faultCode);
    s.trackerHealthRevision = trackerHealth_.revision;
    copyCString(s.trackerErrorMessage, sizeof(s.trackerErrorMessage), trackerHealth_.message);
    return s;
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
    lastHandshakeMs_ = 0;
    lastIncomingPacketMs_ = 0;
    lastHeartbeatMs_ = 0;
    lastSensorInfoMs_ = 0;
    lastRotationAttemptMs_ = 0;
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
    lastPingId_ = 0;
    lastServerFeatureFlags_ = 0;
    lastSetConfigSensorId_ = 0;
    lastSetConfigType_ = 0;
    lastSetConfigState_ = false;
    lastSetConfigApplied_ = false;
    lastProtocolTarget_ = 0;
    lastProtocolVersion_ = 0;
    lastUnknownPacketType_ = 0;
    nextUdpBeginRetryMs_ = 0;
    serverFoundSendGraceUntilMs_ = 0;
    consecutiveSendFailures_ = 0;
    udpReopenRequested_ = false;
    writer_.resetPacketNumber(0);
    if (!keepCounters) resetCounters();
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
        if (len <= 0) continue;

        const UdpEndpoint remote = udp_->remoteEndpoint();
        ++packetsReceived_;
        lastIncomingPacketMs_ = nowMs;
        handleIncomingPacket(incomingBuffer_, static_cast<size_t>(len), remote, nowMs);
    }
}

void SlimeVROutputRuntime::handleIncomingPacket(const uint8_t* data,
                                                size_t len,
                                                const UdpEndpoint& remote,
                                                uint32_t nowMs) {
    if (SlimeVRPacketWriter::isServerHandshakeResponse(data, len)) {
        serverFound_ = true;
        serverEndpoint_ = remote;
        if (serverEndpoint_.port == 0) serverEndpoint_.port = serverPort_;
        ++discoveryResponses_;
        transitionTo(SlimeVROutputState::ServerFound, nowMs);
        // ESP32 WiFiUDP can transiently fail with ENOMEM if we immediately
        // burst SensorInfo/telemetry/rotation in the same update that consumed
        // the discovery response. Give the Wi-Fi stack a short grace period,
        // then resume normal scheduled sends.
        serverFoundSendGraceUntilMs_ = nowMs + SERVER_FOUND_SEND_GRACE_MS;
        lastHeartbeatMs_ = nowMs;
        lastTelemetryMs_ = nowMs;
        lastSignalTelemetryMs_ = nowMs;
        lastTemperatureTelemetryMs_ = nowMs;
        lastBatteryTelemetryMs_ = nowMs;
        lastRotationAttemptMs_ = nowMs;
        lastSensorInfoMs_ = 0;
        lastTrackerErrorMs_ = 0;
        return;
    }

    if (!data || len == 0) return;

    SlimeVRIncomingPacketView packet;
    if (!parseIncomingPacketView(data, len, packet)) return;

    switch (static_cast<SlimeVRReceivePacketType>(packet.type)) {
        case SlimeVRReceivePacketType::HeartBeat0:
        case SlimeVRReceivePacketType::HeartBeat:
            if (serverFound_) {
                ++heartbeatReceived_;
                sendHeartbeat(nowMs);
            }
            return;
        case SlimeVRReceivePacketType::PingPong:
            if (serverFound_) handlePingPong(packet.payload, packet.payloadLen);
            return;
        case SlimeVRReceivePacketType::FeatureFlags:
            handleFeatureFlags(packet.payload, packet.payloadLen);
            return;
        case SlimeVRReceivePacketType::SetConfigFlag:
            handleSetConfigFlag(packet.payload, packet.payloadLen, nowMs);
            return;
        case SlimeVRReceivePacketType::ProtocolChange:
            handleProtocolChange(packet.payload, packet.payloadLen);
            return;
        case SlimeVRReceivePacketType::Handshake:
            // Non-discovery handshake-like packets are ignored; the real
            // discovery response was handled above by isServerHandshakeResponse.
            return;
    }

    ++unknownPacketsReceived_;
    lastUnknownPacketType_ = packet.type;
}

void SlimeVROutputRuntime::handlePingPong(const uint8_t* data, size_t len) {
    if (!serverEndpoint_.valid() || len < 4) return;
    const uint32_t pingId = readU32Be(data);
    const SlimeVRPacketWriteResult packet = writer_.writePingPong(packetBuffer_, sizeof(packetBuffer_), pingId);
    ++pingReceived_;
    lastPingId_ = pingId;
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Control)) {
        ++pongSent_;
    }
}

void SlimeVROutputRuntime::handleFeatureFlags(const uint8_t* data, size_t len) {
    ++featureFlagsReceived_;
    uint32_t packed = 0;
    const size_t n = len < 4 ? len : 4;
    for (size_t i = 0; i < n; ++i) {
        packed = (packed << 8) | static_cast<uint32_t>(data[i]);
    }
    lastServerFeatureFlags_ = packed;
}

void SlimeVROutputRuntime::handleSetConfigFlag(const uint8_t* data, size_t len, uint32_t nowMs) {
    if (len < 4) return;

    const uint8_t targetSensorId = data[0];
    const uint16_t configType = readU16Be(data + 1);
    const bool requestedState = data[3] != 0;

    ++setConfigFlagReceived_;
    lastSetConfigSensorId_ = targetSensorId;
    lastSetConfigType_ = configType;
    lastSetConfigState_ = requestedState;
    lastSetConfigApplied_ = false;

    const bool targetsThisSensor = targetSensorId == sensorId_ || targetSensorId == SLIMEVR_SENSOR_ID_GLOBAL;
    if (!targetsThisSensor) {
        ++setConfigFlagIgnored_;
        return;
    }

    bool applied = false;
    if (setConfigFlag_) {
        applied = setConfigFlag_(sensorId_, configType, requestedState, setConfigFlagUser_);
    } else if (configType == SLIMEVR_CONFIG_TYPE_MAGNETOMETER) {
        applied = true;
    }

    if (applied && configType == SLIMEVR_CONFIG_TYPE_MAGNETOMETER) {
        magSupportEnabled_ = true;
        magEnabled_ = requestedState;
        // Push the updated supported/enabled state promptly instead of waiting
        // for the next periodic SensorInfo refresh.
        sendSensorInfo(nowMs);
    }

    if (applied) {
        ++setConfigFlagApplied_;
        lastSetConfigApplied_ = true;
        sendAckConfigChange(configType);
    } else {
        ++setConfigFlagIgnored_;
    }
}

void SlimeVROutputRuntime::handleProtocolChange(const uint8_t* data, size_t len) {
    ++protocolChangeReceived_;
    if (len >= 2) {
        lastProtocolTarget_ = data[0];
        lastProtocolVersion_ = data[1];
    }
}

void SlimeVROutputRuntime::sendAckConfigChange(uint16_t configType) {
    if (!serverEndpoint_.valid()) return;
    const SlimeVRPacketWriteResult packet = writer_.writeAcknowledgeConfigChange(
        packetBuffer_,
        sizeof(packetBuffer_),
        sensorId_,
        configType
    );
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Control)) {
        ++ackConfigSent_;
    }
}

void SlimeVROutputRuntime::maybeSendDiscovery(uint32_t nowMs) {
    if (!discoveryEnabled_) return;
    if (lastHandshakeMs_ != 0 && nowMs - lastHandshakeMs_ < discoveryIntervalMs_) return;

    UdpEndpoint broadcast;
    broadcast.ipv4 = SLIMEVR_DISCOVERY_BROADCAST_IPV4;
    broadcast.port = serverPort_;
    sendHandshakeTo(broadcast, nowMs);
}

void SlimeVROutputRuntime::sendHandshakeTo(const UdpEndpoint& endpoint, uint32_t nowMs) {
    SlimeVRHandshakeInfo info;
    makeHandshakeInfo(info);
    const SlimeVRPacketWriteResult packet = writer_.writeHandshake(packetBuffer_, sizeof(packetBuffer_), info);
    if (sendPacket(packet, endpoint, PacketPurpose::Discovery)) {
        ++handshakesSent_;
        lastHandshakeMs_ = nowMs;
    }
}

void SlimeVROutputRuntime::sendSensorInfo(uint32_t nowMs) {
    if (!serverEndpoint_.valid()) return;
    SlimeVRSensorInfo info;
    info.sensorId = sensorId_;
    // Keep SensorInfo online even in firmware-fault/degraded mode. SlimeVR
    // Server treats Offline as a tracker disconnect state; repeatedly sending
    // Offline while the UDP session is healthy makes the GUI flap between OK
    // and disconnected. Firmware faults are reported with the Error packet
    // instead, and RotationData stays suppressed until the IMU path recovers.
    info.sensorState = SlimeVRSensorState::Online;
    info.sensorConfig = sensorConfigFlags();
    info.hasCompletedRestCalibration = !trackerHealth_.degradedNoImu && hasCompletedRestCalibration_;
    const SlimeVRPacketWriteResult packet = writer_.writeSensorInfo(packetBuffer_, sizeof(packetBuffer_), info);
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Control)) {
        ++sensorInfoSent_;
        lastSensorInfoMs_ = nowMs;
    }
}

void SlimeVROutputRuntime::sendHeartbeat(uint32_t nowMs) {
    if (!serverEndpoint_.valid()) return;
    const SlimeVRPacketWriteResult packet = writer_.writeHeartbeat(packetBuffer_, sizeof(packetBuffer_));
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Control)) {
        ++heartbeatSent_;
        lastHeartbeatMs_ = nowMs;
    }
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
           rotationSendDue_ + signalStrengthSent_ + temperatureSent_ + batterySent_ +
           batterySendFailures_ + magnetometerAccuracySent_ + tapSent_ + tapSendFailures_ +
           trackerErrorSent_ + trackerErrorSendFailures_ +
           rotationNoSnapshot_ + rotationDuplicateSnapshot_ + packetsReceived_ +
           discoveryResponses_ + heartbeatReceived_ + pingReceived_ + pongSent_ +
           featureFlagsReceived_ + setConfigFlagReceived_ + setConfigFlagApplied_ +
           setConfigFlagIgnored_ + ackConfigSent_ + protocolChangeReceived_ +
           unknownPacketsReceived_ + sendFailures_ + rotationSendFailures_ +
           controlSendFailures_ + telemetrySendFailures_ + discoverySendFailures_ +
           tapTransportSendFailures_ + udpBeginFailures_ + serverSilenceResets_ +
           wifiLostResets_ + udpReopenRequests_ + consecutiveSendFailures_ +
           serviceUpdates_;
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
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Tap)) {
        ++tapSent_;
        lastTapValue_ = value;
        return true;
    }
    ++tapSendFailures_;
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
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Telemetry)) {
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
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Telemetry)) {
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
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Telemetry)) {
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
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Telemetry)) {
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
    if (sendPacket(packet, serverEndpoint_, PacketPurpose::ErrorReport)) {
        ++trackerErrorSent_;
        lastTrackerErrorMs_ = nowMs;
        return;
    }
    ++trackerErrorSendFailures_;
}

void SlimeVROutputRuntime::maybeRecordRotationSuppressedByError(uint32_t nowMs) {
    if (lastRotationAttemptMs_ != 0 && nowMs - lastRotationAttemptMs_ < rotationPeriodMs()) {
        return;
    }
    lastRotationAttemptMs_ = nowMs;
    ++rotationSuppressedByError_;
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
    if (lastRotationAttemptMs_ != 0 && nowMs - lastRotationAttemptMs_ < rotationPeriodMs()) {
        ++rotationRateLimited_;
        return;
    }
    lastRotationAttemptMs_ = nowMs;
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
    const uint8_t accuracy = accuracyFromConfidence(snapshot.confidence);
    const SlimeVRPacketWriteResult packet = writer_.writeRotationData(
        packetBuffer_,
        sizeof(packetBuffer_),
        sensorId_,
        snapshot.q,
        accuracy,
        SlimeVRRotationDataType::Normal
    );

    if (sendPacket(packet, serverEndpoint_, PacketPurpose::Rotation)) {
        ++rotationSent_;
        lastRotationMs_ = nowMs;
        lastRotationSnapshotSequence_ = snapshot.sequence;
        lastRotationRuntimeSample_ = snapshot.runtimeSample;
        lastRotationTimestampUs_ = snapshot.timestampUs;
        lastRotationQualityFlags_ = snapshot.qualityFlags;
        lastRotationConfidence_ = snapshot.confidence;
        const uint64_t nowUs = static_cast<uint64_t>(nowMs) * 1000ULL;
        const uint64_t ageUs = nowUs >= snapshot.timestampUs ? (nowUs - snapshot.timestampUs) : 0ULL;
        lastRotationSnapshotAgeUs_ = ageUs > 0xffffffffULL ? 0xffffffffUL : static_cast<uint32_t>(ageUs);
    }
}

void SlimeVROutputRuntime::makeHandshakeInfo(SlimeVRHandshakeInfo& info) const {
    info.firmwareVersion = "c3-6dsv-fw";
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
            ++controlSendFailures_;
            break;
        case PacketPurpose::Telemetry:
            ++telemetrySendFailures_;
            break;
        case PacketPurpose::Rotation:
            ++rotationSendFailures_;
            break;
        case PacketPurpose::Tap:
            ++tapTransportSendFailures_;
            break;
        case PacketPurpose::ErrorReport:
            ++controlSendFailures_;
            break;
    }
}

bool SlimeVROutputRuntime::sendPacket(const SlimeVRPacketWriteResult& packet,
                                      const UdpEndpoint& endpoint,
                                      PacketPurpose purpose) {
    if (!udp_ || !packet.ok || packet.size == 0) {
        recordSendFailure(purpose);
        ++consecutiveSendFailures_;
        return false;
    }
    if (!udp_->send(endpoint, packetBuffer_, packet.size)) {
        recordSendFailure(purpose);
        ++consecutiveSendFailures_;
        if (consecutiveSendFailures_ >= TRACKER_SLIMEVR_SEND_FAILURE_REOPEN_THRESHOLD) {
            udpReopenRequested_ = true;
        }
        return false;
    }
    consecutiveSendFailures_ = 0;
    return true;
}

} // namespace tracker

#include "runtime/slimevr_output_runtime.hpp"

#include <cstring>
#include <cmath>

namespace tracker {

namespace {

constexpr uint32_t UDP_BEGIN_RETRY_MS = 1000;
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5000;
constexpr uint32_t SENSOR_INFO_INTERVAL_MS = 1000;
constexpr uint32_t SERVER_SILENCE_TIMEOUT_MS = 15000;
constexpr uint16_t ROTATION_RATE_HZ_DEFAULT = 100;
constexpr uint16_t ROTATION_RATE_HZ_MAX = 1000;

void copyCString(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    if (!src) src = "";
    std::strncpy(dst, src, dstSize - 1);
    dst[dstSize - 1] = '\0';
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
    const bool configChanged = enabled_ != config.enabled ||
                               discoveryEnabled_ != config.discoveryEnabled ||
                               manualServerEnabled_ != config.manualServerEnabled ||
                               serverPort_ != config.serverPort ||
                               localPort_ != config.localPort ||
                               sensorId_ != config.sensorId ||
                               magSupportEnabled_ != config.magSupportEnabled ||
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
    signalTelemetryEnabled_ = config.signalTelemetryEnabled;
    temperatureTelemetryEnabled_ = config.temperatureTelemetryEnabled;
    telemetryIntervalMs_ = config.telemetryIntervalMs == 0 ? TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS : config.telemetryIntervalMs;
    latestTemperatureValid_ = config.latestTemperatureValid && std::isfinite(config.latestTemperatureC);
    latestTemperatureC_ = latestTemperatureValid_ ? config.latestTemperatureC : 0.0f;
    copyCString(deviceName_, sizeof(deviceName_), config.deviceName && config.deviceName[0] ? config.deviceName : "c3-6dsv-tracker");

    if (!enabled_) {
        stop();
        return;
    }

    if (configChanged) {
        resetConnectionState(true);
    } else if (rotationRateChanged) {
        lastRotationAttemptMs_ = 0;
    }
}

void SlimeVROutputRuntime::resetCounters() {
    handshakesSent_ = 0;
    heartbeatSent_ = 0;
    sensorInfoSent_ = 0;
    rotationSent_ = 0;
    signalStrengthSent_ = 0;
    temperatureSent_ = 0;
    magnetometerAccuracySent_ = 0;
    rotationNoSnapshot_ = 0;
    rotationDuplicateSnapshot_ = 0;
    packetsReceived_ = 0;
    discoveryResponses_ = 0;
    sendFailures_ = 0;
    udpBeginFailures_ = 0;
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

void SlimeVROutputRuntime::update(uint32_t nowMs) {
    if (!udp_ || !wifi_) return;

    if (!enabled_) {
        if (udp_->active()) udp_->stop();
        transitionTo(SlimeVROutputState::Disabled, nowMs);
        return;
    }

    const bool wifiConnected = wifi_->connected();
    if (!wifiConnected) {
        if (udp_->active()) udp_->stop();
        serverFound_ = false;
        serverEndpoint_ = UdpEndpoint{};
        transitionTo(SlimeVROutputState::WaitingForWifi, nowMs);
        return;
    }

    ensureUdp(nowMs);
    if (!udp_->active()) return;

    pollIncoming(nowMs);

    if (serverFound_) {
        transitionTo(SlimeVROutputState::ServerFound, nowMs);
        if (lastIncomingPacketMs_ != 0 && nowMs - lastIncomingPacketMs_ >= SERVER_SILENCE_TIMEOUT_MS) {
            // UDP sends can continue to succeed while the server process was
            // restarted or the old association disappeared. Drop back to
            // discovery when the server has been silent long enough.
            serverFound_ = false;
            serverEndpoint_ = UdpEndpoint{};
            lastSensorInfoMs_ = 0;
            lastHeartbeatMs_ = 0;
            lastTelemetryMs_ = 0;
            transitionTo(SlimeVROutputState::Discovering, nowMs);
            maybeSendDiscovery(nowMs);
            return;
        }
        if (nowMs - lastHeartbeatMs_ >= HEARTBEAT_INTERVAL_MS) sendHeartbeat(nowMs);
        if (nowMs - lastSensorInfoMs_ >= SENSOR_INFO_INTERVAL_MS) sendSensorInfo(nowMs);
        maybeSendTelemetry(nowMs);
        maybeSendRotation(nowMs);
        return;
    }

    transitionTo(SlimeVROutputState::Discovering, nowMs);
    maybeSendDiscovery(nowMs);
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
    s.signalStrengthSent = signalStrengthSent_;
    s.temperatureSent = temperatureSent_;
    s.magnetometerAccuracySent = magnetometerAccuracySent_;
    s.rotationNoSnapshot = rotationNoSnapshot_;
    s.rotationDuplicateSnapshot = rotationDuplicateSnapshot_;
    s.packetsReceived = packetsReceived_;
    s.discoveryResponses = discoveryResponses_;
    s.sendFailures = sendFailures_;
    s.udpBeginFailures = udpBeginFailures_;
    s.nextPacketNumber = static_cast<uint32_t>(writer_.packetNumber());
    s.rotationRateHz = rotationRateHz_;
    s.preparedOutputAvailable = copyOutputSnapshot_ != nullptr;
    s.magSupportEnabled = magSupportEnabled_;
    s.sensorConfig = sensorConfigFlags();
    s.signalTelemetryEnabled = signalTelemetryEnabled_;
    s.temperatureTelemetryEnabled = temperatureTelemetryEnabled_;
    s.telemetryIntervalMs = telemetryIntervalMs_;
    s.lastSignalStrength = lastSignalStrength_;
    s.lastRssiDbm = lastRssiDbm_;
    s.lastTemperatureC = latestTemperatureC_;
    s.lastTemperatureValid = latestTemperatureValid_;
    s.lastHandshakeMs = lastHandshakeMs_;
    s.lastIncomingPacketMs = lastIncomingPacketMs_;
    s.lastStateChangeMs = lastStateChangeMs_;
    s.lastRotationMs = lastRotationMs_;
    s.lastRotationSnapshotSequence = lastRotationSnapshotSequence_;
    s.lastRotationRuntimeSample = lastRotationRuntimeSample_;
    s.lastRotationTimestampUs = lastRotationTimestampUs_;
    s.lastRotationQualityFlags = lastRotationQualityFlags_;
    s.lastRotationConfidence = lastRotationConfidence_;
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
    lastSignalStrength_ = 0;
    lastRssiDbm_ = 0;
    lastRotationSnapshotSequence_ = 0;
    lastRotationRuntimeSample_ = 0;
    lastRotationTimestampUs_ = 0;
    lastRotationQualityFlags_ = 0;
    lastRotationConfidence_ = 0.0f;
    nextUdpBeginRetryMs_ = 0;
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
        sendSensorInfo(nowMs);
        return;
    }

    if (serverFound_ && SlimeVRPacketWriter::isPacketType(data, len, SlimeVRReceivePacketType::HeartBeat)) {
        sendHeartbeat(nowMs);
        return;
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
    if (sendPacket(packet, endpoint)) {
        ++handshakesSent_;
        lastHandshakeMs_ = nowMs;
    }
}

void SlimeVROutputRuntime::sendSensorInfo(uint32_t nowMs) {
    if (!serverEndpoint_.valid()) return;
    SlimeVRSensorInfo info;
    info.sensorId = sensorId_;
    info.sensorConfig = sensorConfigFlags();
    const SlimeVRPacketWriteResult packet = writer_.writeSensorInfo(packetBuffer_, sizeof(packetBuffer_), info);
    if (sendPacket(packet, serverEndpoint_)) {
        ++sensorInfoSent_;
        lastSensorInfoMs_ = nowMs;
    }
}

void SlimeVROutputRuntime::sendHeartbeat(uint32_t nowMs) {
    if (!serverEndpoint_.valid()) return;
    const SlimeVRPacketWriteResult packet = writer_.writeHeartbeat(packetBuffer_, sizeof(packetBuffer_));
    if (sendPacket(packet, serverEndpoint_)) {
        ++heartbeatSent_;
        lastHeartbeatMs_ = nowMs;
    }
}

void SlimeVROutputRuntime::maybeSendTelemetry(uint32_t nowMs) {
    if (!serverEndpoint_.valid()) return;
    if (telemetryIntervalMs_ == 0) return;
    if (lastTelemetryMs_ != 0 && nowMs - lastTelemetryMs_ < telemetryIntervalMs_) return;
    lastTelemetryMs_ = nowMs;

    if (signalTelemetryEnabled_) sendSignalStrength(nowMs);
    if (temperatureTelemetryEnabled_) sendTemperature(nowMs);
    if (magSupportEnabled_) sendMagnetometerAccuracy(nowMs);
}

void SlimeVROutputRuntime::sendSignalStrength(uint32_t nowMs) {
    (void)nowMs;
    if (!serverEndpoint_.valid() || !wifi_) return;
    const TrackerWifiManagerStatus ws = wifi_->status();
    const uint8_t signal = signalStrengthFromRssi(ws.rssiDbm);
    const SlimeVRPacketWriteResult packet = writer_.writeSignalStrength(
        packetBuffer_,
        sizeof(packetBuffer_),
        sensorId_,
        signal
    );
    if (sendPacket(packet, serverEndpoint_)) {
        ++signalStrengthSent_;
        lastSignalStrength_ = signal;
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
    if (sendPacket(packet, serverEndpoint_)) {
        ++temperatureSent_;
    }
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
    if (sendPacket(packet, serverEndpoint_)) {
        ++magnetometerAccuracySent_;
    }
}

uint32_t SlimeVROutputRuntime::rotationPeriodMs() const {
    const uint16_t hz = rotationRateHz_ == 0 ? ROTATION_RATE_HZ_DEFAULT : rotationRateHz_;
    const uint32_t period = 1000UL / hz;
    return period == 0 ? 1UL : period;
}

uint16_t SlimeVROutputRuntime::sensorConfigFlags() const {
    uint16_t flags = 0;
    if (magSupportEnabled_) flags |= SLIMEVR_SENSOR_CONFIG_MAG_SUPPORTED;
    return flags;
}

uint8_t SlimeVROutputRuntime::accuracyFromConfidence(float confidence) {
    if (confidence >= 0.90f) return 3;
    if (confidence >= 0.65f) return 2;
    if (confidence >= 0.35f) return 1;
    return 0;
}

uint8_t SlimeVROutputRuntime::signalStrengthFromRssi(int32_t rssiDbm) {
    if (rssiDbm <= -100) return 0;
    if (rssiDbm >= -50) return 100;
    return static_cast<uint8_t>((rssiDbm + 100) * 2);
}

void SlimeVROutputRuntime::maybeSendRotation(uint32_t nowMs) {
    if (!serverEndpoint_.valid()) return;
    if (!copyOutputSnapshot_) return;
    if (lastRotationAttemptMs_ != 0 && nowMs - lastRotationAttemptMs_ < rotationPeriodMs()) return;
    lastRotationAttemptMs_ = nowMs;

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

    if (sendPacket(packet, serverEndpoint_)) {
        ++rotationSent_;
        lastRotationMs_ = nowMs;
        lastRotationSnapshotSequence_ = snapshot.sequence;
        lastRotationRuntimeSample_ = snapshot.runtimeSample;
        lastRotationTimestampUs_ = snapshot.timestampUs;
        lastRotationQualityFlags_ = snapshot.qualityFlags;
        lastRotationConfidence_ = snapshot.confidence;
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

bool SlimeVROutputRuntime::sendPacket(const SlimeVRPacketWriteResult& packet, const UdpEndpoint& endpoint) {
    if (!udp_ || !packet.ok || packet.size == 0) {
        ++sendFailures_;
        return false;
    }
    if (!udp_->send(endpoint, packetBuffer_, packet.size)) {
        ++sendFailures_;
        return false;
    }
    return true;
}

} // namespace tracker

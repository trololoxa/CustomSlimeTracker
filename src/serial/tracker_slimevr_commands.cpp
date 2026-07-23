#include "serial/tracker_slimevr_commands.hpp"

#include <Arduino.h>

#include "config/tracker_config_runtime.hpp"
#include "config/tracker_network_config.hpp"
#include "network/udp_transport.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "runtime/battery_runtime.hpp"
#include "runtime/tap_runtime_controller.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

namespace {

Stream& outFor(TrackerSerialCommandContext& ctx) {
    return ctx.io ? *ctx.io : Serial;
}

const char* yn(bool v) { return v ? "yes" : "no"; }

bool motionFrameConfigReady(const TrackerSerialCommandContext& ctx) {
    return ctx.config &&
           ctx.config->data.accelCal.valid &&
           ctx.config->data.frame.sensorToDeviceValid;
}

void printMotionFrameContract(const TrackerSerialCommandContext& ctx,
                              Stream& out,
                              const SlimeVROutputRuntimeStatus& s) {
    const bool protocolReady = slimevr_motion_frame::protocolUsesCorrectedAcceleration(
        s.protocolVersion
    );
    const bool configReady = motionFrameConfigReady(ctx);
    out.print("motion_frame_contract="); out.println(slimevr_motion_frame::CONTRACT_NAME);
    out.print("rotation_convention="); out.println(slimevr_motion_frame::ROTATION_CONVENTION_NAME);
    out.print("acceleration_frame="); out.println(slimevr_motion_frame::ACCELERATION_FRAME_NAME);
    out.print("acceleration_units="); out.println(slimevr_motion_frame::ACCELERATION_UNITS_NAME);
    out.print("legacy_acceleration_correction="); out.println(protocolReady ? "no" : "yes");
    out.print("motion_frame_config_ready="); out.println(yn(configReady));
    out.print("step_mounting_ready=");
    out.println(yn(protocolReady && configReady && s.preparedOutputAvailable &&
                   s.serverFound && s.accelerationSent > 0u && !s.trackerErrorActive));
}

void serviceNonCliRuntime(TrackerSerialCommandContext& ctx) {
    if (ctx.serviceNonCliRuntime) {
        (void)ctx.serviceNonCliRuntime(ctx.serviceNonCliRuntimeUser);
    }
}

bool serialSlimeSetConfigFlag(uint8_t sensorId, uint16_t configType, bool enabled, void* user) {
    (void)sensorId;
    auto* ctx = static_cast<TrackerSerialCommandContext*>(user);
    if (!ctx || configType != SLIMEVR_CONFIG_TYPE_MAGNETOMETER ||
        !ctx->setMagYawCorrectionApplyEnabled) return false;
    return ctx->setMagYawCorrectionApplyEnabled(
        enabled, true, ctx->setMagYawCorrectionApplyEnabledUser
    );
}

SlimeVROutputRuntimeConfig makeConfigFromNetwork(TrackerSerialCommandContext& ctx, const TrackerNetworkConfig& net, uint16_t rotationRateHz) {
    SlimeVROutputRuntimeConfig cfg;
    cfg.enabled = true;
    cfg.discoveryEnabled = net.data.discoveryEnabled;
    cfg.manualServerEnabled = net.data.manualServerEnabled;
    cfg.manualServerHost = net.data.serverHost;
    cfg.deviceName = net.data.deviceName;
    cfg.sensorId = net.data.sensorId;
    cfg.serverPort = net.data.serverPort;
    cfg.localPort = SLIMEVR_DISCOVERY_LOCAL_PORT;
    cfg.discoveryIntervalMs = TRACKER_SLIMEVR_DISCOVERY_INTERVAL_MS;
    cfg.rotationRateHz = rotationRateHz == 0 ? ::tracker::cfg::OUTPUT_RATE_HZ : rotationRateHz;
    if (cfg.rotationRateHz > TRACKER_SLIMEVR_OUTPUT_RATE_HZ_MAX) {
        cfg.rotationRateHz = TRACKER_SLIMEVR_OUTPUT_RATE_HZ_MAX;
    }
    cfg.incomingPacketsPerUpdate = TRACKER_SLIMEVR_INCOMING_PACKETS_PER_UPDATE;
    cfg.setConfigFlag = serialSlimeSetConfigFlag;
    cfg.setConfigFlagUser = &ctx;
    cfg.telemetryIntervalMs = TRACKER_SLIMEVR_TELEMETRY_INTERVAL_MS;
    cfg.signalTelemetryIntervalMs = TRACKER_SLIMEVR_SIGNAL_TELEMETRY_INTERVAL_MS;
    cfg.temperatureTelemetryIntervalMs = TRACKER_SLIMEVR_TEMPERATURE_TELEMETRY_INTERVAL_MS;
    cfg.batteryTelemetryIntervalMs = TRACKER_SLIMEVR_BATTERY_TELEMETRY_INTERVAL_MS;
    if (ctx.config) {
        cfg.magSupportEnabled = ctx.config->data.magCal.driverEnabled ||
                                ctx.config->data.magCal.calibrationValid ||
                                ctx.config->data.magCal.axisAlignmentValid ||
                                ctx.config->data.magYaw.applyEnabled;
        cfg.magEnabled = cfg.magSupportEnabled && ctx.config->data.magYaw.applyEnabled;
    }
    cfg.batteryTelemetryEnabled = (TRACKER_SLIMEVR_ENABLE_BATTERY_TELEMETRY != 0) &&
                                  (TRACKER_ENABLE_BATTERY_RUNTIME != 0);
    if (ctx.batteryRuntime) {
        float voltage = 0.0f;
        float percentage = 0.0f;
        cfg.latestBatteryValid = ctx.batteryRuntime->telemetry(voltage, percentage);
        cfg.latestBatteryVoltage = voltage;
        cfg.latestBatteryPercentage = percentage;
    }
    return cfg;
}

uint16_t slimeRotationRateHzFromConfig(const TrackerSerialCommandContext& ctx) {
    return ctx.config ? ctx.config->data.output.outputRateHz : 100;
}

void stopLocalSerialStreamForSlime(TrackerSerialCommandContext& ctx) {
    if (ctx.streamState) {
        ctx.streamState->mode = TrackerStreamMode::Off;
        ctx.streamState->lastEmitUs = 0;
    }
}


void printSlimeStatusBrief(TrackerSerialCommandContext& ctx,
                           Stream& out,
                           const SlimeVROutputRuntimeStatus& s) {
    char ipBuf[24];
    out.println("# SLIMEVR STATUS");
    out.print("enabled="); out.println(yn(s.enabled));
    out.print("state="); out.println(slimevrOutputStateName(s.state));
    out.print("wifi_connected="); out.println(yn(s.wifiConnected));
    out.print("udp_ready="); out.println(yn(s.udpReady));
    out.print("server_found="); out.println(yn(s.serverFound));
    out.print("manual_server_host="); out.println(s.manualServerHost);
    out.print("manual_server_resolved="); out.println(yn(s.manualServerResolved));
    out.print("server_ip="); out.println(s.serverIpv4 ? udpIpv4ToCString(s.serverIpv4, ipBuf, sizeof(ipBuf)) : "0.0.0.0");
    out.print("server_port="); out.println(s.serverPort);
    out.print("protocol_version="); out.println(s.protocolVersion);
    printMotionFrameContract(ctx, out, s);
    out.print("motion_packet_mode="); out.println(slimevrMotionPacketModeName(s.motionPacketMode));
    out.print("packet23_available="); out.println(s.compactMotionAvailable ? "yes" : "no");
    out.print("packet23_enabled="); out.println(s.compactMotionEnabled ? "yes" : "no");
    out.print("bundle_negotiation_enabled="); out.println(s.bundleNegotiationEnabled ? "yes" : "no");
    out.print("feature_negotiation_state="); out.println(slimevrFeatureNegotiationStateName(s.featureNegotiationState));
    out.print("feature_flags_request_attempts="); out.println(s.featureFlagsRequestAttempts);
    out.print("server_feature_flags_available="); out.println(s.serverFeatureFlagsAvailable ? "yes" : "no");
    out.print("server_bundle_supported="); out.println(s.serverBundleSupported ? "yes" : "no");
    out.print("server_compact_bundle_supported="); out.println(s.serverCompactBundleSupported ? "yes" : "no");
    out.print("fallback_acceleration_rate_hz="); out.println(s.fallbackAccelerationRateHz);
    out.print("bundled_motion_sent="); out.println(s.bundledMotionSent);
    out.print("bundled_motion_send_failures="); out.println(s.bundledMotionSendFailures);
    out.print("acceleration_rate_limited="); out.println(s.accelerationRateLimited);
    out.print("feature_flags_sent="); out.println(s.featureFlagsSent);
    out.print("feature_flags_send_failures="); out.println(s.featureFlagsSendFailures);
    out.print("rotation_sent="); out.println(s.rotationSent);
    out.print("acceleration_sent="); out.println(s.accelerationSent);
    out.print("compact_motion_sent="); out.println(s.compactMotionSent);
    out.print("compact_motion_send_failures="); out.println(s.compactMotionSendFailures);
    out.print("acceleration_skipped_invalid="); out.println(s.accelerationSkippedInvalid);
    out.print("acceleration_skipped_configuration="); out.println(s.accelerationSkippedConfiguration);
    out.print("acceleration_skipped_component_missing="); out.println(s.accelerationSkippedComponentMissing);
    out.print("acceleration_skipped_pair_degraded="); out.println(s.accelerationSkippedPairDegraded);
    out.print("acceleration_skipped_saturated="); out.println(s.accelerationSkippedSaturated);
    out.print("acceleration_skipped_nonfinite="); out.println(s.accelerationSkippedNonFinite);
    out.print("acceleration_skipped_other="); out.println(s.accelerationSkippedOther);
    out.print("acceleration_send_failures="); out.println(s.accelerationSendFailures);
    out.print("rotation_suppressed_by_error="); out.println(s.rotationSuppressedByError);
    out.print("rotation_send_due="); out.println(s.rotationSendDue);
    out.print("rotation_rate_limited="); out.println(s.rotationRateLimited);
    out.print("rotation_missed_deadlines="); out.println(s.rotationMissedDeadlines);
    out.print("rotation_late_events="); out.println(s.rotationLateEvents);
    out.print("rotation_lateness_sum_ms="); out.println(s.rotationLatenessSumMs);
    out.print("rotation_lateness_max_ms="); out.println(s.rotationLatenessMaxMs);
    out.print("service_updates="); out.println(s.serviceUpdates);
    out.print("service_skips="); out.println(s.serviceSkips);
    out.print("rotation_rate_hz="); out.println(s.rotationRateHz);
    serviceNonCliRuntime(ctx);
    out.print("tap_sent="); out.println(s.tapSent);
    out.print("tap_send_failures="); out.println(s.tapSendFailures);
    out.print("last_tap_value="); out.println(s.lastTapValue);
    out.print("user_action_sent="); out.println(s.userActionSent);
    out.print("user_action_send_failures="); out.println(s.userActionSendFailures);
    out.print("last_user_action="); out.println(slimevrUserActionName(s.lastUserAction));
    out.print("tracker_error_active="); out.println(yn(s.trackerErrorActive));
    out.print("tracker_degraded_no_imu="); out.println(yn(s.trackerDegradedNoImu));
    out.print("tracker_error_code="); out.println(s.trackerErrorCode);
    out.print("tracker_error_message="); out.println(s.trackerErrorMessage);
    out.print("tracker_error_sent="); out.println(s.trackerErrorSent);
    out.print("tracker_error_send_failures="); out.println(s.trackerErrorSendFailures);
    out.print("send_failures="); out.println(s.sendFailures);
    out.print("udp_begin_failures="); out.println(s.udpBeginFailures);
    out.print("server_silence_resets="); out.println(s.serverSilenceResets);
    out.print("wifi_lost_resets="); out.println(s.wifiLostResets);
    out.print("udp_reopen_requests="); out.println(s.udpReopenRequests);
    out.print("udp_reopen_suppressed_recent_rx="); out.println(s.udpReopenSuppressedRecentRx);
    serviceNonCliRuntime(ctx);
    out.print("ping_received="); out.println(s.pingReceived);
    out.print("pong_sent="); out.println(s.pongSent);
    out.print("unknown_packets_received="); out.println(s.unknownPacketsReceived);
    out.print("mag_support_enabled="); out.println(yn(s.magSupportEnabled));
    out.print("mag_enabled="); out.println(yn(s.magEnabled));
    out.print("sensor_config=0x"); out.println(s.sensorConfig, HEX);
    out.print("last_rssi_dbm="); out.println(s.lastRssiDbm);
    out.print("last_signal_strength_dbm="); out.println(static_cast<int>(s.lastSignalStrengthDbm));
    out.print("last_temperature_valid="); out.println(yn(s.lastTemperatureValid));
    out.print("last_temperature_c="); out.println(s.lastTemperatureC, 2);
    out.print("battery_telemetry_enabled="); out.println(yn(s.batteryTelemetryEnabled));
    out.print("battery_sent="); out.println(s.batterySent);
    out.print("battery_send_failures="); out.println(s.batterySendFailures);
    out.print("last_battery_valid="); out.println(yn(s.lastBatteryValid));
    out.print("last_battery_voltage_v="); out.println(s.lastBatteryVoltage, 3);
    out.print("last_battery_percentage="); out.println(s.lastBatteryPercentage, 1);
    serviceNonCliRuntime(ctx);
    out.print("last_rotation_confidence="); out.println(s.lastRotationConfidence, 4);
    out.println("# use 'slime debug' for full counters/timestamps");
}

void printSlimeDebug(TrackerSerialCommandContext& ctx,
                     Stream& out,
                     const SlimeVROutputRuntimeStatus& s) {
    char ipBuf[24];
    out.println("# SLIMEVR DEBUG");
    out.print("enabled="); out.println(yn(s.enabled));
    out.print("state="); out.println(slimevrOutputStateName(s.state));
    out.print("wifi_connected="); out.println(yn(s.wifiConnected));
    out.print("udp_ready="); out.println(yn(s.udpReady));
    out.print("local_port="); out.println(s.localPort);
    out.print("discovery_enabled="); out.println(yn(s.discoveryEnabled));
    out.print("manual_server_enabled="); out.println(yn(s.manualServerEnabled));
    out.print("manual_server_host="); out.println(s.manualServerHost);
    out.print("manual_server_resolved="); out.println(yn(s.manualServerResolved));
    out.print("manual_server_ip="); out.println(s.manualServerIpv4 ? udpIpv4ToCString(s.manualServerIpv4, ipBuf, sizeof(ipBuf)) : "0.0.0.0");
    out.print("manual_server_resolve_attempts="); out.println(s.manualServerResolveAttempts);
    out.print("manual_server_resolve_failures="); out.println(s.manualServerResolveFailures);
    out.print("manual_server_handshakes_sent="); out.println(s.manualServerHandshakesSent);
    out.print("server_found="); out.println(yn(s.serverFound));
    out.print("server_ip="); out.println(s.serverIpv4 ? udpIpv4ToCString(s.serverIpv4, ipBuf, sizeof(ipBuf)) : "0.0.0.0");
    out.print("server_port="); out.println(s.serverPort);
    out.print("sensor_id="); out.println(s.sensorId);
    serviceNonCliRuntime(ctx);
    out.print("tracker_error_active="); out.println(yn(s.trackerErrorActive));
    out.print("tracker_degraded_no_imu="); out.println(yn(s.trackerDegradedNoImu));
    out.print("tracker_error_code="); out.println(s.trackerErrorCode);
    out.print("tracker_health_revision="); out.println(s.trackerHealthRevision);
    out.print("tracker_error_message="); out.println(s.trackerErrorMessage);
    out.print("protocol_version="); out.println(s.protocolVersion);
    printMotionFrameContract(ctx, out, s);
    out.print("board_type="); out.println(s.boardType);
    out.print("imu_type="); out.println(s.imuType);
    out.print("mcu_type="); out.println(s.mcuType);
    serviceNonCliRuntime(ctx);
    out.print("handshakes_sent="); out.println(s.handshakesSent);
    out.print("sensor_info_sent="); out.println(s.sensorInfoSent);
    out.print("sensor_info_sync_state="); out.println(slimevrSensorInfoSyncStateName(s.sensorInfoSyncState));
    out.print("sensor_info_dirty="); out.println(yn(s.sensorInfoDirty));
    out.print("sensor_info_ack_received="); out.println(s.sensorInfoAckReceived);
    out.print("sensor_info_ack_malformed="); out.println(s.sensorInfoAckMalformed);
    out.print("sensor_info_ack_mismatch="); out.println(s.sensorInfoAckMismatch);
    out.print("sensor_info_local_status="); out.println(s.sensorInfoLocalStatus);
    out.print("sensor_info_local_config=0x"); out.println(s.sensorInfoLocalConfig, HEX);
    out.print("sensor_info_local_rest_calibration="); out.println(yn(s.sensorInfoLocalRestCalibration));
    out.print("sensor_info_ack_status="); out.println(s.sensorInfoAckStatus);
    out.print("sensor_info_ack_config=0x"); out.println(s.sensorInfoAckConfig, HEX);
    out.print("sensor_info_ack_rest_calibration="); out.println(yn(s.sensorInfoAckRestCalibration));
    out.print("heartbeat_sent="); out.println(s.heartbeatSent);
    out.print("motion_packet_mode="); out.println(slimevrMotionPacketModeName(s.motionPacketMode));
    out.print("packet23_available="); out.println(s.compactMotionAvailable ? "yes" : "no");
    out.print("packet23_enabled="); out.println(s.compactMotionEnabled ? "yes" : "no");
    out.print("bundle_negotiation_enabled="); out.println(s.bundleNegotiationEnabled ? "yes" : "no");
    out.print("server_feature_flags_available="); out.println(s.serverFeatureFlagsAvailable ? "yes" : "no");
    out.print("server_bundle_supported="); out.println(s.serverBundleSupported ? "yes" : "no");
    out.print("server_compact_bundle_supported="); out.println(s.serverCompactBundleSupported ? "yes" : "no");
    out.print("fallback_acceleration_rate_hz="); out.println(s.fallbackAccelerationRateHz);
    out.print("bundled_motion_sent="); out.println(s.bundledMotionSent);
    out.print("bundled_motion_send_failures="); out.println(s.bundledMotionSendFailures);
    out.print("acceleration_rate_limited="); out.println(s.accelerationRateLimited);
    out.print("feature_flags_sent="); out.println(s.featureFlagsSent);
    out.print("feature_flags_send_failures="); out.println(s.featureFlagsSendFailures);
    out.print("rotation_sent="); out.println(s.rotationSent);
    out.print("acceleration_sent="); out.println(s.accelerationSent);
    out.print("compact_motion_sent="); out.println(s.compactMotionSent);
    out.print("compact_motion_send_failures="); out.println(s.compactMotionSendFailures);
    out.print("acceleration_skipped_invalid="); out.println(s.accelerationSkippedInvalid);
    out.print("acceleration_skipped_configuration="); out.println(s.accelerationSkippedConfiguration);
    out.print("acceleration_skipped_component_missing="); out.println(s.accelerationSkippedComponentMissing);
    out.print("acceleration_skipped_pair_degraded="); out.println(s.accelerationSkippedPairDegraded);
    out.print("acceleration_skipped_saturated="); out.println(s.accelerationSkippedSaturated);
    out.print("acceleration_skipped_nonfinite="); out.println(s.accelerationSkippedNonFinite);
    out.print("acceleration_skipped_other="); out.println(s.accelerationSkippedOther);
    out.print("acceleration_send_failures="); out.println(s.accelerationSendFailures);
    out.print("signal_strength_sent="); out.println(s.signalStrengthSent);
    out.print("temperature_sent="); out.println(s.temperatureSent);
    out.print("battery_sent="); out.println(s.batterySent);
    out.print("battery_send_failures="); out.println(s.batterySendFailures);
    serviceNonCliRuntime(ctx);
    out.print("magnetometer_accuracy_sent="); out.println(s.magnetometerAccuracySent);
    out.print("tap_sent="); out.println(s.tapSent);
    out.print("tap_send_failures="); out.println(s.tapSendFailures);
    out.print("tracker_error_sent="); out.println(s.trackerErrorSent);
    out.print("tracker_error_send_failures="); out.println(s.trackerErrorSendFailures);
    out.print("last_tap_value="); out.println(s.lastTapValue);
    out.println("# note: mag support is advertised via SensorInfo.sensor_config; packet 18 is not sent as periodic telemetry");
    out.print("mag_support_enabled="); out.println(yn(s.magSupportEnabled));
    out.print("mag_enabled="); out.println(yn(s.magEnabled));
    out.print("sensor_config=0x"); out.println(s.sensorConfig, HEX);
    serviceNonCliRuntime(ctx);
    out.print("signal_telemetry_enabled="); out.println(yn(s.signalTelemetryEnabled));
    out.print("temperature_telemetry_enabled="); out.println(yn(s.temperatureTelemetryEnabled));
    out.print("battery_telemetry_enabled="); out.println(yn(s.batteryTelemetryEnabled));
    out.print("telemetry_interval_ms="); out.println(s.telemetryIntervalMs);
    out.print("signal_telemetry_interval_ms="); out.println(s.signalTelemetryIntervalMs);
    out.print("temperature_telemetry_interval_ms="); out.println(s.temperatureTelemetryIntervalMs);
    out.print("battery_telemetry_interval_ms="); out.println(s.batteryTelemetryIntervalMs);
    serviceNonCliRuntime(ctx);
    out.print("last_signal_strength_dbm="); out.println(static_cast<int>(s.lastSignalStrengthDbm));
    out.print("last_rssi_dbm="); out.println(s.lastRssiDbm);
    out.print("last_temperature_valid="); out.println(yn(s.lastTemperatureValid));
    out.print("last_temperature_c="); out.println(s.lastTemperatureC, 2);
    out.print("last_battery_valid="); out.println(yn(s.lastBatteryValid));
    out.print("last_battery_voltage_v="); out.println(s.lastBatteryVoltage, 3);
    out.print("last_battery_percentage="); out.println(s.lastBatteryPercentage, 1);
    serviceNonCliRuntime(ctx);
    out.print("rotation_no_snapshot="); out.println(s.rotationNoSnapshot);
    out.print("rotation_duplicate_snapshot="); out.println(s.rotationDuplicateSnapshot);
    out.print("rotation_suppressed_by_error="); out.println(s.rotationSuppressedByError);
    out.print("rotation_send_due="); out.println(s.rotationSendDue);
    out.print("rotation_rate_limited="); out.println(s.rotationRateLimited);
    out.print("service_updates="); out.println(s.serviceUpdates);
    out.print("service_skips="); out.println(s.serviceSkips);
    out.print("rotation_rate_hz="); out.println(s.rotationRateHz);
    out.print("prepared_output_available="); out.println(yn(s.preparedOutputAvailable));
    serviceNonCliRuntime(ctx);
    out.print("next_packet_number="); out.println(s.nextPacketNumber);
    out.print("packets_received="); out.println(s.packetsReceived);
    out.print("foreign_endpoint_packets_dropped="); out.println(s.foreignEndpointPacketsDropped);
    out.print("pre_session_packets_dropped="); out.println(s.preSessionPacketsDropped);
    out.print("malformed_packets="); out.println(s.malformedPackets);
    out.print("malformed_datagram_length="); out.println(s.malformedDatagramLength);
    out.print("malformed_heartbeat="); out.println(s.malformedHeartbeat);
    out.print("malformed_ping="); out.println(s.malformedPing);
    out.print("malformed_feature_flags="); out.println(s.malformedFeatureFlags);
    out.print("malformed_set_config_flag="); out.println(s.malformedSetConfigFlag);
    out.print("malformed_protocol_change="); out.println(s.malformedProtocolChange);
    out.print("malformed_unknown_raw="); out.println(s.malformedUnknownRaw);
    out.print("discovery_responses="); out.println(s.discoveryResponses);
    out.print("heartbeat_received="); out.println(s.heartbeatReceived);
    out.print("ping_received="); out.println(s.pingReceived);
    out.print("pong_sent="); out.println(s.pongSent);
    out.print("feature_flags_received="); out.println(s.featureFlagsReceived);
    out.print("set_config_flag_received="); out.println(s.setConfigFlagReceived);
    out.print("set_config_flag_applied="); out.println(s.setConfigFlagApplied);
    out.print("set_config_flag_apply_failures="); out.println(s.setConfigFlagApplyFailures);
    out.print("set_config_flag_ignored="); out.println(s.setConfigFlagIgnored);
    out.print("ack_config_sent="); out.println(s.ackConfigSent);
    out.print("ack_config_send_failures="); out.println(s.ackConfigSendFailures);
    out.print("user_action_sent="); out.println(s.userActionSent);
    out.print("user_action_send_failures="); out.println(s.userActionSendFailures);
    out.print("last_user_action="); out.println(slimevrUserActionName(s.lastUserAction));
    out.print("protocol_change_received="); out.println(s.protocolChangeReceived);
    out.print("protocol_change_ignored="); out.println(s.protocolChangeIgnored);
    out.print("unknown_packets_received="); out.println(s.unknownPacketsReceived);
    serviceNonCliRuntime(ctx);
    out.print("last_ping_id="); out.println(s.lastPingId);
    out.print("last_server_feature_flags=0x"); out.println(s.lastServerFeatureFlags, HEX);
    out.print("last_set_config_sensor_id="); out.println(s.lastSetConfigSensorId);
    out.print("last_set_config_type=0x"); out.println(s.lastSetConfigType, HEX);
    out.print("last_set_config_state="); out.println(yn(s.lastSetConfigState));
    out.print("last_set_config_applied="); out.println(yn(s.lastSetConfigApplied));
    out.print("last_protocol_target="); out.println(s.lastProtocolTarget);
    out.print("last_protocol_version="); out.println(s.lastProtocolVersion);
    out.print("last_unknown_packet_type="); out.println(s.lastUnknownPacketType);
    serviceNonCliRuntime(ctx);
    out.print("send_failures="); out.println(s.sendFailures);
    out.print("udp_begin_failures="); out.println(s.udpBeginFailures);
    out.print("server_silence_resets="); out.println(s.serverSilenceResets);
    out.print("wifi_lost_resets="); out.println(s.wifiLostResets);
    out.print("udp_reopen_requests="); out.println(s.udpReopenRequests);
    out.print("udp_reopen_suppressed_recent_rx="); out.println(s.udpReopenSuppressedRecentRx);
    out.print("consecutive_send_failures="); out.println(s.consecutiveSendFailures);
    serviceNonCliRuntime(ctx);
    out.print("last_handshake_ms="); out.println(s.lastHandshakeMs);
    out.print("last_incoming_packet_ms="); out.println(s.lastIncomingPacketMs);
    out.print("last_state_change_ms="); out.println(s.lastStateChangeMs);
    out.print("last_rotation_ms="); out.println(s.lastRotationMs);
    out.print("last_rotation_snapshot_sequence="); out.println(s.lastRotationSnapshotSequence);
    out.print("last_rotation_runtime_sample="); out.println(s.lastRotationRuntimeSample);
    out.print("last_rotation_timestamp_us="); tracker_serial_detail::printU64Dec(out, s.lastRotationTimestampUs); out.println();
    out.print("last_rotation_quality_flags=0x"); out.println(s.lastRotationQualityFlags, HEX);
    out.print("last_rotation_confidence="); out.println(s.lastRotationConfidence, 4);
    out.print("last_rotation_snapshot_age_us="); out.println(s.lastRotationSnapshotAgeUs);
}

void printHelp(Stream& out) {
    out.println("slime status");
    out.println("slime debug");
    out.println("slime start");
    out.println("slime stop");
    out.println("slime reconnect");
    out.println("slime rate <hz>");
    out.println("slime action yaw|full|mounting|pause");
    out.println("slime tap-action off|yaw|full|mounting|pause [save]");
    out.println("slime counters reset");
}

} // namespace

bool trackerSerialDispatchSlimeVRCommand(TrackerSerialCommandContext& ctx, int argc, char** argv) {
    if (argc <= 0 || !argv || !argv[0]) return false;
    if (!tracker_serial_detail::eqIgnoreCase(argv[0], "slime") &&
        !tracker_serial_detail::eqIgnoreCase(argv[0], "slimevr")) {
        return false;
    }

    Stream& out = outFor(ctx);
    if (!ctx.slimevrRuntime) {
        tracker_serial_detail::printErr(out, "SlimeVR runtime not available");
        return true;
    }

    if (argc < 2 || tracker_serial_detail::eqIgnoreCase(argv[1], "status")) {
        printSlimeStatusBrief(ctx, out, ctx.slimevrRuntime->status());
        return true;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "debug")) {
        printSlimeDebug(ctx, out, ctx.slimevrRuntime->status());
        return true;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "help") || tracker_serial_detail::eqIgnoreCase(argv[1], "?")) {
        printHelp(out);
        return true;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "start")) {
        if (!ctx.networkConfig) {
            tracker_serial_detail::printErr(out, "network config not available");
            return true;
        }
        ctx.networkConfig->sanitize();
        stopLocalSerialStreamForSlime(ctx);
        ctx.slimevrRuntime->configure(makeConfigFromNetwork(ctx, *ctx.networkConfig, slimeRotationRateHzFromConfig(ctx)));
        tracker_serial_detail::printOk(out, "SlimeVR output started");
        out.println("# use: slime status");
        return true;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "stop")) {
        ctx.slimevrRuntime->stop();
        tracker_serial_detail::printOk(out, "SlimeVR output stopped");
        return true;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "reconnect") || tracker_serial_detail::eqIgnoreCase(argv[1], "restart")) {
        if (!ctx.networkConfig) {
            tracker_serial_detail::printErr(out, "network config not available");
            return true;
        }
        ctx.networkConfig->sanitize();
        stopLocalSerialStreamForSlime(ctx);
        ctx.slimevrRuntime->configure(makeConfigFromNetwork(ctx, *ctx.networkConfig, slimeRotationRateHzFromConfig(ctx)));
        ctx.slimevrRuntime->restart();
        tracker_serial_detail::printOk(out, "SlimeVR output restarted");
        return true;
    }


    if (tracker_serial_detail::eqIgnoreCase(argv[1], "rate")) {
        if (!ctx.config) {
            tracker_serial_detail::printErr(out, "config not available");
            return true;
        }
        if (argc < 3) {
            tracker_serial_detail::printErr(out, "usage: slime rate <hz>");
            return true;
        }
        uint32_t hz = 0;
        if (!tracker_serial_detail::parseU32(argv[2], hz) || hz == 0 || hz > 1000) {
            tracker_serial_detail::printErr(out, "invalid slime rate; expected 1..1000");
            return true;
        }
        ctx.config->data.output.outputRateHz = static_cast<uint16_t>(hz);
        ctx.config->data.output.packetFormat = 0;
        ctx.config->updateCrc();
        if (ctx.networkConfig) {
            ctx.networkConfig->sanitize();
            ctx.slimevrRuntime->configure(makeConfigFromNetwork(ctx, *ctx.networkConfig, static_cast<uint16_t>(hz)));
        }
        tracker_serial_detail::printOk(out, "SlimeVR rotation rate set");
        return true;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "action")) {
        if (argc != 3) {
            tracker_serial_detail::printErr(out, "usage: slime action yaw|full|mounting|pause");
            return true;
        }
        SlimeVRUserAction action = SlimeVRUserAction::None;
        if (!parseSlimeVRUserActionName(argv[2], action) || action == SlimeVRUserAction::None) {
            tracker_serial_detail::printErr(out, "invalid action; expected yaw|full|mounting|pause");
            return true;
        }
        if (!ctx.slimevrRuntime->sendUserAction(action)) {
            tracker_serial_detail::printErr(out, "SlimeVR UserAction send failed");
            return true;
        }
        out.print("# OK SlimeVR UserAction sent action=");
        out.println(slimevrUserActionName(action));
        return true;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "tap-action") ||
        tracker_serial_detail::eqIgnoreCase(argv[1], "tap_action")) {
        if ((argc != 3 && argc != 4) || !ctx.networkConfig ||
            (argc == 4 && !tracker_serial_detail::eqIgnoreCase(argv[3], "save"))) {
            tracker_serial_detail::printErr(out, "usage: slime tap-action off|yaw|full|mounting|pause [save]");
            return true;
        }
        SlimeVRUserAction action = SlimeVRUserAction::None;
        if (!parseSlimeVRUserActionName(argv[2], action)) {
            tracker_serial_detail::printErr(out, "invalid tap action");
            return true;
        }
        const bool persist = argc == 4;
        TrackerNetworkConfig candidate = *ctx.networkConfig;
        candidate.setTapUserAction(action);
        if (persist) {
            if (!ctx.networkConfigStore) {
                tracker_serial_detail::printErr(out, "network config store not available");
                return true;
            }
            if (!ctx.networkConfigStore->save(candidate)) {
                out.print("# ERR tap action save failed: ");
                out.println(ctx.networkConfigStore->lastErrorName());
                return true;
            }
        }
        *ctx.networkConfig = candidate;
        if (ctx.tapRuntime) ctx.tapRuntime->setPhysicalTapUserAction(action);
        out.print("# OK tap UserAction mapping=");
        out.print(slimevrUserActionName(action));
        out.print(" persisted=");
        out.println(persist ? "yes" : "no");
        return true;
    }

    if (tracker_serial_detail::eqIgnoreCase(argv[1], "counters")) {
        if (argc >= 3 && tracker_serial_detail::eqIgnoreCase(argv[2], "reset")) {
            ctx.slimevrRuntime->resetCounters();
            tracker_serial_detail::printOk(out, "SlimeVR counters reset");
            return true;
        }
        tracker_serial_detail::printErr(out, "usage: slime counters reset");
        return true;
    }

    tracker_serial_detail::printErr(out, "unknown slime command; use slime help");
    return true;
}

} // namespace tracker

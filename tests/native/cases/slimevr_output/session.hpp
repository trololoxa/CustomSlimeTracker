#pragma once
static void testSessionLifecycle(TestContext& ctx, TrackerWifiManager& wifi, FakeUdp& udp, FakeSnapshotSource& snapshots,
    SlimeVROutputRuntime& rt, FakeConfigFlagSink& configFlagSink,
    SlimeVROutputRuntimeConfig& cfg) {
    // 0023gj regression: Connect Trackers can provision an already-connected
    // tracker with unchanged credentials. Wi-Fi stays connected here, so only
    // an explicit SlimeVR session restart can force a new discovery and fresh
    // SensorInfo registration lifecycle.
    {
        FakeUdp reconnectUdp;
        SlimeVROutputRuntime reconnectRt;
        reconnectRt.begin(reconnectUdp, wifi, FakeSnapshotSource::copy, &snapshots);
        reconnectRt.configure(cfg);
        reconnectRt.update(1u);
        CHECK(ctx, reconnectUdp.sent.size() == 1u);

        reconnectUdp.incoming = {
            static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
            'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
        };
        reconnectUdp.incomingRemote = UdpEndpoint{0xC0A80011UL, 6969};
        reconnectUdp.incomingPending = true;
        reconnectRt.update(100u);
        reconnectRt.update(201u);
        CHECK(ctx, reconnectRt.status().sensorInfoSent == 1u);

        reconnectUdp.incoming = makeSensorInfoAck(
            cfg.sensorId, static_cast<uint8_t>(SlimeVRSensorState::Online)
        );
        reconnectUdp.incomingPending = true;
        reconnectRt.update(211u);
        CHECK(ctx, reconnectRt.status().serverFound);
        CHECK(ctx, reconnectRt.status().sensorInfoSyncState ==
                   SlimeVRSensorInfoSyncState::Acknowledged);

        const size_t packetsBeforeProvisioningRestart = reconnectUdp.sent.size();
        const uint32_t stopCallsBeforeProvisioningRestart = reconnectUdp.stopCalls;
        reconnectRt.restart();
        SlimeVROutputRuntimeStatus reconnectStatus = reconnectRt.status();
        CHECK(ctx, wifi.connected());
        CHECK(ctx, !reconnectUdp.active());
        CHECK(ctx, reconnectUdp.stopCalls == stopCallsBeforeProvisioningRestart + 1u);
        CHECK(ctx, !reconnectStatus.serverFound);
        CHECK(ctx, reconnectStatus.state == SlimeVROutputState::WaitingForWifi);
        CHECK(ctx, reconnectStatus.serverIpv4 == 0u);
        CHECK(ctx, reconnectStatus.sensorInfoDirty);
        CHECK(ctx, reconnectStatus.sensorInfoSyncState ==
                   SlimeVRSensorInfoSyncState::Dirty);
        CHECK(ctx, reconnectStatus.featureNegotiationState ==
                   SlimeVRFeatureNegotiationState::NotStarted);

        reconnectRt.update(300u);
        CHECK(ctx, reconnectUdp.active());
        CHECK(ctx, reconnectUdp.sent.size() == packetsBeforeProvisioningRestart + 1u);
        CHECK(ctx, reconnectUdp.sent.back().data[3] ==
                   static_cast<uint8_t>(SlimeVRSendPacketType::Handshake));
        CHECK(ctx, readU64BeLocal(reconnectUdp.sent.back().data.data() + 4) == 0u);

        reconnectUdp.incoming = {
            static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
            'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
        };
        reconnectUdp.incomingPending = true;
        reconnectRt.update(310u);
        CHECK(ctx, reconnectRt.status().serverFound);

        reconnectRt.update(411u);
        CHECK(ctx, reconnectRt.status().sensorInfoSent == 2u);
        CHECK(ctx, reconnectRt.status().sensorInfoSyncState ==
                   SlimeVRSensorInfoSyncState::WaitingForAck);
        reconnectUdp.incoming = makeSensorInfoAck(
            cfg.sensorId, static_cast<uint8_t>(SlimeVRSensorState::Online)
        );
        reconnectUdp.incomingPending = true;
        reconnectRt.update(421u);
        CHECK(ctx, reconnectRt.status().sensorInfoSyncState ==
                   SlimeVRSensorInfoSyncState::Acknowledged);

        // The FIFO catch-up service may send only a due rotation. It must not
        // consume incoming packets or run feature/config/telemetry service.
        const SlimeVROutputRuntimeStatus beforeCritical = reconnectRt.status();
        const uint32_t sendsBeforeNotDue = reconnectUdp.sendCalls;
        CHECK(ctx, !reconnectRt.criticalRotationServiceDue(422u));
        for (uint32_t i = 0; i < 32u; ++i) {
            CHECK(ctx, !reconnectRt.updateCritical(422u));
        }
        CHECK(ctx, reconnectUdp.sendCalls == sendsBeforeNotDue);
        CHECK(ctx, reconnectRt.status().rotationSendDue ==
                   beforeCritical.rotationSendDue);

        snapshots.snapshot.sequence += 1u;
        snapshots.snapshot.runtimeSample += 1u;
        snapshots.snapshot.publishedAtMcuUs = 430000u;
        reconnectUdp.incoming = makeServerPacket(
            static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags), {0x01});
        reconnectUdp.incomingRemote = UdpEndpoint{0xC0A80011UL, 6969};
        reconnectUdp.incomingPending = true;
        trackerTestSetMicros(431000u);
        CHECK(ctx, reconnectRt.criticalRotationServiceDue(431u));
        CHECK(ctx, reconnectRt.updateCritical(431u));
        const SlimeVROutputRuntimeStatus afterCritical = reconnectRt.status();
        CHECK(ctx, afterCritical.rotationSent == beforeCritical.rotationSent + 1u);
        CHECK(ctx, afterCritical.serviceUpdates == beforeCritical.serviceUpdates);
        CHECK(ctx, afterCritical.featureFlagsReceived ==
                   beforeCritical.featureFlagsReceived);
        CHECK(ctx, reconnectUdp.incomingPending);

        reconnectRt.update(431u);
        CHECK(ctx, !reconnectUdp.incomingPending);
        CHECK(ctx, reconnectRt.status().featureFlagsReceived ==
                   beforeCritical.featureFlagsReceived + 1u);

        snapshots.snapshot.sequence = 1u;
        snapshots.snapshot.runtimeSample = 123u;
        snapshots.snapshot.publishedAtMcuUs = 1200000u;
    }

    rt.configure(cfg);

    rt.update(1000);
    CHECK(ctx, udp.beginCalls == 1);
    CHECK(ctx, udp.active());
    CHECK(ctx, udp.sent.size() == 1u);
    CHECK(ctx, udp.sent[0].endpoint.ipv4 == SLIMEVR_DISCOVERY_BROADCAST_IPV4);
    CHECK(ctx, udp.sent[0].endpoint.port == 6969);
    CHECK(ctx, udp.sent[0].data.size() >= SLIMEVR_PACKET_HEADER_SIZE);
    CHECK(ctx, udp.sent[0].data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::Handshake));
    CHECK(ctx, readU64BeLocal(udp.sent[0].data.data() + 4) == 0u);
    constexpr size_t handshakeFirmwareLengthOffset = SLIMEVR_PACKET_HEADER_SIZE + 7u * sizeof(uint32_t);
    CHECK(ctx, udp.sent[0].data.size() > handshakeFirmwareLengthOffset);
    const size_t firmwareLength = udp.sent[0].data[handshakeFirmwareLengthOffset];
    CHECK(ctx, firmwareLength == std::strlen(trackerBuildSlimeVRFirmwareVersion()));
    CHECK(ctx, udp.sent[0].data.size() >= handshakeFirmwareLengthOffset + 1u + firmwareLength);
    CHECK(ctx, std::memcmp(
        udp.sent[0].data.data() + handshakeFirmwareLengthOffset + 1u,
        trackerBuildSlimeVRFirmwareVersion(),
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
    CHECK(ctx, st.accelerationSent == 0u);
    CHECK(ctx, st.accelerationSuppressedDuringNegotiation == 1u);
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
    CHECK(ctx, !sawAcceleration);

    // A well-formed ACK for another sensor must not acknowledge our latest
    // SensorInfo state or stop resend.
    udp.incoming = makeSensorInfoAck(
        static_cast<uint8_t>(cfg.sensorId + 1u),
        static_cast<uint8_t>(SlimeVRSensorState::Online)
    );
    udp.incomingRemote = UdpEndpoint{0xC0A80001UL, 6969};
    udp.incomingPending = true;
    rt.update(1211);
    CHECK(ctx, rt.status().sensorInfoAckReceived == 1u);
    CHECK(ctx, rt.status().sensorInfoAckMismatch == 1u);
    CHECK(ctx, rt.status().sensorInfoSyncState == SlimeVRSensorInfoSyncState::WaitingForAck);
    CHECK(ctx, rt.status().sensorInfoDirty);

    udp.incoming = makeSensorInfoAck(
        cfg.sensorId, static_cast<uint8_t>(SlimeVRSensorState::Online)
    );
    udp.incomingRemote = UdpEndpoint{0xC0A80001UL, 6969};
    udp.incomingPending = true;
    rt.update(1221);
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

    rt.update(1222);
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
    rt.update(1231);
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
    rt.update(1241);
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
    rt.update(1251);
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
    snapshots.snapshot.publishedAtMcuUs = 1260500u;
    const size_t sentBeforeBundle = udp.sent.size();
    rt.update(1261);
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
    CHECK(ctx, st.accelerationSent == 1u);
    CHECK(ctx, st.separateToBundleTransitions == 1u);
    CHECK(ctx, udp.sent.size() == sentBeforeBundle + 1u);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::Bundle));
    CHECK(ctx, udp.sent.back().data.size() == 56u);
    CHECK(ctx, udp.sent.back().data[17] == static_cast<uint8_t>(SlimeVRSendPacketType::RotationData));
    CHECK(ctx, udp.sent.back().data[42] == static_cast<uint8_t>(SlimeVRSendPacketType::Accel));

    // Hard-invalid acceleration falls back to a normal rotation packet.
    snapshots.snapshot.sequence = 3u;
    snapshots.snapshot.runtimeSample = 125u;
    snapshots.snapshot.publishedAtMcuUs = 1270500u;
    snapshots.snapshot.linearAccelerationValid = false;
    snapshots.snapshot.linearAccelerationInvalidFlags =
        prepared_output_motion_flags::ACCEL_COMPONENT_MISSING |
        prepared_output_motion_flags::PAIR_COHERENCY_DEGRADED;
    const size_t sentBeforeInvalidAcceleration = udp.sent.size();
    rt.update(1271);
    CHECK(ctx, rt.status().rotationSent == 3u);
    CHECK(ctx, rt.status().accelerationSent == 1u);
    CHECK(ctx, rt.status().accelerationSkippedInvalid == 1u);
    CHECK(ctx, rt.status().accelerationSkippedComponentMissing == 1u);
    CHECK(ctx, rt.status().accelerationSkippedPairDegraded == 1u);
    CHECK(ctx, udp.sent.size() == sentBeforeInvalidAcceleration + 1u);
    CHECK(ctx, udp.sent.back().data[3] == static_cast<uint8_t>(SlimeVRSendPacketType::RotationData));

    // A failed bundle is one physical datagram failure and two logical motion
    // delivery failures. Packet 23 stays compiled but disabled by default.
    snapshots.snapshot.sequence = 4u;
    snapshots.snapshot.publishedAtMcuUs = 1280500u;
    snapshots.snapshot.linearAccelerationValid = true;
    snapshots.snapshot.linearAccelerationInvalidFlags = prepared_output_motion_flags::NONE;
    udp.failPacketType = static_cast<int>(SlimeVRSendPacketType::Bundle);
    const size_t sentBeforeBundleFailure = udp.sent.size();
    rt.update(1281);
    udp.failPacketType = -1;
    CHECK(ctx, rt.status().rotationSent == 3u);
    CHECK(ctx, rt.status().accelerationSent == 1u);
    CHECK(ctx, rt.status().bundledMotionSendFailures == 1u);
    CHECK(ctx, rt.status().compactMotionSendFailures == 0u);
    CHECK(ctx, rt.status().rotationSendFailures == 1u);
    CHECK(ctx, rt.status().accelerationSendFailures == 1u);
    CHECK(ctx, udp.sent.size() == sentBeforeBundleFailure);
    snapshots.snapshot.sequence = 3u;

    cfg.hasCompletedRestCalibration = true;
    rt.configure(cfg);
    const size_t sentBeforeRestRefresh = udp.sent.size();
    rt.update(1290);
    CHECK(ctx, rt.status().hasCompletedRestCalibration);
    CHECK(ctx, rt.status().sensorInfoSent == 1u);
    CHECK(ctx, udp.sent.size() == sentBeforeRestRefresh);
    // Optional background control yields to the active TX-pressure backoff,
    // then retries immediately after the bounded interval.
    rt.update(1296);
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
    rt.update(1306);
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
    rt.update(6111);
    CHECK(ctx, rt.status().heartbeatReceived == 1);

    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::PingPong),
        {0x11, 0x22, 0x33, 0x44}
    );
    udp.incomingRemote = UdpEndpoint{0xC0A80001UL, 6969};
    udp.incomingPending = true;
    const size_t sentBeforePing = udp.sent.size();
    rt.update(6121);
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
    rt.update(6131);
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
    rt.update(6141);
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
    rt.update(6151);
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
    rt.update(6161);
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
    rt.update(6171);
    CHECK(ctx, rt.status().setConfigFlagIgnored == 2u);
    CHECK(ctx, configFlagSink.calls == 2u);

    // A repeated already-applied state is ACKed without another persistent apply.
    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::SetConfigFlag),
        {2, 0x00, 0x01, 0x00}
    );
    udp.incomingPending = true;
    const size_t sentBeforeIdempotentConfig = udp.sent.size();
    rt.update(6181);
    CHECK(ctx, configFlagSink.calls == 2u);
    CHECK(ctx, rt.status().ackConfigSent == 2u);
    CHECK(ctx, udp.sent.size() == sentBeforeIdempotentConfig + 1u);

    // Malformed control packets do not refresh liveness.
    const uint32_t incomingBeforeMalformed = rt.status().lastIncomingPacketMs;
    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::PingPong), {0x11, 0x22}
    );
    udp.incomingPending = true;
    rt.update(6191);
    CHECK(ctx, rt.status().malformedPing == 1u);
    CHECK(ctx, rt.status().lastIncomingPacketMs == incomingBeforeMalformed);

    // Headerless legacy control packets are malformed in the modern session
    // contract and cannot refresh liveness or invoke handlers.
    udp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::HeartBeat)
    };
    udp.incomingPending = true;
    rt.update(6201);
    CHECK(ctx, rt.status().malformedUnknownRaw == 1u);
    CHECK(ctx, rt.status().heartbeatReceived == 1u);
    CHECK(ctx, rt.status().lastIncomingPacketMs == incomingBeforeMalformed);

    // A datagram larger than the fixed RX buffer must not be interpreted from
    // its truncated prefix.
    udp.incoming.assign(600u, 0u);
    udp.incoming[3] = static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags);
    udp.incomingPending = true;
    rt.update(6211);
    CHECK(ctx, rt.status().malformedDatagramLength == 1u);
    CHECK(ctx, rt.status().featureFlagsReceived == 3u);
    CHECK(ctx, rt.status().lastIncomingPacketMs == incomingBeforeMalformed);

    udp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::ProtocolChange),
        {0x01, 0x13}
    );
    udp.incomingPending = true;
    rt.update(6221);
    CHECK(ctx, rt.status().protocolChangeReceived == 1);
    CHECK(ctx, rt.status().protocolChangeIgnored == 1);
    CHECK(ctx, rt.status().lastProtocolTarget == 1);
    CHECK(ctx, rt.status().lastProtocolVersion == 0x13);

    udp.incoming = makeServerPacket(0xFE, {});
    udp.incomingPending = true;
    rt.update(6231);
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

}

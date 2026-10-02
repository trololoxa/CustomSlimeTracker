#pragma once
static void testManualEndpointAndPolicy(TestContext& ctx, TrackerWifiManager& wifi, const SlimeVROutputRuntimeConfig& phaseCfg) {
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

    // Runtime policy is explicit and may change without tearing down the
    // discovered session. Quaternion-only must never emit acceleration.
    manualCfg.motionPacketPolicy = SlimeVRMotionPacketPolicy::QuaternionOnly;
    manualRt.configure(manualCfg);
    manualSnapshots.snapshot.sequence += 1u;
    const size_t quaternionOnlyBegin = manualUdp.sent.size();
    manualRt.update(1201u);
    CHECK(ctx, manualRt.status().motionPacketPolicy ==
               SlimeVRMotionPacketPolicy::QuaternionOnly);
    CHECK(ctx, manualRt.status().motionPacketMode ==
               SlimeVRMotionPacketMode::Rotation17Only);
    bool quaternionOnlyRotation = false;
    bool quaternionOnlyAcceleration = false;
    for (size_t i = quaternionOnlyBegin; i < manualUdp.sent.size(); ++i) {
        if (manualUdp.sent[i].data.size() < 4u) continue;
        const uint8_t type = manualUdp.sent[i].data[3];
        quaternionOnlyRotation |= type == static_cast<uint8_t>(SlimeVRSendPacketType::RotationData);
        quaternionOnlyAcceleration |= type == static_cast<uint8_t>(SlimeVRSendPacketType::Accel) ||
                                      type == static_cast<uint8_t>(SlimeVRSendPacketType::RotationAndAcceleration) ||
                                      type == static_cast<uint8_t>(SlimeVRSendPacketType::Bundle);
    }
    CHECK(ctx, quaternionOnlyRotation);
    CHECK(ctx, !quaternionOnlyAcceleration);

    manualCfg.motionPacketPolicy = SlimeVRMotionPacketPolicy::RotationAcceleration23;
    manualRt.configure(manualCfg);
    manualSnapshots.snapshot.sequence += 1u;
    const size_t packet23Begin = manualUdp.sent.size();
    manualRt.update(1221u);
    CHECK(ctx, manualRt.status().motionPacketMode ==
               SlimeVRMotionPacketMode::ExperimentalRotationAcceleration23);
    bool sawPacket23 = false;
    for (size_t i = packet23Begin; i < manualUdp.sent.size(); ++i) {
        if (manualUdp.sent[i].data.size() >= 4u &&
            manualUdp.sent[i].data[3] ==
                static_cast<uint8_t>(SlimeVRSendPacketType::RotationAndAcceleration)) {
            sawPacket23 = true;
        }
    }
    CHECK(ctx, sawPacket23);

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

}

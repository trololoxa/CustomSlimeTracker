#pragma once
static void testPressureRecovery(TestContext& ctx, TrackerWifiManager& wifi, const SlimeVROutputRuntimeConfig& phaseCfg) {
    // TX pressure uses elapsed motion-success age instead of burst density.
    // Intermittent ENOMEM with continuing fresh motion must keep the socket and
    // packet-100 negotiation intact; only a sustained no-success interval may
    // rebind, and full discovery requires a failed post-rebind interval.
    FakeUdp pressureUdp;
    FakeSnapshotSource pressureSnapshots;
    pressureSnapshots.snapshot.valid = true;
    pressureSnapshots.snapshot.sequence = 1u;
    pressureSnapshots.snapshot.q = Quat::identity();
    pressureSnapshots.snapshot.linearAccelerationValid = true;
    pressureSnapshots.snapshot.linearAccelerationDeviceG = Vec3::zero();
    pressureSnapshots.snapshot.confidence = 1.0f;

    SlimeVROutputRuntime pressureRt;
    pressureRt.begin(pressureUdp, wifi, FakeSnapshotSource::copy, &pressureSnapshots);
    pressureRt.configure(phaseCfg);
    pressureRt.update(1u);
    pressureUdp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    pressureUdp.incomingRemote = UdpEndpoint{0xC0A80003UL, 6969};
    pressureUdp.incomingPending = true;
    pressureRt.update(100u);
    pressureUdp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags), {0x01}
    );
    pressureUdp.incomingPending = true;
    pressureRt.update(110u);
    pressureRt.update(201u);
    CHECK(ctx, pressureRt.status().bundledMotionSent == 1u);

    pressureUdp.failPacketType = static_cast<int>(SlimeVRSendPacketType::Bundle);
    pressureUdp.failSendError = ENOMEM;
    pressureSnapshots.snapshot.sequence = 2u;
    pressureRt.update(211u);
    CHECK(ctx, pressureRt.status().txPressureFailures == 1u);
    CHECK(ctx, pressureRt.status().txPressureEpisodeActive);
    CHECK(ctx, pressureRt.status().txPressureState == SlimeVRTxPressureState::TransientPressure);
    const uint32_t sendsAfterFirstPressure = pressureUdp.sendCalls;
    CHECK(ctx, !pressureRt.sendTap(1u));
    CHECK(ctx, pressureUdp.sendCalls == sendsAfterFirstPressure);
    CHECK(ctx, pressureRt.status().txBackoffDrops == 1u);

    // A successful motion packet resumes delivery without rebind. The episode
    // closes only after a full stable interval, not after this one success.
    pressureUdp.failPacketType = -1;
    pressureSnapshots.snapshot.sequence = 3u;
    pressureRt.update(221u);
    CHECK(ctx, pressureRt.status().bundledMotionSent == 2u);
    CHECK(ctx, pressureRt.status().udpTransportRebindSuccesses == 0u);
    CHECK(ctx, pressureRt.status().txPressureEpisodeActive);
    for (uint32_t now = 231u; now <= 1231u; now += 10u) {
        pressureSnapshots.snapshot.sequence += 1u;
        pressureRt.update(now);
    }
    CHECK(ctx, !pressureRt.status().txPressureEpisodeActive);
    CHECK(ctx, pressureRt.status().txPressureStableResets == 1u);
    CHECK(ctx, pressureRt.status().udpTransportRebindSuccesses == 0u);

    // Eight intermittent pressure failures in the exact 32-attempt window are
    // still reported, but continuing successful motion prevents socket churn.
    FakeUdp densityUdp;
    FakeSnapshotSource densitySnapshots;
    densitySnapshots.snapshot = pressureSnapshots.snapshot;
    densitySnapshots.snapshot.sequence = 1u;
    SlimeVROutputRuntime densityRt;
    densityRt.begin(densityUdp, wifi, FakeSnapshotSource::copy, &densitySnapshots);
    densityRt.configure(phaseCfg);
    densityRt.update(1u);
    densityUdp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    densityUdp.incomingRemote = UdpEndpoint{0xC0A80003UL, 6969};
    densityUdp.incomingPending = true;
    densityRt.update(100u);
    densityUdp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags), {0x01}
    );
    densityUdp.incomingPending = true;
    densityRt.update(110u);
    densityRt.update(201u);
    for (uint32_t attempt = 0; attempt < 36u; ++attempt) {
        densityUdp.failPacketType = (attempt % 4u) == 0u
            ? static_cast<int>(SlimeVRSendPacketType::Bundle)
            : -1;
        densitySnapshots.snapshot.sequence += 1u;
        densityRt.update(211u + attempt * 10u);
    }
    CHECK(ctx, densityRt.status().txFailureWindowTrips >= 1u);
    CHECK(ctx, densityRt.status().udpTransportRebindSuccesses == 0u);
    CHECK(ctx, densityRt.status().udpFullReopenEscalations == 0u);
    CHECK(ctx, densityRt.status().serverFound);

    // Persistent pressure with recent server RX performs one local rebind only
    // after 500 ms without a successful motion datagram. Feature negotiation
    // and bundle mode survive the local socket replacement.
    FakeUdp sustainedUdp;
    FakeSnapshotSource sustainedSnapshots;
    sustainedSnapshots.snapshot = pressureSnapshots.snapshot;
    sustainedSnapshots.snapshot.sequence = 1u;
    SlimeVROutputRuntime sustainedRt;
    sustainedRt.begin(sustainedUdp, wifi, FakeSnapshotSource::copy, &sustainedSnapshots);
    sustainedRt.configure(phaseCfg);
    sustainedRt.update(1u);
    sustainedUdp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    sustainedUdp.incomingRemote = UdpEndpoint{0xC0A80003UL, 6969};
    sustainedUdp.incomingPending = true;
    sustainedRt.update(100u);
    sustainedUdp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags), {0x01}
    );
    sustainedUdp.incomingPending = true;
    sustainedRt.update(110u);
    sustainedRt.update(201u);
    sustainedUdp.failPacketType = static_cast<int>(SlimeVRSendPacketType::Bundle);
    for (uint32_t now = 211u; now <= 731u; now += 10u) {
        sustainedSnapshots.snapshot.sequence += 1u;
        sustainedRt.update(now);
    }
    SlimeVROutputRuntimeStatus sustainedStatus = sustainedRt.status();
    CHECK(ctx, sustainedStatus.udpTransportRebindRequests == 1u);
    CHECK(ctx, sustainedStatus.udpTransportRebindSuccesses == 1u);
    CHECK(ctx, sustainedStatus.udpReopenRequests == 0u);
    CHECK(ctx, sustainedStatus.serverFound);
    CHECK(ctx, sustainedStatus.serverFeatureFlagsAvailable);
    CHECK(ctx, sustainedStatus.motionPacketMode ==
               SlimeVRMotionPacketMode::Bundle100Rotation17Accel4);
    CHECK(ctx, sustainedStatus.txPressureState ==
               SlimeVRTxPressureState::AwaitingPostRebindSuccess);

    // Continued failure for another second after the rebind is the only
    // pressure path that escalates to a full discovery/session restart.
    for (uint32_t now = 741u; now <= 1751u && sustainedRt.status().serverFound; now += 10u) {
        sustainedSnapshots.snapshot.sequence += 1u;
        sustainedRt.update(now);
    }
    sustainedStatus = sustainedRt.status();
    CHECK(ctx, sustainedStatus.udpFullReopenEscalations == 1u);
    CHECK(ctx, sustainedStatus.udpReopenRequests == 1u);
    CHECK(ctx, !sustainedStatus.serverFound);
    CHECK(ctx, !sustainedStatus.serverFeatureFlagsAvailable);
    CHECK(ctx, sustainedStatus.motionPacketMode ==
               SlimeVRMotionPacketMode::SeparateRotation17Accel4);

    // During reconnect capability negotiation, only packet 17 is emitted. No
    // packet-4 fallback burst is allowed until bundle support is confirmed or
    // negotiation is explicitly unavailable.
    sustainedUdp.failPacketType = -1;
    sustainedRt.update(1761u);
    sustainedUdp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    sustainedUdp.incomingRemote = UdpEndpoint{0xC0A80003UL, 6969};
    sustainedUdp.incomingPending = true;
    sustainedRt.update(1800u);
    sustainedSnapshots.snapshot.sequence += 1u;
    sustainedRt.update(1901u);
    CHECK(ctx, sustainedRt.status().rotationSent > 0u);
    CHECK(ctx, sustainedRt.status().accelerationSuppressedDuringNegotiation > 0u);
    CHECK(ctx, sustainedRt.status().separateAccelerationDatagramsSent == 0u);

    // A new pressure episode shortly after a successful local rebind is
    // suppressed once until the bounded cooldown expires. The scheduler does
    // not re-count the same suppression on every 10 ms service tick.
    FakeUdp cooldownUdp;
    FakeSnapshotSource cooldownSnapshots;
    cooldownSnapshots.snapshot = pressureSnapshots.snapshot;
    cooldownSnapshots.snapshot.sequence = 1u;
    SlimeVROutputRuntime cooldownRt;
    cooldownRt.begin(cooldownUdp, wifi, FakeSnapshotSource::copy, &cooldownSnapshots);
    cooldownRt.configure(phaseCfg);
    cooldownRt.update(1u);
    cooldownUdp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    cooldownUdp.incomingRemote = UdpEndpoint{0xC0A80003UL, 6969};
    cooldownUdp.incomingPending = true;
    cooldownRt.update(100u);
    cooldownUdp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags), {0x01}
    );
    cooldownUdp.incomingPending = true;
    cooldownRt.update(110u);
    cooldownRt.update(201u);
    cooldownUdp.failPacketType = static_cast<int>(SlimeVRSendPacketType::Bundle);
    for (uint32_t now = 211u; now <= 731u; now += 10u) {
        cooldownSnapshots.snapshot.sequence += 1u;
        cooldownRt.update(now);
    }
    CHECK(ctx, cooldownRt.status().udpTransportRebindRequests == 1u);
    cooldownUdp.failPacketType = -1;
    for (uint32_t now = 751u; now <= 1761u; now += 10u) {
        cooldownSnapshots.snapshot.sequence += 1u;
        cooldownRt.update(now);
    }
    CHECK(ctx, !cooldownRt.status().txPressureEpisodeActive);
    cooldownUdp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::HeartBeat0), {}
    );
    cooldownUdp.incomingRemote = UdpEndpoint{0xC0A80003UL, 6969};
    cooldownUdp.incomingPending = true;
    cooldownRt.update(1765u);
    cooldownUdp.failPacketType = static_cast<int>(SlimeVRSendPacketType::Bundle);
    for (uint32_t now = 1771u; now <= 3001u; now += 10u) {
        cooldownSnapshots.snapshot.sequence += 1u;
        cooldownRt.update(now);
    }
    CHECK(ctx, cooldownRt.status().udpTransportRebindRequests == 1u);
    CHECK(ctx, cooldownRt.status().udpRebindSuppressedCooldown == 1u);
    CHECK(ctx, cooldownRt.status().udpFullReopenEscalations == 0u);

    // With stale server RX, persistent pressure must still last a full bounded
    // interval before direct full discovery; one isolated late failure is not
    // enough to destroy the session.
    FakeUdp staleUdp;
    FakeSnapshotSource staleSnapshots;
    staleSnapshots.snapshot = pressureSnapshots.snapshot;
    staleSnapshots.snapshot.sequence = 1u;
    SlimeVROutputRuntime staleRt;
    staleRt.begin(staleUdp, wifi, FakeSnapshotSource::copy, &staleSnapshots);
    staleRt.configure(phaseCfg);
    staleRt.update(1u);
    staleUdp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    staleUdp.incomingRemote = UdpEndpoint{0xC0A80003UL, 6969};
    staleUdp.incomingPending = true;
    staleRt.update(100u);
    staleUdp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags), {0x01}
    );
    staleUdp.incomingPending = true;
    staleRt.update(110u);
    staleRt.update(201u);
    staleUdp.failPacketType = static_cast<int>(SlimeVRSendPacketType::Bundle);
    for (uint32_t now = 3000u; now <= 4020u && staleRt.status().serverFound; now += 10u) {
        staleSnapshots.snapshot.sequence += 1u;
        staleRt.update(now);
    }
    CHECK(ctx, staleRt.status().udpTransportRebindRequests == 0u);
    CHECK(ctx, staleRt.status().udpReopenRequests == 1u);
    CHECK(ctx, !staleRt.status().serverFound);

    // A failed local rebind escalates fail-closed, but still only after the
    // sustained no-success gate has been met.
    FakeUdp rebindFailUdp;
    FakeSnapshotSource rebindFailSnapshots;
    rebindFailSnapshots.snapshot = pressureSnapshots.snapshot;
    rebindFailSnapshots.snapshot.sequence = 1u;
    SlimeVROutputRuntime rebindFailRt;
    rebindFailRt.begin(rebindFailUdp, wifi, FakeSnapshotSource::copy, &rebindFailSnapshots);
    rebindFailRt.configure(phaseCfg);
    rebindFailRt.update(1u);
    rebindFailUdp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    rebindFailUdp.incomingRemote = UdpEndpoint{0xC0A80003UL, 6969};
    rebindFailUdp.incomingPending = true;
    rebindFailRt.update(100u);
    rebindFailUdp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::FeatureFlags), {0x01}
    );
    rebindFailUdp.incomingPending = true;
    rebindFailRt.update(110u);
    rebindFailRt.update(201u);
    rebindFailUdp.failPacketType = static_cast<int>(SlimeVRSendPacketType::Bundle);
    for (uint32_t now = 211u; now <= 701u; now += 10u) {
        rebindFailSnapshots.snapshot.sequence += 1u;
        rebindFailRt.update(now);
    }
    rebindFailUdp.beginResult = false;
    rebindFailSnapshots.snapshot.sequence += 1u;
    rebindFailRt.update(711u);
    CHECK(ctx, rebindFailRt.status().udpTransportRebindRequests == 1u);
    CHECK(ctx, rebindFailRt.status().udpTransportRebindFailures == 1u);
    CHECK(ctx, rebindFailRt.status().udpReopenRequests == 1u);
    CHECK(ctx, !rebindFailRt.status().serverFound);

}

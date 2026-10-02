#pragma once
static SlimeVROutputRuntimeConfig testScheduler(TestContext& ctx,
    TrackerWifiManager& wifi, const SlimeVROutputRuntimeConfig& cfg) {
    // Scheduler phase-lock: a late update advances to the nearest future
    // deadline, records skipped periods, and still sends at most one current
    // snapshot instead of a catch-up burst.
    FakeUdp phaseUdp;
    FakeSnapshotSource phaseSnapshots;
    phaseSnapshots.snapshot.valid = true;
    phaseSnapshots.snapshot.sequence = 1u;
    phaseSnapshots.snapshot.q = Quat::identity();
    phaseSnapshots.snapshot.linearAccelerationValid = false;
    phaseSnapshots.snapshot.confidence = 1.0f;

    SlimeVROutputRuntime phaseRt;
    phaseRt.begin(phaseUdp, wifi, FakeSnapshotSource::copy, &phaseSnapshots);
    SlimeVROutputRuntimeConfig phaseCfg = cfg;
    phaseCfg.latestTemperatureValid = false;
    phaseCfg.latestBatteryValid = false;
    phaseCfg.signalTelemetryEnabled = false;
    phaseCfg.temperatureTelemetryEnabled = false;
    phaseCfg.batteryTelemetryEnabled = false;
    phaseRt.configure(phaseCfg);
    phaseRt.update(1u);

    phaseUdp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    phaseUdp.incomingRemote = UdpEndpoint{0xC0A80002UL, 6969};
    phaseUdp.incomingPending = true;
    phaseRt.update(100u); // Intentional send grace ends at 200 ms.

    phaseRt.update(201u);
    CHECK(ctx, phaseRt.status().rotationSent == 1u);
    CHECK(ctx, phaseRt.status().rotationMissedDeadlines == 0u);

    phaseSnapshots.snapshot.sequence = 2u;
    const size_t packetsBeforeLateTick = phaseUdp.sent.size();
    phaseRt.update(225u); // Deadlines 210 and 220 elapsed; send once.
    SlimeVROutputRuntimeStatus phaseStatus = phaseRt.status();
    CHECK(ctx, phaseStatus.rotationSent == 2u);
    CHECK(ctx, phaseStatus.rotationSendDue == 2u);
    CHECK(ctx, phaseStatus.rotationMissedDeadlines == 1u);
    CHECK(ctx, phaseStatus.rotationLateEvents == 2u);
    CHECK(ctx, phaseStatus.rotationLatenessMaxMs == 15u);
    CHECK(ctx, phaseUdp.sent.size() == packetsBeforeLateTick + 1u);

    phaseSnapshots.snapshot.sequence = 3u;
    phaseRt.update(226u);
    CHECK(ctx, phaseRt.status().rotationSent == 2u);
    phaseRt.update(230u);
    CHECK(ctx, phaseRt.status().rotationSent == 3u);
    CHECK(ctx, phaseRt.status().rotationMissedDeadlines == 1u);

    return phaseCfg;
}

static void testNegotiationFallback(TestContext& ctx, TrackerWifiManager& wifi, const SlimeVROutputRuntimeConfig& phaseCfg) {
    // Servers that do not answer FeatureFlags stay on packet 17 at the full
    // pose rate while coherent packet-4 acceleration is limited to 50 Hz.
    FakeUdp fallbackUdp;
    FakeSnapshotSource fallbackSnapshots;
    fallbackSnapshots.snapshot.valid = true;
    fallbackSnapshots.snapshot.sequence = 1u;
    fallbackSnapshots.snapshot.q = Quat::identity();
    fallbackSnapshots.snapshot.linearAccelerationValid = true;
    fallbackSnapshots.snapshot.linearAccelerationDeviceG = Vec3::zero();
    fallbackSnapshots.snapshot.confidence = 1.0f;

    SlimeVROutputRuntime fallbackRt;
    fallbackRt.begin(fallbackUdp, wifi, FakeSnapshotSource::copy, &fallbackSnapshots);
    fallbackRt.configure(phaseCfg);
    fallbackRt.update(1u);
    fallbackUdp.incoming = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    fallbackUdp.incomingRemote = UdpEndpoint{0xC0A80004UL, 6969};
    fallbackUdp.incomingPending = true;
    fallbackRt.update(100u);
    fallbackRt.update(201u);
    CHECK(ctx, fallbackRt.status().motionPacketMode ==
               SlimeVRMotionPacketMode::SeparateRotation17Accel4);
    CHECK(ctx, fallbackRt.status().rotationSent == 1u);
    CHECK(ctx, fallbackRt.status().accelerationSent == 0u);
    CHECK(ctx, fallbackRt.status().accelerationSuppressedDuringNegotiation == 1u);

    fallbackSnapshots.snapshot.sequence = 2u;
    fallbackRt.update(211u);
    CHECK(ctx, fallbackRt.status().rotationSent == 2u);
    CHECK(ctx, fallbackRt.status().accelerationSent == 0u);
    CHECK(ctx, fallbackRt.status().accelerationSuppressedDuringNegotiation == 2u);

    fallbackSnapshots.snapshot.sequence = 3u;
    fallbackRt.update(221u);
    CHECK(ctx, fallbackRt.status().rotationSent == 3u);
    CHECK(ctx, fallbackRt.status().accelerationSent == 0u);

    for (uint8_t i = fallbackRt.status().featureFlagsRequestAttempts;
         i < TRACKER_SLIMEVR_FEATURE_FLAGS_REQUEST_ATTEMPTS; ++i) {
        const uint32_t now = 701u + static_cast<uint32_t>(i) * 500u;
        fallbackUdp.incoming = makeServerPacket(
            static_cast<uint8_t>(SlimeVRReceivePacketType::HeartBeat0), {}
        );
        fallbackUdp.incomingRemote = UdpEndpoint{0xC0A80004UL, 6969};
        fallbackUdp.incomingPending = true;
        fallbackSnapshots.snapshot.sequence += 1u;
        fallbackRt.update(now);
    }
    fallbackUdp.incoming = makeServerPacket(
        static_cast<uint8_t>(SlimeVRReceivePacketType::HeartBeat0), {}
    );
    fallbackUdp.incomingRemote = UdpEndpoint{0xC0A80004UL, 6969};
    fallbackUdp.incomingPending = true;
    fallbackRt.update(9000u);
    CHECK(ctx, fallbackRt.status().featureFlagsRequestAttempts ==
               TRACKER_SLIMEVR_FEATURE_FLAGS_REQUEST_ATTEMPTS);
    CHECK(ctx, fallbackRt.status().featureNegotiationState ==
               SlimeVRFeatureNegotiationState::Unavailable);
    CHECK(ctx, fallbackRt.status().motionPacketMode ==
               SlimeVRMotionPacketMode::SeparateRotation17Accel4);
    fallbackSnapshots.snapshot.sequence += 1u;
    fallbackRt.update(9010u);
    CHECK(ctx, fallbackRt.status().accelerationSent == 1u);

}

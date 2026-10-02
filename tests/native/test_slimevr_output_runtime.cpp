#include "test_common.hpp"

#include <Arduino.h>

#include <cerrno>
#include <cstring>
#include <initializer_list>
#include <vector>

#include "build_config/build_identity.hpp"
#include "network/udp_transport.hpp"
#include "network/wifi_manager.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "runtime/tracker_runtime_types.hpp"

using namespace tracker;

#include "cases/slimevr_output/fixtures.hpp"
#include "cases/slimevr_output/session.hpp"
#include "cases/slimevr_output/scheduler.hpp"
#include "cases/slimevr_output/pressure.hpp"
#include "cases/slimevr_output/manual_endpoint.hpp"

int main() {
    TestContext ctx;

    char ip[24];
    CHECK(ctx, std::strcmp(udpIpv4ToCString(0xC0A80073UL, ip, sizeof(ip)), "192.168.0.115") == 0);
    uint32_t parsedIpv4 = 0;
    CHECK(ctx, udpParseIpv4("192.168.0.42", parsedIpv4));
    CHECK(ctx, parsedIpv4 == 0xC0A8002AUL);
    CHECK(ctx, !udpParseIpv4("192.168.0.999", parsedIpv4));
    CHECK(ctx, !udpParseIpv4("192.168.0", parsedIpv4));
    CHECK(ctx, !udpParseIpv4("0.0.0.0", parsedIpv4));
    CHECK(ctx, std::strcmp(slimevrOutputStateName(SlimeVROutputState::Discovering), "discovering") == 0);

    FakeWifiAdapter wifiAdapter;
    wifiAdapter.current.linkStatus = WifiLinkStatus::Connected;
    wifiAdapter.current.ipv4 = 0xC0A80073UL;
    wifiAdapter.current.mac[0] = 0xE8;
    wifiAdapter.current.rssiDbm = -68;
    TrackerWifiManager wifi;
    wifi.begin(wifiAdapter);
    wifi.configure(wifiConfig());
    wifi.update(0);
    wifiAdapter.current.linkStatus = WifiLinkStatus::Connected;
    wifiAdapter.current.ipv4 = 0xC0A80073UL;
    wifiAdapter.current.mac[0] = 0xE8;
    wifi.update(20);
    CHECK(ctx, wifi.connected());

    FakeUdp udp;
    FakeSnapshotSource snapshots;
    snapshots.snapshot.valid = true;
    snapshots.snapshot.sequence = 1;
    snapshots.snapshot.runtimeSample = 123;
    snapshots.snapshot.timestampUs = 456789;
    snapshots.snapshot.publishedAtMcuUs = 1200000u;
    snapshots.snapshot.q = Quat::identity();
    snapshots.snapshot.linearAccelerationValid = true;
    snapshots.snapshot.linearAccelerationDeviceG = Vec3(0.25f, -0.5f, 1.75f);
    snapshots.snapshot.qualityFlags = 0x1234;
    snapshots.snapshot.confidence = 0.99f;

    SlimeVROutputRuntime rt;
    FakeConfigFlagSink configFlagSink;
    rt.begin(udp, wifi, FakeSnapshotSource::copy, &snapshots);

    SlimeVROutputRuntimeConfig cfg;
    cfg.enabled = true;
    cfg.deviceName = "unit-test";
    cfg.sensorId = 2;
    cfg.serverPort = 6969;
    cfg.localPort = 6969;
    cfg.discoveryIntervalMs = 1000;
    cfg.rotationRateHz = 100;
    // Preserve the historical bundle-capable baseline for the broad protocol
    // regression. Product defaults are independently tested as quaternion-only.
    cfg.motionPacketPolicy = SlimeVRMotionPacketPolicy::BundleRotation17Acceleration4;
    cfg.magSupportEnabled = true;
    cfg.magEnabled = true;
    cfg.latestTemperatureValid = true;
    cfg.latestTemperatureC = 42.5f;
    cfg.batteryTelemetryEnabled = true;
    cfg.latestBatteryValid = true;
    cfg.latestBatteryVoltage = 3.80f;
    cfg.latestBatteryPercentage = 55.0f;
    // Keep this protocol test focused on payload/cadence behavior rather than
    // inheriting the profile's production telemetry intervals.
    cfg.signalTelemetryIntervalMs = 5000u;
    cfg.temperatureTelemetryIntervalMs = 5000u;
    cfg.batteryTelemetryIntervalMs = 5000u;
    cfg.setConfigFlag = FakeConfigFlagSink::apply;
    cfg.setConfigFlagUser = &configFlagSink;

    testSessionLifecycle(ctx, wifi, udp, snapshots, rt, configFlagSink, cfg);
    const auto phaseCfg = testScheduler(ctx, wifi, cfg);
    testNegotiationFallback(ctx, wifi, phaseCfg);
    testPressureRecovery(ctx, wifi, phaseCfg);
    testManualEndpointAndPolicy(ctx, wifi, phaseCfg);

    return ctx.finish("slimevr_output_runtime");
}

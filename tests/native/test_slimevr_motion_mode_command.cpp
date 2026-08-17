#include "test_common.hpp"

#include <cstdio>
#include <string>

#include <Arduino.h>
#include <Preferences.h>

#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "runtime/slimevr_output_runtime.hpp"
#include "serial/tracker_slimevr_commands.hpp"
#include "../../src/serial/tracker_slimevr_commands.cpp"

Stream Serial;

using namespace tracker;

namespace {

class CaptureStream final : public Stream {
public:
    std::string output;

    size_t write(uint8_t value) override {
        output.push_back(static_cast<char>(value));
        return 1u;
    }

    size_t write(const uint8_t* data, size_t len) override {
        if (data != nullptr) output.append(reinterpret_cast<const char*>(data), len);
        return data != nullptr ? len : 0u;
    }
};

struct MotionControlDeps {
    TrackerConfig* active = nullptr;
    TrackerConfigStore* store = nullptr;
    SlimeVROutputRuntime* runtime = nullptr;
};

bool setMotionPolicy(SlimeVRMotionPacketPolicy policy, void* user) {
    auto* deps = static_cast<MotionControlDeps*>(user);
    if (!deps || !deps->active || !deps->store || !deps->runtime) return false;
    if (!deps->store->saveSlimeVRMotionPacketPolicy(
            *deps->active, policy, TrackerCalibrationProvenance::Manual)) {
        return false;
    }
    deps->runtime->setMotionPacketPolicy(policy);
    return true;
}

void dispatchMotionMode(TrackerSerialCommandContext& context, const char* mode) {
    char command[] = "slime";
    char action[] = "motion-mode";
    char value[16] = {};
    std::snprintf(value, sizeof(value), "%s", mode);
    char* argv[] = {command, action, value};
    trackerSerialDispatchSlimeVRCommand(context, 3, argv);
}

} // namespace

int main() {
    TestContext ctx;
    Preferences::clearTestStorage();

    CaptureStream output;
    TrackerConfig config;
    config.resetDefaults();
    CHECK(ctx, config.slimevrMotionPacketPolicy() == SlimeVRMotionPacketPolicy::QuaternionOnly);

    TrackerConfigStore store("motion_mode", "cfg");
    SlimeVROutputRuntime runtime;
    CHECK(ctx, runtime.status().motionPacketPolicy == SlimeVRMotionPacketPolicy::QuaternionOnly);

    MotionControlDeps control{&config, &store, &runtime};
    TrackerSerialCommandContext context;
    context.io = &output;
    context.config = &config;
    context.configStore = &store;
    context.slimevrRuntime = &runtime;
    context.setSlimeVRMotionPacketPolicy = setMotionPolicy;
    context.setSlimeVRMotionPacketPolicyUser = &control;

    // With no authoritative config yet, the command persists a clean default
    // baseline plus the selected policy rather than arbitrary RAM-only edits.
    config.data.output.outputRateHz = 77u;
    config.updateCrc();
    dispatchMotionMode(context, "bundle");
    CHECK(ctx, config.slimevrMotionPacketPolicy() ==
               SlimeVRMotionPacketPolicy::BundleRotation17Acceleration4);
    CHECK(ctx, config.data.output.outputRateHz == 77u);
    CHECK(ctx, runtime.status().motionPacketPolicy ==
               SlimeVRMotionPacketPolicy::BundleRotation17Acceleration4);

    TrackerConfig reloaded;
    bool loaded = false;
    CHECK(ctx, store.loadOrDefaults(reloaded, &loaded));
    CHECK(ctx, loaded);
    CHECK(ctx, reloaded.slimevrMotionPacketPolicy() ==
               SlimeVRMotionPacketPolicy::BundleRotation17Acceleration4);
    CHECK(ctx, reloaded.data.output.outputRateHz == tracker::cfg::OUTPUT_RATE_HZ);
    store.confirmAuthoritativeConfigApplied();

    // Once an authoritative generation exists, persist only motion policy.
    // An unsaved output-rate edit must remain RAM-only and must not hitchhike
    // into NVS when the policy changes.
    config.data.output.outputRateHz = 50u;
    config.data.output.serialDebugEnabled = false;
    config.updateCrc();
    dispatchMotionMode(context, "packet23");
    CHECK(ctx, config.slimevrMotionPacketPolicy() ==
               SlimeVRMotionPacketPolicy::RotationAcceleration23);
    CHECK(ctx, config.data.output.outputRateHz == 50u);
    CHECK(ctx, !config.data.output.serialDebugEnabled);
    CHECK(ctx, runtime.status().motionPacketPolicy ==
               SlimeVRMotionPacketPolicy::RotationAcceleration23);

    CHECK(ctx, store.load(reloaded));
    CHECK(ctx, reloaded.slimevrMotionPacketPolicy() ==
               SlimeVRMotionPacketPolicy::RotationAcceleration23);
    CHECK(ctx, reloaded.data.output.outputRateHz == tracker::cfg::OUTPUT_RATE_HZ);
    CHECK(ctx, reloaded.data.output.serialDebugEnabled);
    store.confirmAuthoritativeConfigApplied();

    dispatchMotionMode(context, "quaternion");
    CHECK(ctx, config.slimevrMotionPacketPolicy() == SlimeVRMotionPacketPolicy::QuaternionOnly);
    CHECK(ctx, runtime.status().motionPacketPolicy == SlimeVRMotionPacketPolicy::QuaternionOnly);
    CHECK(ctx, store.load(reloaded));
    CHECK(ctx, reloaded.slimevrMotionPacketPolicy() == SlimeVRMotionPacketPolicy::QuaternionOnly);
    store.confirmAuthoritativeConfigApplied();

    const TrackerConfig beforeFailedSave = config;
    const SlimeVRMotionPacketPolicy runtimeBeforeFailure = runtime.status().motionPacketPolicy;
    Preferences::setNextBeginFailures(1u);
    dispatchMotionMode(context, "bundle");
    CHECK(ctx, config.data.output.packetFormat == beforeFailedSave.data.output.packetFormat);
    CHECK(ctx, runtime.status().motionPacketPolicy == runtimeBeforeFailure);
    CHECK(ctx, output.output.find("motion mode save/apply failed") != std::string::npos);

    const TrackerConfig beforeInvalid = config;
    dispatchMotionMode(context, "invalid");
    CHECK(ctx, config.data.output.packetFormat == beforeInvalid.data.output.packetFormat);
    CHECK(ctx, output.output.find("invalid motion mode") != std::string::npos);

    // Handler must not silently persist/apply when the domain hook is absent.
    TrackerSerialCommandContext incomplete = context;
    incomplete.setSlimeVRMotionPacketPolicy = nullptr;
    incomplete.setSlimeVRMotionPacketPolicyUser = nullptr;
    const TrackerConfig beforeUnavailable = config;
    dispatchMotionMode(incomplete, "packet23");
    CHECK(ctx, config.data.output.packetFormat == beforeUnavailable.data.output.packetFormat);
    CHECK(ctx, output.output.find("motion-mode control unavailable") != std::string::npos);

    return ctx.finish("slimevr_motion_mode_command");
}

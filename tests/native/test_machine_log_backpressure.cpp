#include "test_common.hpp"

#include <cstdint>
#include <string>

#include "runtime/machine_log_runtime.hpp"

using namespace tracker;

class AdmissionStream final : public Stream {
public:
    int writable = 0;
    std::string output;

    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    int availableForWrite() override { return writable; }
    size_t write(uint8_t value) override {
        output.push_back(static_cast<char>(value));
        return 1u;
    }
    size_t write(const uint8_t* data, size_t len) override {
        if (data != nullptr) {
            output.append(reinterpret_cast<const char*>(data), len);
        }
        return data != nullptr ? len : 0u;
    }
};

static size_t lineCount(const std::string& text) {
    size_t count = 0u;
    for (char value : text) {
        if (value == '\n') ++count;
    }
    return count;
}

int main() {
    TestContext ctx;
    AdmissionStream out;
    TrackerSerialLogState state;
    state.mode = TrackerLogMode::Basic;
    state.bind(out, TrackerCommandOrigin::RemoteTcp, 7u);
    MachineLogCounters counters;
    uint32_t lastBiasEmitUs = 0u;
    MachineLogDeferredRuntime runtime;
    runtime.begin(&state, &counters, &lastBiasEmitUs, 1000000u);

    CHECK(ctx, sizeof(MachineLogDeferredRecord) <= 224u);
    CHECK(ctx, sizeof(MachineLogDeferredRuntime) <=
                   TRACKER_MACHINE_LOG_QUEUE_RECORDS * 224u + 128u);

    trackerTestSetMicros(100u);
    CHECK(ctx, runtime.enqueueState("TRACKING", "test,bad\nreason", 123u, 0u, 1.0f));
    CHECK(ctx, out.output.empty());
    CHECK(ctx, counters.state == 0u);
    CHECK(ctx, counters.backpressureDrop == 0u);
    CHECK(ctx, state.sequence == 1u);
    CHECK(ctx, runtime.status().queued == 1u);

    out.writable = 64;
    CHECK(ctx, !runtime.service());
    CHECK(ctx, out.output.empty());
    CHECK(ctx, counters.serviceDeferral == 1u);
    CHECK(ctx, runtime.status().queued == 1u);

    trackerTestAdvanceMicros(75u);
    out.writable = 1024;
    CHECK(ctx, runtime.service());
    CHECK(ctx, !out.output.empty());
    CHECK(ctx, out.output == "STATE,123,0,TRACKING,test_bad_reason,0x0,1.0000\n");
    CHECK(ctx, counters.state == 1u);
    CHECK(ctx, state.sequence == 1u);
    CHECK(ctx, runtime.status().queued == 0u);
    CHECK(ctx, runtime.status().serialized == 1u);
    CHECK(ctx, runtime.status().maxRecordAgeUs == 75u);

    runtime.reset(false);
    counters = MachineLogCounters{};
    state.sequence = 0u;
    state.mode = TrackerLogMode::Full;
    state.finishing = false;
    out.output.clear();
    MagProcessedSample mag;
    mag.valid = true;
    mag.t_us = 500u;
    mag.seq = 9u;
    MagHeadingSample heading;
    MagFieldReliabilityOutput reliability;
    MagYawCorrectionOutput yaw;
    CHECK(ctx, runtime.enqueueMag(mag, heading, reliability, yaw, 0u, false));
    CHECK(ctx, runtime.service());
    CHECK(ctx, lineCount(out.output) == 1u);
    CHECK(ctx, runtime.status().queued == 1u);
    CHECK(ctx, runtime.status().serialized == 0u);
    CHECK(ctx, runtime.service());
    CHECK(ctx, lineCount(out.output) == 2u);
    CHECK(ctx, runtime.status().queued == 1u);
    CHECK(ctx, runtime.status().serialized == 0u);
    CHECK(ctx, runtime.service());
    CHECK(ctx, lineCount(out.output) == 3u);
    CHECK(ctx, runtime.status().queued == 0u);
    CHECK(ctx, runtime.status().serialized == 1u);

    runtime.reset(false);
    counters = MachineLogCounters{};
    state.sequence = 0u;
    state.lastNetworkEmitUs = 0u;
    out.output.clear();
    TrackerWifiManager wifi;
    SlimeVROutputRuntime slime;
    CHECK(ctx, runtime.enqueueNetwork(1000000u, wifi, slime));
    CHECK(ctx, runtime.service());
    CHECK(ctx, out.output.rfind("NET,1000000,0,", 0u) == 0u);
    CHECK(ctx, counters.network == 1u);
    CHECK(ctx, runtime.status().serialized == 1u);

    runtime.reset(false);
    counters = MachineLogCounters{};
    state.sequence = 0u;
    for (uint32_t i = 0u; i < TRACKER_MACHINE_LOG_QUEUE_RECORDS; ++i) {
        CHECK(ctx, runtime.enqueueState("TRACKING", "fill", 200u + i, 0u, 1.0f));
    }
    CHECK(ctx, runtime.status().queued == TRACKER_MACHINE_LOG_QUEUE_RECORDS);
    CHECK(ctx, runtime.status().highWater == TRACKER_MACHINE_LOG_QUEUE_RECORDS);
    CHECK(ctx, !runtime.enqueueState("TRACKING", "overflow", 999u, 0u, 1.0f));
    CHECK(ctx, counters.producerQueueDrop == 1u);
    CHECK(ctx, counters.backpressureDrop == 1u);
    CHECK(ctx, state.sequence == TRACKER_MACHINE_LOG_QUEUE_RECORDS);

    runtime.reset(true);
    CHECK(ctx, runtime.status().queued == 0u);
    CHECK(ctx, counters.shutdownDrop == TRACKER_MACHINE_LOG_QUEUE_RECORDS);
    CHECK(ctx, counters.backpressureDrop == TRACKER_MACHINE_LOG_QUEUE_RECORDS + 1u);

    return ctx.finish("machine_log_backpressure");
}

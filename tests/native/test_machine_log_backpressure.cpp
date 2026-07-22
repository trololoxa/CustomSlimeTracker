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
};

int main() {
    TestContext ctx;
    AdmissionStream out;
    TrackerSerialLogState state;
    state.mode = TrackerLogMode::Basic;
    MachineLogCounters counters;

    out.writable = 64;
    machineLogEmitStateEvent(out, state, counters, "TRACKING", "test", 123u, 0u, 1.0f);
    CHECK(ctx, out.output.empty());
    CHECK(ctx, counters.state == 0u);
    CHECK(ctx, counters.backpressureDrop == 1u);
    CHECK(ctx, state.sequence == 0u);

    out.writable = 1024;
    machineLogEmitStateEvent(out, state, counters, "TRACKING", "test", 124u, 0u, 1.0f);
    CHECK(ctx, !out.output.empty());
    CHECK(ctx, counters.state == 1u);
    CHECK(ctx, counters.backpressureDrop == 1u);
    CHECK(ctx, state.sequence == 1u);

    return ctx.finish("machine_log_backpressure");
}

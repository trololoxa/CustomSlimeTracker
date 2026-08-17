#include "test_common.hpp"

#include <string>

#include <Arduino.h>
Stream Serial;
#include "../../src/serial/tracker_output_commands.cpp"
#include "../../src/serial/tracker_test_commands.cpp"

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

Stream* stoppedThrough = nullptr;
bool stoppedForce = false;
uint32_t stopCalls = 0u;
uint32_t startedDurationMs = 0u;
uint32_t summaryCalls = 0u;

bool stopTest(Stream& out, bool force, void*) {
    stoppedThrough = &out;
    stoppedForce = force;
    ++stopCalls;
    return true;
}

bool startStaticTest(uint32_t durationMs, Stream&, void*) {
    startedDurationMs = durationMs;
    return true;
}

bool printStaticSummary(Stream& out, void*) {
    ++summaryCalls;
    out.println("TESTSUM,static,test");
    return true;
}

void dispatchStop(TrackerSerialCommandContext& context) {
    char command[] = "test";
    char action[] = "stop";
    char* argv[] = {command, action};
    trackerSerialDispatchTestCommand(context, 2, argv);
}

void dispatch(TrackerSerialCommandContext& context,
              void (*handler)(TrackerSerialCommandContext&, int, char**),
              const char* first,
              const char* second,
              const char* third) {
    char a[16] = {};
    char b[16] = {};
    char c[16] = {};
    std::snprintf(a, sizeof(a), "%s", first);
    std::snprintf(b, sizeof(b), "%s", second);
    std::snprintf(c, sizeof(c), "%s", third);
    char* argv[] = {a, b, c};
    handler(context, 3, argv);
}

} // namespace

int main() {
    TestContext ctx;
    CaptureStream remote;
    CaptureStream usb;

    TrackerSerialCommandContext context;
    context.io = &remote;
    context.origin = TrackerCommandOrigin::RemoteTcp;
    context.stopStaticTest = stopTest;
    dispatchStop(context);
    CHECK(ctx, stoppedThrough == &remote);
    CHECK(ctx, stoppedForce);
    CHECK(ctx, stopCalls == 1u);

    context.io = &usb;
    context.origin = TrackerCommandOrigin::UsbSerial;
    dispatchStop(context);
    CHECK(ctx, stoppedThrough == &usb);
    CHECK(ctx, stoppedForce);
    CHECK(ctx, stopCalls == 2u);

    TrackerSerialLogState logState;
    context.logState = &logState;
    context.origin = TrackerCommandOrigin::RemoteTcp;
    dispatch(context, trackerSerialDispatchLogCommand, "log", "rate", "200");
    CHECK(ctx, logState.rateHz == 200u);
    dispatch(context, trackerSerialDispatchLogCommand, "log", "rate", "201");
    CHECK(ctx, logState.rateHz == 200u);

    context.startStaticTest = startStaticTest;
    dispatch(context, trackerSerialDispatchTestCommand, "test", "static", "21600");
    CHECK(ctx, startedDurationMs == 21600000u);
    dispatch(context, trackerSerialDispatchTestCommand, "test", "static", "21601");
    CHECK(ctx, startedDurationMs == 21600000u);

    context.printStaticTestSummary = printStaticSummary;
    dispatch(context, trackerSerialDispatchTestCommand, "test", "summary", "static");
    CHECK(ctx, summaryCalls == 1u);
    CHECK(ctx, usb.output.find("TESTSUM,static,test") != std::string::npos);

    return ctx.finish("cli_transport_parity");
}

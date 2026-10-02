#include "test_common.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "serial/bounded_duplex_stream.hpp"
// Exercise the real report without adding its hardware wiring to every test.
#include "../../src/runtime/runtime_status_reporter.cpp"

using namespace tracker;

class FakeDuplexTransport final : public Stream {
public:
    std::vector<uint8_t> input;
    std::vector<uint8_t> output;
    size_t readPos = 0u;
    int writable = 1024;

    int available() override { return static_cast<int>(input.size() - readPos); }
    int read() override { return readPos < input.size() ? input[readPos++] : -1; }
    int peek() override { return readPos < input.size() ? input[readPos] : -1; }
    int availableForWrite() override { return writable; }
    size_t write(uint8_t value) override { return write(&value, 1u); }
    size_t write(const uint8_t* data, size_t len) override {
        if (!data || len == 0u || writable <= 0) return 0u;
        const size_t accepted = std::min(len, static_cast<size_t>(writable));
        output.insert(output.end(), data, data + accepted);
        return accepted;
    }
};

// No sensor I/O is permitted while rendering a status snapshot.
class StatusOnlyBus final : public Lsm6dsvTransport {
public:
    bool touched = false;
    bool read(uint8_t, uint8_t*, size_t) override { touched = true; return false; }
    bool write(uint8_t, const uint8_t*, size_t) override { touched = true; return false; }
    void delayMs(uint32_t) override { touched = true; }
};

static void testDiagnosticBurst(TestContext& ctx) {
    StatusOnlyBus bus;
    Lsm6dsv imu(bus);
    Lsm6dsvFifoReader fifo(bus, imu);
    FifoRuntimeProcessor runtime;
    SensorProgressWatchdog progress;
    SensorRecoveryController recovery;
    TrackingStateController tracking;
    TrackerConfig config;
    Ahrs6Dof ahrs;
    ImuQualityMonitor quality;
    MagRuntimeProcessor mag;
    MagProcessedSample field;
    MagHeadingEstimator heading;
    MagHeadingSample headingSample;
    MagHeadingReferenceState reference;
    MagHeadingAutoReferenceState autoReference;
    MagYawCorrectionOutput yaw;
    RuntimeStatusReporterDeps deps;
    deps.config = &config; deps.ahrs = &ahrs; deps.quality = &quality;
    deps.fifo = &fifo; deps.fifoRuntime = &runtime;
    deps.sensorProgress = &progress; deps.sensorRecovery = &recovery;
    deps.trackingState = &tracking; deps.magProcessor = &mag;
    deps.lastMagProcessed = &field; deps.magHeading = &heading;
    deps.lastMagHeading = &headingSample; deps.magHeadingRef = &reference;
    deps.magHeadingAutoRef = &autoReference; deps.lastMagYawCorrection = &yaw;
    deps.runtimeSamples = UINT32_MAX; deps.fifoIntCount = UINT32_MAX;
    deps.trackingStateName = "DEGRADED_MAG";
    FakeDuplexTransport sink;
    runtimeStatusPrintHealth(sink, deps);
    // The native Arduino stub emits LF. Model target CRLF explicitly and leave
    // additional room for wider runtime counters, valid pose age and boot text.
    std::string burst = "# HEALTH\r\n";
    for (uint8_t ch : sink.output) {
        if (ch == '\n') burst += '\r';
        burst += static_cast<char>(ch);
    }
    CHECK(ctx, !bus.touched);
    CHECK(ctx, burst.size() + 1024u < 8192u);
    CHECK(ctx, burst.find("mean_dt_us=") != std::string::npos);
    BoundedDuplexStream<8192, 512> large;
    BoundedDuplexStream<1536, 512> normal;
    FakeDuplexTransport output;
    output.writable = 0;
    large.begin(output); normal.begin(output);
    large.print(burst.c_str()); normal.print(burst.c_str());
    CHECK(ctx, normal.status().recordsDropped > 0u);
    CHECK(ctx, large.status().recordsDropped == 0u);
    CHECK(ctx, large.drain(48u) == 0u);
    CHECK(ctx, large.status().queuedBytes == burst.size());
    output.writable = 1024;
    // Capacity never expands a single drain's existing production byte budget.
    for (size_t i = 0; i < 8192u / 48u + 1u && large.hasPending(); ++i) {
        CHECK(ctx, large.drain(48u) <= 48u);
    }
    CHECK(ctx, !large.hasPending());
    CHECK(ctx, std::string(output.output.begin(), output.output.end()) == burst);
    std::cout << "health_crlf_bytes=" << burst.size() << " queue=8192 drain_budget=48\n";
}

int main() {
    TestContext ctx;
    testDiagnosticBurst(ctx);

    FakeDuplexTransport transport;
    transport.input = {'h', 'i', '\n'};

    BoundedDuplexStream<8, 8> stream;
    stream.begin(transport);
    CHECK(ctx, stream.available() == 3);
    CHECK(ctx, stream.peek() == 'h');
    CHECK(ctx, stream.read() == 'h');
    CHECK(ctx, stream.available() == 2);

    // Output is staged until a complete line is available.
    CHECK(ctx, stream.print("abc") == 3u);
    CHECK(ctx, !stream.hasPending());
    CHECK(ctx, stream.status().stagedBytes == 3u);
    CHECK(ctx, stream.println("de") == 3u);
    auto status = stream.status();
    CHECK(ctx, status.queuedBytes == 6u);
    CHECK(ctx, status.stagedBytes == 0u);
    CHECK(ctx, status.recordsQueued == 1u);
    CHECK(ctx, status.bytesDropped == 0u);

    transport.writable = 3;
    CHECK(ctx, stream.drain(3u) == 3u);
    CHECK(ctx, std::string(transport.output.begin(), transport.output.end()) == "abc");
    CHECK(ctx, stream.status().queuedBytes == 3u);

    transport.writable = 0;
    CHECK(ctx, stream.drain(8u) == 0u);
    CHECK(ctx, stream.status().drainStalls == 1u);

    // An oversized record is discarded as one unit. It cannot overwrite or
    // splice the already queued suffix of the previous line.
    CHECK(ctx, stream.print("0123456789\n") == 11u);
    status = stream.status();
    CHECK(ctx, status.queuedBytes == 3u);
    CHECK(ctx, status.bytesDropped == 11u);
    CHECK(ctx, status.recordsDropped == 1u);
    CHECK(ctx, status.oversizedRecordsDropped == 1u);

    transport.writable = 1024;
    CHECK(ctx, stream.drain(32u) == 3u);
    CHECK(ctx, std::string(transport.output.begin(), transport.output.end()) == "abcde\n");

    // Empty drains are a hot-path no-op.
    const uint32_t drainCallsBeforeEmpty = stream.status().drainCalls;
    CHECK(ctx, stream.drain(8u) == 0u);
    CHECK(ctx, stream.status().drainCalls == drainCallsBeforeEmpty);

    // Complete records still use the memcpy-based wrapping ring.
    BoundedDuplexStream<16, 8> wrapStream;
    FakeDuplexTransport wrapTransport;
    wrapStream.begin(wrapTransport);
    CHECK(ctx, wrapStream.print("abcde\n") == 6u);
    CHECK(ctx, wrapStream.print("12345\n") == 6u);
    CHECK(ctx, wrapStream.drain(7u) == 7u);
    CHECK(ctx, wrapStream.print("XYZ\n") == 4u);
    std::string wrapOutput;
    CHECK(ctx, wrapStream.drainWith(32u, [&wrapOutput](const uint8_t* data, size_t len) {
        wrapOutput.append(reinterpret_cast<const char*>(data), len);
        return len;
    }) == 9u);
    CHECK(ctx, wrapOutput == "2345\nXYZ\n");

    // A caller-supplied writer supports WiFiClient MSG_DONTWAIT and partial
    // commits while preserving the remaining queue bytes.
    BoundedDuplexStream<16, 8> socketStream;
    FakeDuplexTransport socketTransport;
    socketStream.begin(socketTransport);
    CHECK(ctx, socketStream.print("WIFI\n") == 5u);
    std::string socketOutput;
    CHECK(ctx, socketStream.drainWith(3u, [&socketOutput](const uint8_t* data, size_t len) {
        const size_t accepted = std::min<size_t>(2u, len);
        socketOutput.append(reinterpret_cast<const char*>(data), accepted);
        return accepted;
    }) == 2u);
    CHECK(ctx, socketOutput == "WI");
    CHECK(ctx, socketStream.status().queuedBytes == 3u);
    CHECK(ctx, socketStream.drainWith(8u, [&socketOutput](const uint8_t* data, size_t len) {
        socketOutput.append(reinterpret_cast<const char*>(data), len);
        return len;
    }) == 3u);
    CHECK(ctx, socketOutput == "WIFI\n");

    // flush() commits one unterminated short record before invoking the bounded
    // transport handler.
    struct FlushProbe { int calls = 0; } flushProbe;
    socketStream.setFlushHandler([](void* user) {
        auto* probe = static_cast<FlushProbe*>(user);
        if (probe) ++probe->calls;
    }, &flushProbe);
    socketStream.print("tail");
    CHECK(ctx, socketStream.status().stagedBytes == 4u);
    socketStream.flush();
    CHECK(ctx, flushProbe.calls == 1);
    CHECK(ctx, socketStream.status().stagedBytes == 0u);
    CHECK(ctx, socketStream.status().queuedBytes == 4u);

    // Queue pressure discards a complete line and later emits one complete
    // warning record. No prefix or suffix of the rejected line is delivered.
    BoundedDuplexStream<64, 32> burstStream;
    FakeDuplexTransport burstTransport;
    burstStream.begin(burstTransport);
    CHECK(ctx, burstStream.print("first-record-0000\n") == 18u);
    CHECK(ctx, burstStream.print("second-record-000\n") == 18u);
    CHECK(ctx, burstStream.print("this-whole-record-is-dropped\n") == 29u);
    status = burstStream.status();
    CHECK(ctx, status.recordsDropped == 1u);
    CHECK(ctx, status.queuedBytes == 36u);
    CHECK(ctx, status.pendingDropNoticeRecords == 1u);
    CHECK(ctx, burstStream.drain(64u) == 36u);
    CHECK(ctx, burstStream.drain(64u) > 0u);
    const std::string burstOutput(burstTransport.output.begin(), burstTransport.output.end());
    CHECK(ctx, burstOutput.find("first-record-0000\nsecond-record-000\n") == 0u);
    CHECK(ctx, burstOutput.find("this-whole-record-is-dropped") == std::string::npos);
    CHECK(ctx, burstOutput.find("# WARN console dropped 1 complete line(s)\n") != std::string::npos);

    // A full reset discards stale queued and staged bytes and establishes a
    // clean baseline for the next console status report.
    burstStream.print("staged-without-newline");
    burstStream.resetOutputState();
    status = burstStream.status();
    CHECK(ctx, status.queuedBytes == 0u);
    CHECK(ctx, status.stagedBytes == 0u);
    CHECK(ctx, status.bytesQueued == 0u);
    CHECK(ctx, status.bytesDrained == 0u);
    CHECK(ctx, status.bytesDropped == 0u);
    CHECK(ctx, status.recordsDropped == 0u);
    CHECK(ctx, status.highWaterBytes == 0u);

    return ctx.finish("bounded_duplex_stream");
}

#include "test_common.hpp"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "serial/bounded_duplex_stream.hpp"

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

int main() {
    TestContext ctx;

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

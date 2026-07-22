#pragma once

#include <Arduino.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace tracker {

struct BoundedDuplexStreamStatus {
    size_t capacityBytes = 0;
    size_t recordCapacityBytes = 0;
    size_t queuedBytes = 0;
    size_t stagedBytes = 0;
    size_t highWaterBytes = 0;
    uint32_t bytesQueued = 0;
    uint32_t bytesDrained = 0;
    uint32_t bytesDropped = 0;
    uint32_t recordsQueued = 0;
    uint32_t recordsDropped = 0;
    uint32_t oversizedRecordsDropped = 0;
    uint32_t dropNoticesQueued = 0;
    uint32_t pendingDropNoticeRecords = 0;
    uint32_t enqueueCalls = 0;
    uint32_t drainCalls = 0;
    uint32_t drainStalls = 0;
};

// Duplex Stream adapter used by the CLI. Input is read directly from the
// transport. Output is first assembled into complete newline-terminated
// records, then copied atomically into a bounded ring and drained later under
// an explicit byte budget. A congested console can therefore drop a complete
// line, but can never splice two unrelated command responses together.
//
// Record staging and queue storage are fixed-size arrays: there is no heap
// allocation and no shared storage with the IMU/FIFO pipeline.
template <size_t Capacity, size_t RecordCapacity>
class BoundedDuplexStream final : public Stream {
    static_assert(Capacity > 0, "BoundedDuplexStream capacity must be non-zero");
    static_assert(RecordCapacity > 0, "BoundedDuplexStream record capacity must be non-zero");
    static_assert(RecordCapacity <= Capacity,
                  "A complete output record must fit in the bounded queue");

public:
    using FlushHandler = void (*)(void* user);

    void begin(Stream& transport) {
        transport_ = &transport;
        clearPending(false);
    }

    void detach(bool countPendingAsDropped = true) {
        clearPending(countPendingAsDropped);
        transport_ = nullptr;
    }

    bool attached() const { return transport_ != nullptr; }
    bool hasPending() const { return size_ != 0u || pendingDropNoticeRecords_ != 0u; }
    size_t queuedBytes() const { return size_; }
    size_t stagedBytes() const { return recordSize_; }
    size_t freeBytes() const { return Capacity - size_; }

    // Used by callers that already know the complete record length, such as
    // the remote diagnostic-line helper. The current staged record is included
    // so the result remains conservative.
    bool canAccept(size_t len) const {
        return !droppingRecord_ &&
               recordSize_ + len <= RecordCapacity &&
               recordSize_ + len <= freeBytes();
    }

    // Records an externally rejected complete record without first formatting
    // it into this stream.
    void recordDroppedRecord(size_t len, bool oversized = false) {
        bytesDropped_ += static_cast<uint32_t>(len);
        ++recordsDropped_;
        if (oversized) ++oversizedRecordsDropped_;
        ++pendingDropNoticeRecords_;
    }

    void setFlushHandler(FlushHandler handler, void* user = nullptr) {
        flushHandler_ = handler;
        flushHandlerUser_ = user;
    }

    int available() override {
        return transport_ ? transport_->available() : 0;
    }

    int read() override {
        return transport_ ? transport_->read() : -1;
    }

    int peek() override {
        return transport_ ? transport_->peek() : -1;
    }

    // This value is intentionally the free queue capacity rather than the
    // transport capacity. High-rate producers use it as an admission gate to
    // avoid expensive float-to-text formatting when the console is congested.
    int availableForWrite() override {
        const size_t free = Capacity - size_;
        return free > static_cast<size_t>(0x7fffffff) ? 0x7fffffff : static_cast<int>(free);
    }

    size_t write(uint8_t value) override {
        return write(&value, 1u);
    }

    size_t write(const uint8_t* data, size_t len) override {
        if (!data || len == 0u) return 0u;
        ++enqueueCalls_;

        size_t offset = 0u;
        while (offset < len) {
            const uint8_t* begin = data + offset;
            const size_t remaining = len - offset;
            const auto* newline = static_cast<const uint8_t*>(std::memchr(begin, '\n', remaining));
            const size_t segmentLen = newline
                ? static_cast<size_t>(newline - begin) + 1u
                : remaining;

            if (droppingRecord_) {
                droppedRecordBytes_ += segmentLen;
                if (newline) finalizeDroppedRecord(true);
                offset += segmentLen;
                continue;
            }

            if (recordSize_ + segmentLen > RecordCapacity) {
                droppedRecordBytes_ = recordSize_ + segmentLen;
                recordSize_ = 0u;
                droppingRecord_ = true;
                if (newline) finalizeDroppedRecord(true);
                offset += segmentLen;
                continue;
            }

            std::memcpy(recordBuffer_ + recordSize_, begin, segmentLen);
            recordSize_ += segmentLen;
            offset += segmentLen;

            if (newline) commitStagedRecord();
        }

        // Print callers should finish formatting the logical record even when
        // the bounded queue rejected it. The rejection is reported through
        // counters and a later whole-line warning, never through a short write
        // that could make the caller emit a malformed suffix.
        return len;
    }

    // Arduino Print::flush() is frequently used before reboot/reset actions.
    // Commit an unterminated final record, then permit one bounded transport
    // drain attempt through the configured handler.
    void flush() override {
        finalizePartialRecord();
        if (flushHandler_) {
            flushHandler_(flushHandlerUser_);
            return;
        }
        (void)drain(Capacity);
    }

    size_t drain(size_t byteBudget) {
        if (!transport_ || byteBudget == 0u) return 0u;
        return drainWith(byteBudget, [this](const uint8_t* data, size_t len) -> size_t {
            const int writable = transport_->availableForWrite();
            if (writable <= 0) return 0u;
            size_t chunk = len;
            if (chunk > static_cast<size_t>(writable)) chunk = static_cast<size_t>(writable);
            return chunk > 0u ? transport_->write(data, chunk) : 0u;
        });
    }

    // Drains through a caller-provided non-blocking writer. The writer must
    // return immediately and report the number of committed bytes. This is
    // used by WiFiClient because Arduino-ESP32 2.0.x inherits the default
    // Print::availableForWrite()==0 even though the underlying socket can be
    // written safely with MSG_DONTWAIT.
    template <typename Writer>
    size_t drainWith(size_t byteBudget, Writer&& writer) {
        if (byteBudget == 0u) return 0u;
        tryQueueDropNotice(0u);
        if (size_ == 0u) return 0u;

        ++drainCalls_;
        size_t total = 0u;
        while (total < byteBudget && size_ > 0u) {
            const size_t contiguous = head_ < tail_ || size_ == 0u
                ? (tail_ - head_)
                : (Capacity - head_);
            size_t chunk = contiguous;
            const size_t remainingBudget = byteBudget - total;
            if (chunk > remainingBudget) chunk = remainingBudget;
            if (chunk == 0u) {
                ++drainStalls_;
                break;
            }

            const size_t written = writer(buffer_ + head_, chunk);
            if (written == 0u) {
                ++drainStalls_;
                break;
            }

            const size_t committed = written < chunk ? written : chunk;
            head_ += committed;
            if (head_ >= Capacity) head_ -= Capacity;
            size_ -= committed;
            total += committed;
            bytesDrained_ += static_cast<uint32_t>(committed);

            if (committed < chunk) {
                ++drainStalls_;
                break;
            }
        }

        // A drop notice is appended only after existing output, preserving the
        // order of all records that were successfully admitted.
        tryQueueDropNotice(0u);
        return total;
    }

    size_t discardPending() {
        return clearPending(true);
    }

    // `console reset` uses this stronger operation: stale queued/staged output
    // is discarded without contaminating the freshly reset counters.
    void resetOutputState() {
        clearPending(false);
        resetCounters();
    }

    void resetCounters() {
        highWaterBytes_ = size_;
        bytesQueued_ = 0u;
        bytesDrained_ = 0u;
        bytesDropped_ = 0u;
        recordsQueued_ = 0u;
        recordsDropped_ = 0u;
        oversizedRecordsDropped_ = 0u;
        dropNoticesQueued_ = 0u;
        pendingDropNoticeRecords_ = 0u;
        enqueueCalls_ = 0u;
        drainCalls_ = 0u;
        drainStalls_ = 0u;
    }

    BoundedDuplexStreamStatus status() const {
        BoundedDuplexStreamStatus s;
        s.capacityBytes = Capacity;
        s.recordCapacityBytes = RecordCapacity;
        s.queuedBytes = size_;
        s.stagedBytes = recordSize_;
        s.highWaterBytes = highWaterBytes_;
        s.bytesQueued = bytesQueued_;
        s.bytesDrained = bytesDrained_;
        s.bytesDropped = bytesDropped_;
        s.recordsQueued = recordsQueued_;
        s.recordsDropped = recordsDropped_;
        s.oversizedRecordsDropped = oversizedRecordsDropped_;
        s.dropNoticesQueued = dropNoticesQueued_;
        s.pendingDropNoticeRecords = pendingDropNoticeRecords_;
        s.enqueueCalls = enqueueCalls_;
        s.drainCalls = drainCalls_;
        s.drainStalls = drainStalls_;
        return s;
    }

private:
    void finalizePartialRecord() {
        if (droppingRecord_) {
            finalizeDroppedRecord(true);
        } else if (recordSize_ != 0u) {
            commitStagedRecord();
        }
    }

    void commitStagedRecord() {
        if (recordSize_ == 0u) return;

        // Insert a pending warning only when both warning and current record
        // fit. Current user output has priority over the warning itself.
        tryQueueDropNotice(recordSize_);
        if (recordSize_ > freeBytes()) {
            bytesDropped_ += static_cast<uint32_t>(recordSize_);
            ++recordsDropped_;
            ++pendingDropNoticeRecords_;
            recordSize_ = 0u;
            return;
        }

        enqueueRaw(recordBuffer_, recordSize_);
        ++recordsQueued_;
        recordSize_ = 0u;
    }

    void finalizeDroppedRecord(bool oversized) {
        bytesDropped_ += static_cast<uint32_t>(droppedRecordBytes_);
        ++recordsDropped_;
        if (oversized) ++oversizedRecordsDropped_;
        ++pendingDropNoticeRecords_;
        droppedRecordBytes_ = 0u;
        droppingRecord_ = false;
        recordSize_ = 0u;
    }

    void tryQueueDropNotice(size_t reserveBytes) {
        if (pendingDropNoticeRecords_ == 0u) return;

        char notice[72];
        const int printed = std::snprintf(
            notice,
            sizeof(notice),
            "# WARN console dropped %lu complete line(s)\n",
            static_cast<unsigned long>(pendingDropNoticeRecords_)
        );
        if (printed <= 0) return;
        const size_t noticeLen = static_cast<size_t>(printed);
        if (noticeLen > sizeof(notice) - 1u) return;
        if (noticeLen > Capacity) {
            // Tiny test/minimal configurations may be unable to represent the
            // warning itself. Do not leave such a stream permanently pending.
            pendingDropNoticeRecords_ = 0u;
            return;
        }
        if (noticeLen + reserveBytes > freeBytes()) return;

        enqueueRaw(reinterpret_cast<const uint8_t*>(notice), noticeLen);
        ++dropNoticesQueued_;
        pendingDropNoticeRecords_ = 0u;
    }

    void enqueueRaw(const uint8_t* data, size_t len) {
        if (!data || len == 0u) return;
        const size_t first = len < (Capacity - tail_) ? len : (Capacity - tail_);
        if (first != 0u) std::memcpy(buffer_ + tail_, data, first);
        const size_t second = len - first;
        if (second != 0u) std::memcpy(buffer_, data + first, second);
        tail_ += len;
        if (tail_ >= Capacity) tail_ -= Capacity;
        size_ += len;
        bytesQueued_ += static_cast<uint32_t>(len);
        if (size_ > highWaterBytes_) highWaterBytes_ = size_;
    }

    size_t clearPending(bool countAsDropped) {
        const size_t pending = size_ + recordSize_ + droppedRecordBytes_;
        if (countAsDropped && pending != 0u) {
            bytesDropped_ += static_cast<uint32_t>(pending);
            ++recordsDropped_;
        }
        head_ = 0u;
        tail_ = 0u;
        size_ = 0u;
        recordSize_ = 0u;
        droppedRecordBytes_ = 0u;
        droppingRecord_ = false;
        pendingDropNoticeRecords_ = 0u;
        return pending;
    }

    Stream* transport_ = nullptr;
    FlushHandler flushHandler_ = nullptr;
    void* flushHandlerUser_ = nullptr;
    uint8_t buffer_[Capacity] = {};
    uint8_t recordBuffer_[RecordCapacity] = {};
    size_t head_ = 0u;
    size_t tail_ = 0u;
    size_t size_ = 0u;
    size_t recordSize_ = 0u;
    size_t droppedRecordBytes_ = 0u;
    bool droppingRecord_ = false;
    size_t highWaterBytes_ = 0u;
    uint32_t bytesQueued_ = 0u;
    uint32_t bytesDrained_ = 0u;
    uint32_t bytesDropped_ = 0u;
    uint32_t recordsQueued_ = 0u;
    uint32_t recordsDropped_ = 0u;
    uint32_t oversizedRecordsDropped_ = 0u;
    uint32_t dropNoticesQueued_ = 0u;
    uint32_t pendingDropNoticeRecords_ = 0u;
    uint32_t enqueueCalls_ = 0u;
    uint32_t drainCalls_ = 0u;
    uint32_t drainStalls_ = 0u;
};

inline void printBoundedDuplexStreamStatus(Stream& out,
                                           const char* prefix,
                                           const BoundedDuplexStreamStatus& s) {
    const char* p = prefix ? prefix : "output";
    out.print(p); out.print("_capacity_bytes="); out.println(static_cast<unsigned long>(s.capacityBytes));
    out.print(p); out.print("_record_capacity_bytes="); out.println(static_cast<unsigned long>(s.recordCapacityBytes));
    out.print(p); out.print("_queued_bytes="); out.println(static_cast<unsigned long>(s.queuedBytes));
    out.print(p); out.print("_staged_bytes="); out.println(static_cast<unsigned long>(s.stagedBytes));
    out.print(p); out.print("_high_water_bytes="); out.println(static_cast<unsigned long>(s.highWaterBytes));
    out.print(p); out.print("_bytes_queued="); out.println(s.bytesQueued);
    out.print(p); out.print("_bytes_drained="); out.println(s.bytesDrained);
    out.print(p); out.print("_bytes_dropped="); out.println(s.bytesDropped);
    out.print(p); out.print("_records_queued="); out.println(s.recordsQueued);
    out.print(p); out.print("_records_dropped="); out.println(s.recordsDropped);
    out.print(p); out.print("_oversized_records_dropped="); out.println(s.oversizedRecordsDropped);
    out.print(p); out.print("_drop_notices_queued="); out.println(s.dropNoticesQueued);
    out.print(p); out.print("_pending_drop_notice_records="); out.println(s.pendingDropNoticeRecords);
    out.print(p); out.print("_enqueue_calls="); out.println(s.enqueueCalls);
    out.print(p); out.print("_drain_calls="); out.println(s.drainCalls);
    out.print(p); out.print("_drain_stalls="); out.println(s.drainStalls);
}

} // namespace tracker

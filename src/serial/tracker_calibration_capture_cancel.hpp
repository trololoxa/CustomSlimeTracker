#pragma once

#include "sensor/fifo_calibrations.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

// Temporarily connects a blocking FIFO calibration capture to the command
// stream. Only q/Q requests cancellation; line endings and other bytes are
// consumed because the command dispatcher cannot safely run recursively.
class TrackerCalibrationCaptureCancelScope {
public:
    explicit TrackerCalibrationCaptureCancelScope(TrackerSerialCommandContext& ctx)
        : io_(ctx.calibrationIo), stream_(ctx.io) {
        if (!io_ || !stream_) return;
        previousCallback_ = io_->cancelRequested;
        previousUser_ = io_->cancelUser;
        io_->cancelRequested = &poll;
        io_->cancelUser = this;
        installed_ = true;
    }

    ~TrackerCalibrationCaptureCancelScope() {
        if (!installed_) return;
        io_->cancelRequested = previousCallback_;
        io_->cancelUser = previousUser_;
    }

    TrackerCalibrationCaptureCancelScope(const TrackerCalibrationCaptureCancelScope&) = delete;
    TrackerCalibrationCaptureCancelScope& operator=(
        const TrackerCalibrationCaptureCancelScope&) = delete;

private:
    static bool poll(void* user) {
        auto* self = static_cast<TrackerCalibrationCaptureCancelScope*>(user);
        if (!self || !self->stream_) return false;
        if (self->previousCallback_ && self->previousCallback_(self->previousUser_)) self->cancelled_ = true;
        for (uint8_t count = 0u; count < 32u && self->stream_->available() > 0; ++count) {
            const int value = self->stream_->read();
            if (value == 'q' || value == 'Q') self->cancelled_ = true;
        }
        return self->cancelled_;
    }

    FifoCalibrationIo* io_ = nullptr;
    Stream* stream_ = nullptr;
    bool (*previousCallback_)(void*) = nullptr;
    void* previousUser_ = nullptr;
    bool cancelled_ = false;
    bool installed_ = false;
};

} // namespace tracker

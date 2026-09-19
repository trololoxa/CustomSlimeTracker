#pragma once

#include <Arduino.h>
#include <cstddef>

#include "defines.h"
#include "serial/tracker_command_line_parser.hpp"
#include "serial/tracker_serial_context.hpp"

#if TRACKER_ENABLE_SERIAL_CLI
#include "serial/tracker_system_commands.hpp"
#if TRACKER_HAS_MOTION_LIGHT_SLEEP
#include "serial/tracker_sleep_commands.hpp"
#endif
#if TRACKER_ENABLE_RUNTIME_PROFILER
#include "serial/tracker_perf_commands.hpp"
#include "serial/tracker_motion_commands.hpp"
#endif
#if TRACKER_ENABLE_CONFIG_COMMANDS
#include "serial/tracker_config_commands.hpp"
#include "serial/tracker_fifo_config_control.hpp"
#endif
#if TRACKER_ENABLE_FULL_CLI
#include "serial/tracker_imu_fifo_commands.hpp"
#include "serial/tracker_ahrs_commands.hpp"
#include "serial/tracker_output_commands.hpp"
#include "serial/tracker_bias_commands.hpp"
#endif
#if TRACKER_ENABLE_CALIBRATION_COMMANDS
#include "serial/tracker_calibration_commands.hpp"
#endif
#if TRACKER_ENABLE_MAG_COMMANDS
#include "serial/tracker_mag_commands.hpp"
#endif
#if TRACKER_ENABLE_TEST_COMMANDS
#include "serial/tracker_test_commands.hpp"
#endif
#if TRACKER_ENABLE_NETWORK_COMMANDS
#include "serial/tracker_network_commands.hpp"
#endif
#if TRACKER_ENABLE_SLIMEVR_COMMANDS
#include "serial/tracker_slimevr_commands.hpp"
#endif
#if TRACKER_ENABLE_SETUP_COMMANDS
#include "serial/tracker_setup_commands.hpp"
#endif
#if TRACKER_ENABLE_TAP_RUNTIME
#include "serial/tracker_tap_commands.hpp"
#endif
#if TRACKER_ENABLE_STATUS_LED
#include "serial/tracker_led_commands.hpp"
#endif
#if TRACKER_ENABLE_BATTERY_RUNTIME
#include "serial/tracker_battery_commands.hpp"
#endif
#if TRACKER_ENABLE_SLIMEVR_SERIAL_COMPAT
#include "serial/tracker_slimevr_serial_compat_commands.hpp"
#endif
#endif // TRACKER_ENABLE_SERIAL_CLI

#if defined(__GNUC__) || defined(__clang__)
#define TRACKER_SERIAL_NOINLINE __attribute__((noinline))
#else
#define TRACKER_SERIAL_NOINLINE
#endif

namespace tracker {

class TrackerTelnetInputFilter {
public:
    void reset() { state_ = State::Data; }

    // Returns true for Telnet negotiation/subnegotiation bytes that must not
    // enter the ASCII CLI line buffer. Raw TCP clients are unaffected.
    bool consume(uint8_t value) {
        switch (state_) {
            case State::Data:
                if (value == 0xffu) {
                    state_ = State::Iac;
                    return true;
                }
                return value == 0u;  // Telnet CR-NUL line ending.
            case State::Iac:
                if (value >= 0xfbu && value <= 0xfeu) state_ = State::Option;
                else if (value == 0xfau) state_ = State::Subnegotiation;
                else state_ = State::Data;
                return true;
            case State::Option:
                state_ = State::Data;
                return true;
            case State::Subnegotiation:
                if (value == 0xffu) state_ = State::SubnegotiationIac;
                return true;
            case State::SubnegotiationIac:
                state_ = value == 0xf0u ? State::Data : State::Subnegotiation;
                return true;
        }
        state_ = State::Data;
        return true;
    }

private:
    enum class State : uint8_t {
        Data,
        Iac,
        Option,
        Subnegotiation,
        SubnegotiationIac,
    };

    State state_ = State::Data;
};

class TrackerCommandDispatcher {
public:
    static void dispatch(TrackerSerialCommandContext& ctx, int argc, char** argv);
};

// 192 bytes covers the longest supported SET BWIFI command (32-byte SSID and
// 64-byte password after base64 expansion) without dynamic allocation.
template <size_t LINE_CAP = 192, size_t MAX_ARGS = 10>
class TrackerSerialCommandInterface {
public:
    void begin(TrackerSerialCommandContext& ctx) {
        ctx_ = &ctx;
        len_ = 0;
        overflow_ = false;
        telnetFilter_.reset();
    }

    void setFactoryResetRecoveryOnly(bool enabled) {
        if (ctx_) ctx_->factoryResetRecoveryOnly = enabled;
    }

    TRACKER_SERIAL_NOINLINE size_t poll(size_t maxBytes = 0) {
        if (!ctx_ || !ctx_->io) return 0;

        size_t consumed = 0;
        Stream& s = *ctx_->io;
        while (s.available() > 0) {
            if (maxBytes > 0 && consumed >= maxBytes) break;
            const int raw = s.read();
            if (raw < 0) break;
            if (ctx_->origin == TrackerCommandOrigin::RemoteTcp &&
                telnetFilter_.consume(static_cast<uint8_t>(raw))) {
                consumed++;
                continue;
            }
            const char c = static_cast<char>(raw);
            feed(c);
            consumed++;
        }
        return consumed;
    }

    void feed(char c) {
        if (!ctx_ || !ctx_->io) return;

        if (c == '\r') return;

        if (c == '\n') {
            line_[len_] = '\0';
            if (overflow_) {
                tracker_serial_detail::printErr(*ctx_->io, "line too long");
            } else {
                processLine(line_);
            }
            len_ = 0;
            overflow_ = false;
            return;
        }

        if (c == '\b' || c == 0x7F) {
            if (len_ > 0) len_--;
            return;
        }

        if (len_ >= LINE_CAP - 1) {
            overflow_ = true;
            return;
        }

        line_[len_++] = c;
    }

    void processLine(char* line) {
        if (!ctx_ || !line) return;

        char* argv[MAX_ARGS] = {};
        const TrackerCommandLineParseResult parsed =
            trackerParseCommandLine(line, argv, MAX_ARGS);
        switch (parsed.status) {
            case TrackerCommandLineParseStatus::Empty:
                return;
            case TrackerCommandLineParseStatus::TooManyArguments:
                tracker_serial_detail::printErr(*ctx_->io, "too many arguments");
                return;
            case TrackerCommandLineParseStatus::UnclosedQuote:
                tracker_serial_detail::printErr(*ctx_->io, "unclosed quote");
                return;
            case TrackerCommandLineParseStatus::TrailingCharactersAfterQuote:
                tracker_serial_detail::printErr(*ctx_->io, "characters after closing quote");
                return;
            case TrackerCommandLineParseStatus::Ok:
                break;
        }

        TrackerCommandDispatcher::dispatch(*ctx_, static_cast<int>(parsed.argc), argv);
    }

private:
    TrackerSerialCommandContext* ctx_ = nullptr;
    char line_[LINE_CAP] = {};
    size_t len_ = 0;
    bool overflow_ = false;
    TrackerTelnetInputFilter telnetFilter_;
};

} // namespace tracker

#undef TRACKER_SERIAL_NOINLINE

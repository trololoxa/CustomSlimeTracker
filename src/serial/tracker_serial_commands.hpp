#pragma once

#include <Arduino.h>
#include <cstddef>

#include "defines.h"
#include "serial/tracker_serial_context.hpp"

#if TRACKER_ENABLE_SERIAL_CLI
#include "serial/tracker_system_commands.hpp"
#if TRACKER_ENABLE_CONFIG_COMMANDS
#include "serial/tracker_config_commands.hpp"
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

namespace tracker {

class TrackerCommandDispatcher {
public:
    static void dispatch(TrackerSerialCommandContext& ctx, int argc, char** argv);
};

template <size_t LINE_CAP = 128, size_t MAX_ARGS = 10>
class TrackerSerialCommandInterface {
public:
    void begin(TrackerSerialCommandContext& ctx) {
        ctx_ = &ctx;
        len_ = 0;
        overflow_ = false;
    }

    size_t poll(size_t maxBytes = 0) {
        if (!ctx_ || !ctx_->io) return 0;

        size_t consumed = 0;
        Stream& s = *ctx_->io;
        while (s.available() > 0) {
            if (maxBytes > 0 && consumed >= maxBytes) break;
            const char c = static_cast<char>(s.read());
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
        const int argc = tokenize(line, argv, MAX_ARGS);
        if (argc <= 0) return;

        TrackerCommandDispatcher::dispatch(*ctx_, argc, argv);
    }

private:
    static int tokenize(char* line, char** argv, size_t maxArgs) {
        size_t argc = 0;
        char* p = line;

        while (*p && argc < maxArgs) {
            while (*p == ' ' || *p == '\t') ++p;
            if (*p == '\0') break;
            if (*p == '#') break;

            char* dst = p;
            bool quoted = false;
            if (*p == '"') {
                quoted = true;
                ++p;
                argv[argc++] = dst;
                while (*p) {
                    if (*p == '\\' && p[1] != '\0') {
                        ++p;
                        *dst++ = *p++;
                        continue;
                    }
                    if (*p == '"') {
                        ++p;
                        break;
                    }
                    *dst++ = *p++;
                }
                *dst = '\0';
            } else {
                argv[argc++] = p;
                while (*p && *p != ' ' && *p != '\t') ++p;
                if (*p == '\0') break;
                *p++ = '\0';
            }

            if (quoted) {
                while (*p && *p != ' ' && *p != '\t') {
                    // Treat trailing garbage after a closing quote as part of
                    // token separation rather than another argument; this keeps
                    // command parsing deterministic for malformed host input.
                    ++p;
                }
                if (*p == '\0') break;
                *p++ = '\0';
            }
        }

        return static_cast<int>(argc);
    }

    TrackerSerialCommandContext* ctx_ = nullptr;
    char line_[LINE_CAP] = {};
    size_t len_ = 0;
    bool overflow_ = false;
};

} // namespace tracker

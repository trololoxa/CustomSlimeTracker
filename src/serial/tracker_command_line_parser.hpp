#pragma once

#include <cstddef>
#include <cstdint>

namespace tracker {

enum class TrackerCommandLineParseStatus : uint8_t {
    Empty = 0,
    Ok,
    TooManyArguments,
    UnclosedQuote,
    TrailingCharactersAfterQuote,
};

struct TrackerCommandLineParseResult {
    TrackerCommandLineParseStatus status = TrackerCommandLineParseStatus::Empty;
    size_t argc = 0;
};

// In-place, allocation-free tokenizer shared by USB and TCP command paths.
// A malformed line is never dispatched, even if it has a valid prefix.
inline TrackerCommandLineParseResult trackerParseCommandLine(
    char* line, char** argv, size_t maxArgs) {
    TrackerCommandLineParseResult result;
    if (!line || !argv || maxArgs == 0u) return result;

    char* p = line;
    while (true) {
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '\0' || *p == '#') break;
        if (result.argc == maxArgs) {
            result.status = TrackerCommandLineParseStatus::TooManyArguments;
            return result;
        }

        if (*p == '"') {
            char* dst = p;
            ++p;
            argv[result.argc++] = dst;
            bool closed = false;
            while (*p != '\0') {
                if (*p == '\\') {
                    ++p;
                    if (*p == '\0') break;
                    *dst++ = *p++;
                    continue;
                }
                if (*p == '"') {
                    ++p;
                    closed = true;
                    break;
                }
                *dst++ = *p++;
            }
            *dst = '\0';
            if (!closed) {
                result.status = TrackerCommandLineParseStatus::UnclosedQuote;
                return result;
            }
            if (*p != '\0' && *p != ' ' && *p != '\t' && *p != '#') {
                result.status = TrackerCommandLineParseStatus::TrailingCharactersAfterQuote;
                return result;
            }
        } else {
            argv[result.argc++] = p;
            while (*p != '\0' && *p != ' ' && *p != '\t') ++p;
            if (*p == '\0') break;
            *p++ = '\0';
        }
    }

    result.status = result.argc == 0u
        ? TrackerCommandLineParseStatus::Empty
        : TrackerCommandLineParseStatus::Ok;
    return result;
}

} // namespace tracker

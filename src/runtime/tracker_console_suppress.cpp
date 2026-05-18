#include "runtime/tracker_console_suppress.hpp"

namespace tracker {
namespace {

uint32_t g_suppressTrackingMessagesUntilMs = 0;

bool timeBefore(uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a - b) < 0;
}

} // namespace

void trackerConsoleSuppressTrackingMessagesUntil(uint32_t untilMs) {
    const uint32_t nowMs = millis();
    if (untilMs == 0 || !timeBefore(nowMs, untilMs)) {
        return;
    }
    if (g_suppressTrackingMessagesUntilMs == 0 ||
        timeBefore(g_suppressTrackingMessagesUntilMs, untilMs)) {
        g_suppressTrackingMessagesUntilMs = untilMs;
    }
}

void trackerConsoleSuppressTrackingMessagesFor(uint32_t durationMs) {
    if (durationMs == 0) return;
    const uint32_t nowMs = millis();
    trackerConsoleSuppressTrackingMessagesUntil(nowMs + durationMs);
}

bool trackerConsoleTrackingMessagesSuppressed(uint32_t nowMs) {
    if (g_suppressTrackingMessagesUntilMs == 0) return false;
    if (timeBefore(nowMs, g_suppressTrackingMessagesUntilMs)) return true;
    g_suppressTrackingMessagesUntilMs = 0;
    return false;
}

uint32_t trackerConsoleTrackingMessagesSuppressedUntilMs() {
    return g_suppressTrackingMessagesUntilMs;
}

} // namespace tracker

#pragma once

#include <Arduino.h>
#include <cstdint>

#include "defines.h"

namespace tracker {

#if TRACKER_ENABLE_CONSOLE_SUPPRESS

// Suppresses human-facing recovery/state console noise for a short grace window
// after intentionally blocking diagnostic/setup commands such as Wi-Fi scans.
// Machine-log state events and counters still update; this only avoids mixing
// expected FIFO-recovery chatter into command replies consumed by SlimeVR Server.
void trackerConsoleSuppressTrackingMessagesFor(uint32_t durationMs);
void trackerConsoleSuppressTrackingMessagesUntil(uint32_t untilMs);
bool trackerConsoleTrackingMessagesSuppressed(uint32_t nowMs = millis());
uint32_t trackerConsoleTrackingMessagesSuppressedUntilMs();

#else

static inline void trackerConsoleSuppressTrackingMessagesFor(uint32_t durationMs) {
    (void)durationMs;
}

static inline void trackerConsoleSuppressTrackingMessagesUntil(uint32_t untilMs) {
    (void)untilMs;
}

static inline bool trackerConsoleTrackingMessagesSuppressed(uint32_t nowMs = millis()) {
    (void)nowMs;
    return false;
}

static inline uint32_t trackerConsoleTrackingMessagesSuppressedUntilMs() {
    return 0;
}

#endif

} // namespace tracker

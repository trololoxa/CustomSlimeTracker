#pragma once

#include <Arduino.h>
#include <cstdint>

namespace tracker {

// Suppresses human-facing recovery/state console noise for a short grace window
// after intentionally blocking diagnostic/setup commands such as Wi-Fi scans.
// Machine-log state events and counters still update; this only avoids mixing
// expected FIFO-recovery chatter into command replies consumed by SlimeVR Server.
void trackerConsoleSuppressTrackingMessagesFor(uint32_t durationMs);
void trackerConsoleSuppressTrackingMessagesUntil(uint32_t untilMs);
bool trackerConsoleTrackingMessagesSuppressed(uint32_t nowMs = millis());
uint32_t trackerConsoleTrackingMessagesSuppressedUntilMs();

} // namespace tracker

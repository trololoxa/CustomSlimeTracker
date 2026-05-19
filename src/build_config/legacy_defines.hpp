#pragma once

// Backward-compatible no-op/alias macros for old build flags. Keep them here
// while command/runtime modules are moved from coarse feature gates to profile
// aware source filtering in later optimization waves.
#ifndef TRACKER_TAP_SEND_SINGLE
#define TRACKER_TAP_SEND_SINGLE 0
#endif

#ifndef TRACKER_TAP_COOLDOWN_MS
#define TRACKER_TAP_COOLDOWN_MS TRACKER_TAP_POST_SEND_LOCKOUT_MS
#endif

#pragma once

// PlatformIO adds a generated include directory before compiling project
// sources. Native tests and non-PlatformIO tooling intentionally use the
// fallback values below.
#if __has_include("tracker_build_identity_generated.hpp")
#include "tracker_build_identity_generated.hpp"
#else
#define TRACKER_BUILD_GIT_AVAILABLE 0
#define TRACKER_BUILD_GIT_HEAD "unknown"
#define TRACKER_BUILD_WORKTREE_FINGERPRINT "unknown"
#define TRACKER_BUILD_GIT_DIRTY 0
#define TRACKER_BUILD_IDENTITY_STRING "unknown"
#define TRACKER_BUILD_PIO_ENVIRONMENT "native-or-unknown"
#define TRACKER_BUILD_FIRMWARE_VERSION "c3-6dsv-unknown"
#endif

namespace tracker {

static inline bool trackerBuildGitAvailable() {
    return TRACKER_BUILD_GIT_AVAILABLE != 0;
}

static inline bool trackerBuildGitDirty() {
    return TRACKER_BUILD_GIT_DIRTY != 0;
}

static inline const char* trackerBuildGitHead() {
    return TRACKER_BUILD_GIT_HEAD;
}

static inline const char* trackerBuildWorktreeFingerprint() {
    return TRACKER_BUILD_WORKTREE_FINGERPRINT;
}

static inline const char* trackerBuildIdentityString() {
    return TRACKER_BUILD_IDENTITY_STRING;
}

static inline const char* trackerBuildPioEnvironment() {
    return TRACKER_BUILD_PIO_ENVIRONMENT;
}

static inline const char* trackerBuildFirmwareVersion() {
    return TRACKER_BUILD_FIRMWARE_VERSION;
}

} // namespace tracker

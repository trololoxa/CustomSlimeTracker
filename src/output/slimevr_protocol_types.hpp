#pragma once

#include <cstdint>

namespace tracker {

// Server-side actions accepted by SlimeVR packet 21. None is a local sentinel
// used for disabled tap mapping and is never serialized.
enum class SlimeVRUserAction : uint8_t {
    None = 0,
    FullReset = 2,
    YawReset = 3,
    MountingReset = 4,
    PauseTracking = 5,
};

const char* slimevrUserActionName(SlimeVRUserAction action);
bool parseSlimeVRUserActionName(const char* text, SlimeVRUserAction& out);

} // namespace tracker

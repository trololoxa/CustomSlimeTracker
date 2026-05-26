// Force the SlimeVR output runtime implementation into PlatformIO builds from
// the src/ root. This keeps it resilient to profile/source-filter merges that
// accidentally skip nested runtime/*.cpp files.
#include "runtime/slimevr_output_runtime_impl.inc"

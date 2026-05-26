// PlatformIO builds use a root-level link unit for the SlimeVR runtime.
//
// Some local/merged trees ended up with source filters that dropped this
// nested translation unit while headers still referenced SlimeVROutputRuntime,
// producing undefined references at link time. Keep this file usable for
// native tests and non-PlatformIO builds, but allow PlatformIO to disable it
// and compile the implementation through src/slimevr_output_runtime_link_unit.cpp.
#ifndef TRACKER_SLIMEVR_OUTPUT_RUNTIME_DISABLE_STANDALONE_TU
#include "runtime/slimevr_output_runtime_impl.inc"
#endif

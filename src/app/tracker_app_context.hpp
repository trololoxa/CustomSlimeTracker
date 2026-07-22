#pragma once

// Tracker application context: top-level app composition entry points.
// Ownership is split by responsibility:
//   - tracker_hardware_context.hpp: board transports, raw buffers and ISR state
//   - tracker_runtime_context.hpp: logical runtime singletons and counters
//   - tracker_app_hooks.hpp: dependency builders and callback glue

#include "app/tracker_hardware_context.hpp"
#include "app/tracker_runtime_context.hpp"
#include "app/tracker_app_hooks.hpp"

#if TRACKER_HAS_SERIAL_CONSOLE
static void trackerAppSerialFlush(void*) {
    (void)g_serialConsoleStream.drain(TRACKER_SERIAL_OUTPUT_BYTES_PER_DRAIN);
}
#endif

static void trackerAppContextSetup() {
#if TRACKER_HAS_SERIAL_CONSOLE
    g_serialConsoleStream.begin(Serial);
    g_serialConsoleStream.setFlushHandler(trackerAppSerialFlush, nullptr);
#endif
    g_app.begin(makeTrackerAppDeps());
    g_app.setup();
}

static void trackerAppContextLoop() {
    g_app.loop();
}

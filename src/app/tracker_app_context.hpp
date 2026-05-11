#pragma once

// Tracker application context: top-level app composition entry points.
// Ownership is split by responsibility:
//   - tracker_hardware_context.hpp: board transports, raw buffers and ISR state
//   - tracker_runtime_context.hpp: logical runtime singletons and counters
//   - tracker_app_hooks.hpp: dependency builders and callback glue

#include "app/tracker_hardware_context.hpp"
#include "app/tracker_runtime_context.hpp"
#include "app/tracker_app_hooks.hpp"

static void trackerAppContextSetup() {
    g_app.begin(makeTrackerAppDeps());
    g_app.setup();
}

static void trackerAppContextLoop() {
    g_app.loop();
}

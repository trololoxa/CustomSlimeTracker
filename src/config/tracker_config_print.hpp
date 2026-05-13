#pragma once

#include <Arduino.h>

#include "config/tracker_config_runtime.hpp"
#include "config/tracker_network_config.hpp"

namespace tracker {

void printTrackerConfigSummary(Stream& out, const TrackerConfig& cfg);
void printTrackerNetworkConfigSummary(Stream& out, const TrackerNetworkConfig& cfg, bool revealSecrets = false);

} // namespace tracker

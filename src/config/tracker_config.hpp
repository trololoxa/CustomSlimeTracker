#pragma once

// Umbrella include kept for compatibility.
// Implementation is split by responsibility; include narrower headers in new code
// when only schema, runtime apply, NVS store, network config, or printing is needed.

#include "config/tracker_config_detail.hpp"
#include "config/tracker_config_schema.hpp"
#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "config/tracker_network_config.hpp"
#include "config/tracker_config_print.hpp"

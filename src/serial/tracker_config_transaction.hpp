#pragma once

#include <utility>

#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

// Commit a fully prepared configuration candidate.  When persistence is
// requested, the active RAM configuration and runtime state are changed only
// after NVS accepted the candidate.  This is intentionally a small current-
// schema transaction helper; the dual-slot storage work belongs to patch 0021.
template <typename ApplyFn>
bool trackerCommitConfigCandidate(TrackerSerialCommandContext& ctx,
                                  TrackerConfig candidate,
                                  bool persist,
                                  ApplyFn&& applyRuntime) {
    if (!ctx.config) return false;

    candidate.sanitize();
    candidate.updateCrc();

    if (persist) {
        if (!ctx.configStore ||
            !ctx.configStore->save(candidate, TrackerCalibrationProvenance::Manual)) {
            return false;
        }
    }

    *ctx.config = candidate;
    std::forward<ApplyFn>(applyRuntime)();
    return true;
}

inline bool trackerCommitConfigCandidate(TrackerSerialCommandContext& ctx,
                                         TrackerConfig candidate,
                                         bool persist) {
    return trackerCommitConfigCandidate(ctx, candidate, persist, []() {});
}

} // namespace tracker

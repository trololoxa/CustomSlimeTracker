#pragma once

#include <utility>

#include "config/tracker_config_runtime.hpp"
#include "config/tracker_config_store.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

enum class TrackerConfigCommitMode : uint8_t {
    PersistThenApply,
    VolatilePreview,
};

// Commit a fully prepared immutable configuration candidate. When persistence is
// requested, the active RAM configuration and runtime state are changed only
// after NVS accepted and verified the candidate. VolatilePreview is explicit
// at call sites so a preview cannot be mistaken for durable configuration.
// This is intentionally a small current-
// schema transaction helper; the dual-slot storage work belongs to patch 0021.
template <typename ApplyFn>
bool trackerCommitConfigCandidate(TrackerSerialCommandContext& ctx,
                                  TrackerConfig candidate,
                                  TrackerConfigCommitMode mode,
                                  ApplyFn&& applyRuntime) {
    if (!ctx.config) return false;

    if (!candidate.validateSemanticConfig()) return false;
    candidate.updateCrc();

    if (mode == TrackerConfigCommitMode::PersistThenApply) {
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
                                         TrackerConfigCommitMode mode) {
    return trackerCommitConfigCandidate(ctx, candidate, mode, []() {});
}

} // namespace tracker

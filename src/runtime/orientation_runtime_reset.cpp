#include "runtime/orientation_runtime_reset.hpp"

namespace tracker {

bool resetOrientationRuntime(const OrientationRuntimeResetDeps& deps,
                             const char* reason,
                             uint64_t timestampUs) {
    if (deps.ahrs == nullptr || deps.preparedOutput == nullptr) {
        return false;
    }

    deps.ahrs->reset();
    deps.preparedOutput->reset();

    if (deps.resetDependentState != nullptr) {
        deps.resetDependentState(reason, timestampUs, false);
    }
    return true;
}

} // namespace tracker

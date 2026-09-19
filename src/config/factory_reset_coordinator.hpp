#pragma once

#include <cstdint>

#include "config/tracker_config_store.hpp"
#include "config/tracker_network_config.hpp"
#include "runtime/calibration_autonomy_store.hpp"

namespace tracker {

enum class FactoryResetScope : uint8_t {
    Main = 1,
    Network = 2,
    Calibration = 3,
    Full = 4,
};

const char* factoryResetScopeName(FactoryResetScope scope);

enum class FactoryResetError : uint8_t {
    None = 0,
    InvalidScope,
    MarkerReadFailed,
    MarkerInvalid,
    PendingScopeMismatch,
    MarkerWriteFailed,
    MainResetFailed,
    NetworkResetFailed,
    CalibrationResetFailed,
    MarkerClearFailed,
};

const char* factoryResetErrorName(FactoryResetError error);

namespace factory_reset_detail {
static constexpr const char* NVS_NAMESPACE = "factory_reset";
static constexpr const char* NVS_KEY_PENDING = "pending";
static constexpr uint32_t MARKER_MAGIC = 0x54535246UL; // 'FRST'
static constexpr uint16_t MARKER_VERSION = 1u;
static constexpr uint16_t MARKER_ENCODED_SIZE = 16u;
}

struct FactoryResetMarker {
    uint32_t magic = factory_reset_detail::MARKER_MAGIC;
    uint16_t version = factory_reset_detail::MARKER_VERSION;
    uint16_t size = factory_reset_detail::MARKER_ENCODED_SIZE;
    FactoryResetScope scope = FactoryResetScope::Full;
    uint8_t completedSteps = 0u;
    uint8_t reserved[2] = {};
    uint32_t crc32 = 0u;
};

// Cold-path transaction coordinator. Each destructive step is preceded by a
// verified marker and followed by a verified checkpoint. Replaying a step is
// safe, so boot can finish an interrupted reset before loading runtime state.
class FactoryResetCoordinator {
public:
    FactoryResetCoordinator(TrackerConfigStore& configStore,
                            TrackerNetworkConfigStore& networkStore,
                            CalibrationAutonomyStore* autonomyStore = nullptr,
                            const char* markerNamespace = factory_reset_detail::NVS_NAMESPACE);

    bool request(FactoryResetScope scope);
    bool resumePending();
    bool pending(bool& outPending, FactoryResetScope* outScope = nullptr);
    FactoryResetError lastError() const { return lastError_; }
    const char* lastErrorName() const { return factoryResetErrorName(lastError_); }

    static uint32_t markerCrc(const FactoryResetMarker& marker);
    static bool valid(const FactoryResetMarker& marker);

private:
    TrackerConfigStore& configStore_;
    TrackerNetworkConfigStore& networkStore_;
    CalibrationAutonomyStore* autonomyStore_ = nullptr;
    const char* markerNamespace_ = nullptr;
    FactoryResetError lastError_ = FactoryResetError::None;

    bool readMarker(FactoryResetMarker& marker, bool& exists);
    bool writeMarker(const FactoryResetMarker& marker);
    bool clearMarker();
    bool advanceConfirmed(FactoryResetMarker& marker);
    bool advance(FactoryResetMarker& marker);
};

} // namespace tracker

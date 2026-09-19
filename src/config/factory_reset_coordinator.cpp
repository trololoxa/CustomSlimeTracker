#include "config/factory_reset_coordinator.hpp"

#include <Preferences.h>
#include <cstddef>
#include <cstring>

namespace tracker {
namespace {

constexpr uint8_t STEP_MAIN = 1u << 0;
constexpr uint8_t STEP_NETWORK = 1u << 1;
constexpr uint8_t STEP_CALIBRATION = 1u << 2;
constexpr size_t MARKER_BYTES = factory_reset_detail::MARKER_ENCODED_SIZE;

void putU16(uint8_t* out, uint16_t value) {
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8u);
}

void putU32(uint8_t* out, uint32_t value) {
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8u);
    out[2] = static_cast<uint8_t>(value >> 16u);
    out[3] = static_cast<uint8_t>(value >> 24u);
}

uint16_t getU16(const uint8_t* in) {
    return static_cast<uint16_t>(in[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(in[1]) << 8u);
}

uint32_t getU32(const uint8_t* in) {
    return static_cast<uint32_t>(in[0]) |
           (static_cast<uint32_t>(in[1]) << 8u) |
           (static_cast<uint32_t>(in[2]) << 16u) |
           (static_cast<uint32_t>(in[3]) << 24u);
}

void encodeMarker(const FactoryResetMarker& marker, uint8_t out[MARKER_BYTES]) {
    std::memset(out, 0, MARKER_BYTES);
    putU32(out, marker.magic);
    putU16(out + 4u, marker.version);
    putU16(out + 6u, marker.size);
    out[8] = static_cast<uint8_t>(marker.scope);
    out[9] = marker.completedSteps;
    out[10] = marker.reserved[0];
    out[11] = marker.reserved[1];
    putU32(out + 12u, marker.crc32);
}

FactoryResetMarker decodeMarker(const uint8_t in[MARKER_BYTES]) {
    FactoryResetMarker marker;
    marker.magic = getU32(in);
    marker.version = getU16(in + 4u);
    marker.size = getU16(in + 6u);
    marker.scope = static_cast<FactoryResetScope>(in[8]);
    marker.completedSteps = in[9];
    marker.reserved[0] = in[10];
    marker.reserved[1] = in[11];
    marker.crc32 = getU32(in + 12u);
    return marker;
}

bool validScope(FactoryResetScope scope) {
    return scope == FactoryResetScope::Main || scope == FactoryResetScope::Network ||
           scope == FactoryResetScope::Calibration || scope == FactoryResetScope::Full;
}

bool includesMain(FactoryResetScope scope) {
    return scope == FactoryResetScope::Main || scope == FactoryResetScope::Calibration ||
           scope == FactoryResetScope::Full;
}

bool includesNetwork(FactoryResetScope scope) {
    return scope == FactoryResetScope::Network || scope == FactoryResetScope::Full;
}

bool includesCalibrationMetadata(FactoryResetScope scope) {
    return scope == FactoryResetScope::Main || scope == FactoryResetScope::Calibration ||
           scope == FactoryResetScope::Full;
}

} // namespace

const char* factoryResetScopeName(FactoryResetScope scope) {
    switch (scope) {
        case FactoryResetScope::Main: return "main";
        case FactoryResetScope::Network: return "network";
        case FactoryResetScope::Calibration: return "calibration";
        case FactoryResetScope::Full: return "full";
    }
    return "invalid";
}

const char* factoryResetErrorName(FactoryResetError error) {
    switch (error) {
        case FactoryResetError::None: return "none";
        case FactoryResetError::InvalidScope: return "invalid_scope";
        case FactoryResetError::MarkerReadFailed: return "marker_read_failed";
        case FactoryResetError::MarkerInvalid: return "marker_invalid";
        case FactoryResetError::PendingScopeMismatch: return "pending_scope_mismatch";
        case FactoryResetError::MarkerWriteFailed: return "marker_write_failed";
        case FactoryResetError::MainResetFailed: return "main_reset_failed";
        case FactoryResetError::NetworkResetFailed: return "network_reset_failed";
        case FactoryResetError::CalibrationResetFailed: return "calibration_reset_failed";
        case FactoryResetError::MarkerClearFailed: return "marker_clear_failed";
    }
    return "unknown";
}

FactoryResetCoordinator::FactoryResetCoordinator(
    TrackerConfigStore& configStore,
    TrackerNetworkConfigStore& networkStore,
    CalibrationAutonomyStore* autonomyStore,
    const char* markerNamespace)
    : configStore_(configStore),
      networkStore_(networkStore),
      autonomyStore_(autonomyStore),
      markerNamespace_(markerNamespace) {}

uint32_t FactoryResetCoordinator::markerCrc(const FactoryResetMarker& marker) {
    FactoryResetMarker copy = marker;
    copy.crc32 = 0u;
    uint8_t encoded[MARKER_BYTES]{};
    encodeMarker(copy, encoded);
    return tracker_config_detail::fnv1a32(encoded, sizeof(encoded));
}

bool FactoryResetCoordinator::valid(const FactoryResetMarker& marker) {
    return marker.magic == factory_reset_detail::MARKER_MAGIC &&
           marker.version == factory_reset_detail::MARKER_VERSION &&
           marker.size == MARKER_BYTES && validScope(marker.scope) &&
           marker.reserved[0] == 0u && marker.reserved[1] == 0u &&
           (marker.completedSteps & static_cast<uint8_t>(~(STEP_MAIN | STEP_NETWORK | STEP_CALIBRATION))) == 0u &&
           marker.crc32 == markerCrc(marker);
}

bool FactoryResetCoordinator::readMarker(FactoryResetMarker& marker, bool& exists) {
    marker = FactoryResetMarker{};
    exists = false;
    Preferences prefs;
    // ESP32 NVS cannot open a never-created namespace read-only. Open it
    // read-write so a clean first boot creates only the namespace metadata;
    // no pending marker is written unless a reset is explicitly confirmed.
    if (!prefs.begin(markerNamespace_, false)) {
        lastError_ = FactoryResetError::MarkerReadFailed;
        return false;
    }
    exists = prefs.isKey(factory_reset_detail::NVS_KEY_PENDING);
    if (!exists) {
        prefs.end();
        return true;
    }
    uint8_t encoded[MARKER_BYTES]{};
    const bool read =
        prefs.getBytesLength(factory_reset_detail::NVS_KEY_PENDING) == sizeof(encoded) &&
        prefs.getBytes(factory_reset_detail::NVS_KEY_PENDING, encoded, sizeof(encoded)) == sizeof(encoded);
    prefs.end();
    if (read) marker = decodeMarker(encoded);
    if (!read) {
        lastError_ = FactoryResetError::MarkerReadFailed;
        return false;
    }
    if (!valid(marker)) {
        lastError_ = FactoryResetError::MarkerInvalid;
        return false;
    }
    return true;
}

bool FactoryResetCoordinator::writeMarker(const FactoryResetMarker& input) {
    FactoryResetMarker marker = input;
    marker.size = MARKER_BYTES;
    marker.crc32 = markerCrc(marker);
    uint8_t encoded[MARKER_BYTES]{};
    encodeMarker(marker, encoded);
    Preferences prefs;
    if (!prefs.begin(markerNamespace_, false)) {
        lastError_ = FactoryResetError::MarkerWriteFailed;
        return false;
    }
    const bool written = prefs.putBytes(
        factory_reset_detail::NVS_KEY_PENDING, encoded, sizeof(encoded)) == sizeof(encoded);
    uint8_t verifyBytes[MARKER_BYTES]{};
    const bool verified = written &&
        prefs.getBytesLength(factory_reset_detail::NVS_KEY_PENDING) == sizeof(verifyBytes) &&
        prefs.getBytes(factory_reset_detail::NVS_KEY_PENDING, verifyBytes, sizeof(verifyBytes)) == sizeof(verifyBytes) &&
        std::memcmp(verifyBytes, encoded, sizeof(encoded)) == 0 &&
        valid(decodeMarker(verifyBytes));
    prefs.end();
    if (!verified) {
        lastError_ = FactoryResetError::MarkerWriteFailed;
        return false;
    }
    return true;
}

bool FactoryResetCoordinator::clearMarker() {
    Preferences prefs;
    if (!prefs.begin(markerNamespace_, false)) {
        lastError_ = FactoryResetError::MarkerClearFailed;
        return false;
    }
    const bool removed = !prefs.isKey(factory_reset_detail::NVS_KEY_PENDING) ||
                         prefs.remove(factory_reset_detail::NVS_KEY_PENDING);
    const bool gone = removed && !prefs.isKey(factory_reset_detail::NVS_KEY_PENDING);
    prefs.end();
    if (!gone) {
        lastError_ = FactoryResetError::MarkerClearFailed;
        return false;
    }
    return true;
}

bool FactoryResetCoordinator::request(FactoryResetScope scope) {
    if (!validScope(scope)) {
        lastError_ = FactoryResetError::InvalidScope;
        return false;
    }
    bool alreadyPending = false;
    FactoryResetMarker oldMarker;
    if (!readMarker(oldMarker, alreadyPending)) {
        // A damaged marker is not executable. An explicit newly confirmed
        // request may replace it; boot-time resume never guesses its scope.
        if (lastError_ != FactoryResetError::MarkerInvalid) return false;
        alreadyPending = false;
    }
    if (alreadyPending) {
        if (oldMarker.scope != scope) {
            lastError_ = FactoryResetError::PendingScopeMismatch;
            return false;
        }
        return advanceConfirmed(oldMarker);
    }

    FactoryResetMarker marker;
    marker.scope = scope;
    marker.crc32 = markerCrc(marker);
    if (!writeMarker(marker)) return false;
    return advanceConfirmed(marker);
}

bool FactoryResetCoordinator::advanceConfirmed(FactoryResetMarker& marker) {
    // A verified pending marker can only originate from an explicitly
    // confirmed request. It remains the recovery escape after a reboot enters
    // safe mode, while all pre-existing write-inhibit policy is restored if
    // the transaction has to defer again.
    const bool configWasInhibited = configStore_.writeInhibited();
    const bool networkWasInhibited = networkStore_.writeInhibited();
    const bool autonomyWasInhibited = autonomyStore_ && autonomyStore_->writeInhibited();
    const bool probationBarrierWasEnabled = configStore_.autonomyProbationWriteBarrier();
    configStore_.setWriteInhibited(false);
    networkStore_.setWriteInhibited(false);
    if (autonomyStore_) autonomyStore_->setWriteInhibited(false);

    const bool result = advance(marker);
    configStore_.setWriteInhibited(configWasInhibited);
    configStore_.setAutonomyProbationWriteBarrier(probationBarrierWasEnabled);
    networkStore_.setWriteInhibited(networkWasInhibited);
    if (autonomyStore_) autonomyStore_->setWriteInhibited(autonomyWasInhibited);
    return result;
}

bool FactoryResetCoordinator::advance(FactoryResetMarker& marker) {
    if (includesMain(marker.scope) && (marker.completedSteps & STEP_MAIN) == 0u) {
        configStore_.setAutonomyProbationWriteBarrier(false);
        bool ok = false;
        if (marker.scope == FactoryResetScope::Calibration) {
            TrackerConfig clean;
            if (!configStore_.verify(clean)) {
                if (configStore_.lastError() != TrackerConfigError::NotFound) {
                    lastError_ = FactoryResetError::MainResetFailed;
                    return false;
                }
                clean.resetDefaults();
            }
            clean.clearAllCalibrationPreservingPolicy();
            ok = clean.validateSemanticConfig() &&
                 configStore_.save(clean, TrackerCalibrationProvenance::Manual) &&
                 configStore_.discardCandidate();
        } else {
            ok = configStore_.erase();
        }
        if (!ok) {
            lastError_ = FactoryResetError::MainResetFailed;
            return false;
        }
        marker.completedSteps |= STEP_MAIN;
        if (!writeMarker(marker)) return false;
    }

    if (includesNetwork(marker.scope) && (marker.completedSteps & STEP_NETWORK) == 0u) {
        if (!networkStore_.erase()) {
            lastError_ = FactoryResetError::NetworkResetFailed;
            return false;
        }
        marker.completedSteps |= STEP_NETWORK;
        if (!writeMarker(marker)) return false;
    }

    if (includesCalibrationMetadata(marker.scope) &&
        (marker.completedSteps & STEP_CALIBRATION) == 0u) {
#if TRACKER_HAS_CALIBRATION_AUTONOMY
        if (autonomyStore_ && !autonomyStore_->eraseAll()) {
            lastError_ = FactoryResetError::CalibrationResetFailed;
            return false;
        }
#endif
        marker.completedSteps |= STEP_CALIBRATION;
        if (!writeMarker(marker)) return false;
    }

    if (!clearMarker()) return false;
    lastError_ = FactoryResetError::None;
    return true;
}

bool FactoryResetCoordinator::resumePending() {
    FactoryResetMarker marker;
    bool exists = false;
    if (!readMarker(marker, exists)) return false;
    if (!exists) {
        lastError_ = FactoryResetError::None;
        return true;
    }
    return advanceConfirmed(marker);
}

bool FactoryResetCoordinator::pending(bool& outPending, FactoryResetScope* outScope) {
    FactoryResetMarker marker;
    if (!readMarker(marker, outPending)) return false;
    if (outPending && outScope) *outScope = marker.scope;
    lastError_ = FactoryResetError::None;
    return true;
}

} // namespace tracker

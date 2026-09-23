#pragma once
// Host-only measurement contracts; no production state or firmware dependencies.
#include "reference_rotation.hpp"
#include <cstdint>
#include <optional>

namespace dev05 {

// Count-based clean window, preserving the timestamp of its actual first sample.
// Later episodes must never replace the first completed window. This does not
// certify endpoint retention or turn a sample count into a wall-clock dwell.
class FirstSampleDwell {
public:
    FirstSampleDwell(uint32_t requiredSamples, uint64_t epochUs)
        : required_(requiredSamples), previousUs_(epochUs) {
        if (requiredSamples == 0) throw std::invalid_argument("empty dwell window");
    }

    void observe(uint64_t sampleUs, bool qualifies) {
        if (sampleUs <= previousUs_) throw std::invalid_argument("nonmonotonic dwell sample");
        previousUs_ = sampleUs;
        if (firstUs_) return;
        if (!qualifies) {
            consecutive_ = 0;
            return;
        }
        if (consecutive_ == 0) windowStartUs_ = sampleUs;
        if (++consecutive_ == required_) firstUs_ = windowStartUs_;
    }

    std::optional<uint64_t> firstWindowStartUs() const { return firstUs_; }

private:
    uint32_t required_;
    uint32_t consecutive_ = 0;
    uint64_t previousUs_;
    uint64_t windowStartUs_ = 0;
    std::optional<uint64_t> firstUs_;
};

// Numerical resolution for the single 0.2 rad/s * 1 ms propagation probe,
// not an AHRS accuracy SLA. The missing step is about 0.01146 degrees.
constexpr double propagationProbeToleranceDeg = 0.00001;
inline bool gyroPropagationObserved(bool updated, Q actual, Q expected,
                                    uint64_t previousIntegratedUs,
                                    uint64_t integratedUs, uint64_t sampleUs) {
    if (!updated || sampleUs <= previousIntegratedUs || integratedUs != sampleUs) return false;
    double normSq = 0;
    for (double v : actual) normSq += v * v;
    // Validate raw output before angular error normalizes it.
    if (!std::isfinite(normSq) || std::abs(std::sqrt(normSq) - 1) >= 0.0001) return false;
    return error(actual, expected) <= propagationProbeToleranceDeg;
}

} // namespace dev05

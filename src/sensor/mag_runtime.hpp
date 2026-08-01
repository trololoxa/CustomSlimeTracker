#pragma once

#include <cstdint>

#include "core/math.hpp"
#include "connection/lsm6dsv_fifo.hpp"
#include "sensor/frame_transform.hpp"

namespace tracker {

enum MagRejectFlags : uint32_t {
    MAG_REJECT_NONE             = 0,
    MAG_REJECT_DISABLED         = 1u << 0,
    MAG_REJECT_RAW_SATURATED    = 1u << 1,
    MAG_REJECT_RAW_NONFINITE    = 1u << 2,
    MAG_REJECT_NOT_CALIBRATED   = 1u << 3,
    MAG_REJECT_AXIS_NOT_ALIGNED = 1u << 4,
    MAG_REJECT_NORM_TOO_LOW     = 1u << 5,
    MAG_REJECT_NORM_TOO_HIGH    = 1u << 6,
    MAG_REJECT_STALE            = 1u << 7,
    MAG_REJECT_ZERO_NORM        = 1u << 8,
};

struct MagRuntimeConfig {
    bool enabled = false;

    bool calibrationValid = false;
    bool axisAlignmentValid = false;

    Vec3 hardIron = Vec3::zero();
    Mat3 softIron = Mat3::identity();
    Mat3 magToImu = Mat3::identity();
    bool sensorToDeviceValid = false;
    Mat3 sensorToDevice = Mat3::identity();
    // Runtime controllers may supply an already validated frame. Direct unit
    // tests and standalone callers retain the fail-closed fallback above.
    bool sensorToDevicePrevalidated = false;
    SensorToDeviceFrame sensorToDeviceFrame;

    float expectedFieldNorm = 1.0f;
    float minTrustNorm = 0.25f;
    float maxTrustNorm = 2.50f;

    uint32_t maxSampleAgeMs = 250;
    float minUsableNorm = 1.0e-6f;
};

struct MagProcessedSample {
    bool valid = false;
    bool trusted = false;

    uint32_t rejectFlags = MAG_REJECT_NONE;
    uint64_t t_us = 0;
    uint32_t receivedMs = 0;
    uint32_t seq = 0;

    Vec3 raw = Vec3::zero();
    Vec3 calibratedMagFrame = Vec3::zero();
    Vec3 body = Vec3::zero();

    float rawNorm = 0.0f;
    float calibratedNorm = 0.0f;
    float bodyNorm = 0.0f;

    uint16_t rawFlags = 0;

    // Gyro endpoint captured at the exact app callback that processed this
    // magnetic FIFO sample. The FIFO runtime dispatches mag chronologically,
    // so guided/runtime mag-axis learners can retain timestamp coherence even
    // if later IMU callbacks run before a polling consumer reads this struct.
    // True when the runtime processor validated and applied the configured
    // sensor-to-device proper rotation for this sample. The controller can
    // reuse the same accepted matrix for the coherent gyro endpoint instead
    // of repeating the expensive SO(3) validation in the 60 Hz hot path.
    bool sensorToDeviceApplied = false;
    bool gyroEndpointValid = false;
    Vec3 gyroSensorRadS = Vec3::zero();
    uint64_t gyroTimestampUs = 0;
    uint32_t gyroEndpointSkewUs = 0;
};

struct MagRuntimeStats {
    uint32_t rawSamples = 0;
    uint32_t processedSamples = 0;
    uint32_t trustedSamples = 0;
    uint32_t rejectedSamples = 0;

    uint32_t rejectedDisabled = 0;
    uint32_t rejectedRawSaturated = 0;
    uint32_t rejectedRawNonfinite = 0;
    uint32_t rejectedNotCalibrated = 0;
    uint32_t rejectedAxisNotAligned = 0;
    uint32_t rejectedNormTooLow = 0;
    uint32_t rejectedNormTooHigh = 0;
    uint32_t rejectedStale = 0;
    uint32_t rejectedZeroNorm = 0;

    uint32_t lastSampleMs = 0;
    uint32_t lastTrustedMs = 0;

    float rawNormMin = 0.0f;
    float rawNormMax = 0.0f;
    double rawNormSum = 0.0;

    float bodyNormMin = 0.0f;
    float bodyNormMax = 0.0f;
    double bodyNormSum = 0.0;

    float trustedBodyNormMin = 0.0f;
    float trustedBodyNormMax = 0.0f;
    double trustedBodyNormSum = 0.0;

    float rawNormMean() const;
    float bodyNormMean() const;
    float trustedBodyNormMean() const;
};

class MagRuntimeProcessor {
public:
    void reset();

    const MagRuntimeStats& stats() const;

    bool process(const Lsm6dsvFifoReader::MagRawSample& raw,
                 const MagRuntimeConfig& cfg,
                 uint32_t nowMs,
                 MagProcessedSample& out);


    static uint32_t ageMsForUse(const MagProcessedSample& sample, uint32_t nowMs);
    static uint32_t rejectFlagsForUse(const MagProcessedSample& sample,
                                      const MagRuntimeConfig& cfg,
                                      uint32_t nowMs);
    static bool trustedForUse(const MagProcessedSample& sample,
                              const MagRuntimeConfig& cfg,
                              uint32_t nowMs);
    static const char* rejectFlagName(uint32_t singleFlag);

private:
    void addReject(MagProcessedSample& out, uint32_t flag);
    void countRejects(uint32_t flags);
    void pushRawNorm(float n);
    void pushBodyNorm(float n);
    void pushTrustedBodyNorm(float n);

    MagRuntimeStats stats_;
};

} // namespace tracker

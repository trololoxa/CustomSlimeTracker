#pragma once

#include <cstdint>

#include "connection/lsm6dsv_driver.hpp"
#include "core/math.hpp"
#include "sensor/calibration.hpp"

namespace tracker {

// One-time physical board/case alignment. The device convention is:
//   +X = right
//   +Y = forward (use one repeatable marked case edge)
//   +Z = outward/top face
//
// The solver consumes two stationary gravity observations in the calibrated
// native LSM6DSV frame:
//   topSensorG     - device +Z points upward
//   forwardSensorG - device +Y points upward
// and returns the proper rotation that maps sensor vectors into device axes.
struct SensorToDeviceAlignmentParams {
    float minObservationNormG = 0.80f;
    float maxObservationNormG = 1.20f;
    float maxAbsObservationDot = 0.35f;
    float maxMappedAxisErrorDeg = 21.0f;
};

struct SensorToDeviceAlignmentResult {
    bool valid = false;
    Mat3 sensorToDevice = Mat3::identity();
    float topNormG = 0.0f;
    float forwardNormG = 0.0f;
    float observationSeparationDeg = 0.0f;
    float mappedTopErrorDeg = 180.0f;
    float mappedForwardErrorDeg = 180.0f;
    float determinant = 0.0f;
};

SensorToDeviceAlignmentResult solveSensorToDeviceAlignment(
    const Vec3& topSensorG,
    const Vec3& forwardSensorG,
    const SensorToDeviceAlignmentParams& params = SensorToDeviceAlignmentParams{}
);

struct SensorToDeviceCalibrationSeparationResult {
    bool valid = false;
    Mat3 sensorToDevice = Mat3::identity();
    Mat3 accelScaleSensorFrame = Mat3::identity();
    Mat3 extractedAccelRotation = Mat3::identity();
    SensorToDeviceAlignmentResult alignment;
    float reconstructionError = 999.0f;
};

// A legacy/full six-side accel fit can absorb a small board rotation into its
// full 3x3 correction matrix. This helper extracts the nearest proper rotation
// from that matrix, leaves scale/non-orthogonality in the native sensor frame,
// and solves the complete sensor-to-device rotation from the two raw gravity
// observations. Bias remains in the native sensor frame.
SensorToDeviceCalibrationSeparationResult separateSensorToDeviceFromAccelCalibration(
    const Vec3& topScaledSensorG,
    const Vec3& forwardScaledSensorG,
    const Vec3& accelBiasSensorG,
    const Mat3& combinedAccelScale,
    const SensorToDeviceAlignmentParams& params = SensorToDeviceAlignmentParams{}
);

struct SensorToDeviceObservationCaptureParams {
    uint32_t requiredSamples = 256;
    uint32_t resetAfterConsecutiveRejected = 8;
    float maxGyroNormDps = 2.0f;
    float minAccelNormG = 0.80f;
    float maxAccelNormG = 1.20f;
    float maxAccelVarianceNormG2 = square(0.020f);
};

struct SensorToDeviceObservationCaptureStatus {
    bool complete = false;
    uint32_t acceptedSamples = 0;
    uint32_t rejectedSamples = 0;
    uint32_t resetCount = 0;
    Vec3 meanAccelG = Vec3::zero();
    Vec3 meanSourceAccelG = Vec3::zero();
    Vec3 accelVarianceG2 = Vec3::zero();
    float meanAccelNormG = 0.0f;
};

// Small contiguous-still-window accumulator used by the guided frame stage.
// It does not drain FIFO itself; setup feeds normal production-pipeline samples
// so Wi-Fi, SlimeVR and the rest of runtime continue to be serviced.
class SensorToDeviceObservationCapture {
public:
    explicit SensorToDeviceObservationCapture(
        const SensorToDeviceObservationCaptureParams& params = SensorToDeviceObservationCaptureParams{}
    );

    void reset();
    bool push(const Lsm6dsv::Sample& sensorFrameCalibrated);
    bool push(const Lsm6dsv::Sample& sensorFrameCalibrated, const Vec3& sourceAccelG);
    bool complete() const;
    SensorToDeviceObservationCaptureStatus status() const;
    const SensorToDeviceObservationCaptureParams& params() const;

private:
    void resetWindow();

    SensorToDeviceObservationCaptureParams params_;
    StationaryStats stats_;
    Vec3 sourceAccelMeanG_ = Vec3::zero();
    uint32_t rejectedSamples_ = 0;
    uint32_t consecutiveRejected_ = 0;
    uint32_t resetCount_ = 0;
};

} // namespace tracker

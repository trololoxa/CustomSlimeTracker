#include "sensor/sensor_to_device_alignment.hpp"

#include <cmath>

#include "sensor/frame_transform.hpp"

namespace tracker {
namespace {

float angleDeg(const Vec3& a, const Vec3& b) {
    const Vec3 an = a.normalized();
    const Vec3 bn = b.normalized();
    if (an.normSq() <= MATH_EPSILON || bn.normSq() <= MATH_EPSILON) return 180.0f;
    return std::acos(clampf(dot(an, bn), -1.0f, 1.0f)) * MATH_RAD_TO_DEG;
}

bool normInRange(float n, const SensorToDeviceAlignmentParams& params) {
    return std::isfinite(n) &&
           n >= params.minObservationNormG &&
           n <= params.maxObservationNormG;
}

float matrixMaxAbsDiff(const Mat3& a, const Mat3& b) {
    float out = 0.0f;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            const float d = std::fabs(a.m[r][c] - b.m[r][c]);
            if (d > out) out = d;
        }
    }
    return out;
}

bool nearestProperRotation(const Mat3& input, Mat3& rotation) {
    if (!input.isFinite() || input.determinant() <= MATH_EPSILON) return false;
    Mat3 q = input;
    for (uint8_t i = 0; i < 12; ++i) {
        Mat3 inv;
        if (!q.inverse(inv)) return false;
        const Mat3 next = 0.5f * (q + inv.transposed());
        const float delta = matrixMaxAbsDiff(next, q);
        q = next;
        if (delta < 1.0e-6f) break;
    }
    if (!isProperRotationMatrix(q, 0.005f, 0.005f, 0.01f)) return false;
    rotation = q;
    return true;
}

} // namespace

SensorToDeviceAlignmentResult solveSensorToDeviceAlignment(
    const Vec3& topSensorG,
    const Vec3& forwardSensorG,
    const SensorToDeviceAlignmentParams& params
) {
    SensorToDeviceAlignmentResult out;
    out.topNormG = topSensorG.norm();
    out.forwardNormG = forwardSensorG.norm();

    if (!topSensorG.isFinite() || !forwardSensorG.isFinite() ||
        !normInRange(out.topNormG, params) ||
        !normInRange(out.forwardNormG, params)) {
        return out;
    }

    const Vec3 sensorZ = topSensorG / out.topNormG;
    const Vec3 forwardUnit = forwardSensorG / out.forwardNormG;
    const float observationDot = clampf(dot(sensorZ, forwardUnit), -1.0f, 1.0f);
    out.observationSeparationDeg = std::acos(observationDot) * MATH_RAD_TO_DEG;
    if (std::fabs(observationDot) > params.maxAbsObservationDot) return out;

    Vec3 sensorY = forwardUnit - sensorZ * observationDot;
    if (!sensorY.normalizeInPlace()) return out;

    Vec3 sensorX = cross(sensorY, sensorZ);
    if (!sensorX.normalizeInPlace()) return out;
    sensorY = cross(sensorZ, sensorX).normalized();
    if (sensorY.normSq() <= MATH_EPSILON) return out;

    // Rows are device unit axes expressed in the sensor frame. Multiplying a
    // sensor vector therefore returns its +X/+Y/+Z device components.
    out.sensorToDevice = Mat3::fromRows(sensorX, sensorY, sensorZ);
    out.determinant = out.sensorToDevice.determinant();
    if (!isProperRotationMatrix(out.sensorToDevice)) return out;

    out.mappedTopErrorDeg = angleDeg(out.sensorToDevice * topSensorG, Vec3::unitZ());
    out.mappedForwardErrorDeg = angleDeg(out.sensorToDevice * forwardSensorG, Vec3::unitY());
    out.valid = out.mappedTopErrorDeg <= params.maxMappedAxisErrorDeg &&
                out.mappedForwardErrorDeg <= params.maxMappedAxisErrorDeg;
    return out;
}

SensorToDeviceCalibrationSeparationResult separateSensorToDeviceFromAccelCalibration(
    const Vec3& topScaledSensorG,
    const Vec3& forwardScaledSensorG,
    const Vec3& accelBiasSensorG,
    const Mat3& combinedAccelScale,
    const SensorToDeviceAlignmentParams& params
) {
    SensorToDeviceCalibrationSeparationResult out;
    if (!topScaledSensorG.isFinite() || !forwardScaledSensorG.isFinite() ||
        !accelBiasSensorG.isFinite() || !combinedAccelScale.isFinite()) {
        return out;
    }

    if (!nearestProperRotation(combinedAccelScale, out.extractedAccelRotation)) return out;
    out.accelScaleSensorFrame = out.extractedAccelRotation.transposed() * combinedAccelScale;
    if (!out.accelScaleSensorFrame.isFinite() ||
        out.accelScaleSensorFrame.determinant() <= MATH_EPSILON) {
        return out;
    }

    const Vec3 topCorrected = out.accelScaleSensorFrame * (topScaledSensorG - accelBiasSensorG);
    const Vec3 forwardCorrected = out.accelScaleSensorFrame * (forwardScaledSensorG - accelBiasSensorG);
    out.alignment = solveSensorToDeviceAlignment(topCorrected, forwardCorrected, params);
    if (!out.alignment.valid) return out;
    out.sensorToDevice = out.alignment.sensorToDevice;

    const Mat3 reconstructed = out.extractedAccelRotation * out.accelScaleSensorFrame;
    out.reconstructionError = matrixMaxAbsDiff(reconstructed, combinedAccelScale);
    out.valid = out.reconstructionError <= 0.002f &&
                isProperRotationMatrix(out.sensorToDevice);
    return out;
}

SensorToDeviceObservationCapture::SensorToDeviceObservationCapture(
    const SensorToDeviceObservationCaptureParams& params
) : params_(params) {
    reset();
}

void SensorToDeviceObservationCapture::resetWindow() {
    stats_.reset();
    sourceAccelMeanG_ = Vec3::zero();
    consecutiveRejected_ = 0;
}

void SensorToDeviceObservationCapture::reset() {
    stats_.reset();
    sourceAccelMeanG_ = Vec3::zero();
    rejectedSamples_ = 0;
    consecutiveRejected_ = 0;
    resetCount_ = 0;
}

bool SensorToDeviceObservationCapture::push(const Lsm6dsv::Sample& sample) {
    return push(sample, sample.accel_g);
}

bool SensorToDeviceObservationCapture::push(const Lsm6dsv::Sample& sample, const Vec3& sourceAccelG) {
    if (complete()) return true;

    const float gyroNormDps = sample.gyro_rad_s.norm() * MATH_RAD_TO_DEG;
    const float accelNormG = sample.accel_g.norm();
    const bool instantValid = sample.gyro_rad_s.isFinite() && sample.accel_g.isFinite() && sourceAccelG.isFinite() &&
                              gyroNormDps <= params_.maxGyroNormDps &&
                              accelNormG >= params_.minAccelNormG &&
                              accelNormG <= params_.maxAccelNormG;
    if (!instantValid) {
        rejectedSamples_++;
        consecutiveRejected_++;
        if (stats_.count > 0 &&
            params_.resetAfterConsecutiveRejected > 0 &&
            consecutiveRejected_ >= params_.resetAfterConsecutiveRejected) {
            resetCount_++;
            resetWindow();
        }
        return false;
    }

    consecutiveRejected_ = 0;
    stats_.push(sample);
    const float n = static_cast<float>(stats_.count);
    sourceAccelMeanG_ += (sourceAccelG - sourceAccelMeanG_) / n;
    if (stats_.count >= params_.requiredSamples &&
        stats_.accelVarianceNormG2() > params_.maxAccelVarianceNormG2) {
        rejectedSamples_ += stats_.count;
        resetCount_++;
        resetWindow();
        return false;
    }
    return complete();
}

bool SensorToDeviceObservationCapture::complete() const {
    return stats_.count >= params_.requiredSamples &&
           stats_.accelVarianceNormG2() <= params_.maxAccelVarianceNormG2;
}

SensorToDeviceObservationCaptureStatus SensorToDeviceObservationCapture::status() const {
    SensorToDeviceObservationCaptureStatus out;
    out.complete = complete();
    out.acceptedSamples = stats_.count;
    out.rejectedSamples = rejectedSamples_;
    out.resetCount = resetCount_;
    out.meanAccelG = stats_.accelMeanG;
    out.meanSourceAccelG = sourceAccelMeanG_;
    out.accelVarianceG2 = stats_.accelVarianceG2();
    out.meanAccelNormG = stats_.accelNormMeanG;
    return out;
}

const SensorToDeviceObservationCaptureParams& SensorToDeviceObservationCapture::params() const {
    return params_;
}

} // namespace tracker

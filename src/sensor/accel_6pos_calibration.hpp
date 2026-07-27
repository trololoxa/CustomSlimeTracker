#pragma once

#include <cstdint>

#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"

namespace tracker {

namespace accel_cal_quality_flags {
static constexpr uint32_t OK                    = 0u;
static constexpr uint32_t MISSING_FACE          = 1u << 0;
static constexpr uint32_t TOO_FEW_SAMPLES       = 1u << 1;
static constexpr uint32_t FACE_VARIANCE_HIGH    = 1u << 2;
static constexpr uint32_t FACE_NORM_IMPLAUSIBLE = 1u << 3;
static constexpr uint32_t FACE_DIRECTION_BAD    = 1u << 4;
static constexpr uint32_t AXIS_SEPARATION_LOW   = 1u << 5;
static constexpr uint32_t BIAS_IMPLAUSIBLE      = 1u << 6;
static constexpr uint32_t SCALE_IMPLAUSIBLE     = 1u << 7;
static constexpr uint32_t NORM_RESIDUAL_HIGH    = 1u << 8;
static constexpr uint32_t AXIS_RESIDUAL_HIGH    = 1u << 9;
static constexpr uint32_t PAIR_CENTER_RESIDUAL_HIGH = 1u << 10;
static constexpr uint32_t MATRIX_SINGULAR       = 1u << 11;
static constexpr uint32_t AUTO_FACE_AMBIGUOUS    = 1u << 12;
static constexpr uint32_t INDEPENDENT_VALIDATION_FAILED = 1u << 13;
}

class Accel6PosCalibration {
public:
    enum class Face : uint8_t {
        XP = 0,
        XN = 1,
        YP = 2,
        YN = 3,
        ZP = 4,
        ZN = 5,
        Count = 6,
        Invalid = 255,
    };

    struct FaceData {
        bool valid = false;
        uint32_t samples = 0;
        Vec3 meanG = Vec3::zero();
        Vec3 varianceG2 = Vec3::zero();
        float meanNormG = 0.0f;
    };

    struct ValidationParams {
        uint32_t minSamplesPerFace = 512;
        float minFaceNormG = 0.75f;
        float maxFaceNormG = 1.25f;
        float maxFaceStddevG = 0.030f;
        float minExpectedAxisAbsG = 0.50f;
        float minAxisSeparationG = 0.75f;
        float maxAbsBiasG = 0.30f;
        float maxPairCenterResidualG = 0.080f;
        float minScale = 0.70f;
        float maxScale = 1.30f;
        float maxPostCalNormErrorG = 0.080f;
        float maxPostCalAxisResidualG = 0.220f;
    };

    struct FaceDetectionParams {
        float minNormG = 0.75f;
        float maxNormG = 1.25f;
        // Dominant raw axis must be clearly stronger than the other axes.
        // This still allows modest IMU solder/board misalignment while rejecting
        // diagonal/unstable positions that would poison a 6-position solve.
        float minDominantAbsG = 0.70f;
        float minDominanceMarginG = 0.18f;
    };

    struct FaceDetectionResult {
        bool valid = false;
        Face face = Face::Invalid;
        float normG = 0.0f;
        float dominantAbsG = 0.0f;
        float secondAbsG = 0.0f;
        float dominanceMarginG = 0.0f;
    };

    struct Result {
        bool valid = false;
        Vec3 biasG = Vec3::zero();
        // Diagonal entries of scaleMatrix kept for compact legacy prints.
        // The actual correction is the full 3x3 scaleMatrix below.
        Vec3 scale = Vec3::one();
        // Full affine accelerometer correction: calibrated = scaleMatrix * (raw - biasG).
        // Off-diagonal terms compensate cross-axis/misalignment from the six face means.
        Mat3 scaleMatrix = Mat3::identity();
        float maxFaceNormErrorG = 0.0f;
        float maxAxisResidualG = 0.0f;
        float maxPairCenterResidualG = 0.0f;
        float matrixDeterminant = 0.0f;
        float qualityScore = 0.0f;
        uint32_t qualityFlags = accel_cal_quality_flags::MISSING_FACE;
        float faceNormErrorG[6] = {};
        float faceAxisResidualG[6] = {};
    };

    void reset();

    bool setFace(Face face, const Vec3& meanG, uint32_t samples, const Vec3& varianceG2 = Vec3::zero());
    bool hasFace(Face face) const;
    bool hasAllFaces() const;
    const FaceData& faceData(Face face) const;
    const Result& result() const;
    void addQualityFlag(uint32_t flag);

    bool compute();
    bool compute(const ValidationParams& params);

    Vec3 apply(const Vec3& accelG) const;
    static Vec3 apply(const Vec3& accelG, const Result& r);

    static const char* faceName(Face face);
    static Face parseFace(const char* s);
    static const char* qualityFlagName(uint32_t flag);
    static Vec3 expectedVector(Face face);
    static FaceDetectionResult detectFace(const Vec3& meanG);
    static FaceDetectionResult detectFace(const Vec3& meanG, const FaceDetectionParams& params);

private:
    static float clamp01(float x);
    static int faceIndex(Face face);
    static char upper(char c);

    FaceData faces_[6];
    FaceData dummyFace_;
    Result result_;
};

class Accel6PosCapture {
public:
    struct Params {
        uint32_t requiredSamples = 1024;
        float maxGyroNormDps = 2.0f;
        float minAccelNormG = 0.75f;
        float maxAccelNormG = 1.25f;
        // When non-zero, a burst of rejected/moving samples after some stable
        // samples resets the running mean.  This prevents one capture from being
        // a blend of two physical sides while the user is moving the tracker.
        uint32_t resetAfterConsecutiveRejected = 8;
    };

    struct Snapshot {
        bool active = false;
        Accel6PosCalibration::Face face = Accel6PosCalibration::Face::Invalid;
        uint32_t acceptedSamples = 0;
        uint32_t rejectedSamples = 0;
        uint32_t requiredSamples = 0;
        Vec3 meanG = Vec3::zero();
        Vec3 varianceG2 = Vec3::zero();
        float meanNormG = 0.0f;
    };

    Accel6PosCapture();
    explicit Accel6PosCapture(const Params& params);

    void begin(Accel6PosCalibration::Face face);
    void cancel();

    bool active() const;
    Accel6PosCalibration::Face face() const;

    bool push(const Lsm6dsv::Sample& sample);
    bool done() const;
    bool finish(Accel6PosCalibration& cal);
    Snapshot snapshot() const;

private:
    Params params_;
    bool active_ = false;
    Accel6PosCalibration::Face face_ = Accel6PosCalibration::Face::Invalid;
    uint32_t accepted_ = 0;
    uint32_t rejected_ = 0;
    uint32_t consecutiveRejected_ = 0;
    Vec3 meanG_ = Vec3::zero();
    Vec3 m2G_ = Vec3::zero();
    float meanNormG_ = 0.0f;

    void resetAccumulation();
};

} // namespace tracker

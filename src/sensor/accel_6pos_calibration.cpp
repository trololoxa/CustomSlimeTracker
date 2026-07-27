#include "sensor/accel_6pos_calibration.hpp"

#include <cmath>

namespace tracker {

void Accel6PosCalibration::reset() {
    for (uint8_t i = 0; i < 6; ++i) {
        faces_[i] = FaceData{};
    }
    result_ = Result{};
}

bool Accel6PosCalibration::setFace(Face face, const Vec3& meanG, uint32_t samples, const Vec3& varianceG2) {
    const int idx = faceIndex(face);
    if (idx < 0 || !meanG.isFinite() || samples == 0) {
        return false;
    }

    faces_[idx].valid = true;
    faces_[idx].samples = samples;
    faces_[idx].meanG = meanG;
    faces_[idx].varianceG2 = varianceG2;
    faces_[idx].meanNormG = meanG.norm();
    result_.valid = false;
    return true;
}

bool Accel6PosCalibration::hasFace(Face face) const {
    const int idx = faceIndex(face);
    return idx >= 0 && faces_[idx].valid;
}

bool Accel6PosCalibration::hasAllFaces() const {
    for (uint8_t i = 0; i < 6; ++i) {
        if (!faces_[i].valid) return false;
    }
    return true;
}

const Accel6PosCalibration::FaceData& Accel6PosCalibration::faceData(Face face) const {
    const int idx = faceIndex(face);
    if (idx < 0) return dummyFace_;
    return faces_[idx];
}

const Accel6PosCalibration::Result& Accel6PosCalibration::result() const {
    return result_;
}

void Accel6PosCalibration::addQualityFlag(uint32_t flag) {
    result_.qualityFlags |= flag;
    result_.valid = false;
}

bool Accel6PosCalibration::compute() {
    return compute(ValidationParams{});
}

bool Accel6PosCalibration::compute(const ValidationParams& params) {
    result_ = Result{};
    result_.qualityFlags = accel_cal_quality_flags::OK;

    if (!hasAllFaces()) {
        result_.qualityFlags |= accel_cal_quality_flags::MISSING_FACE;
        return false;
    }

    for (uint8_t i = 0; i < 6; ++i) {
        const FaceData& f = faces_[i];
        if (f.samples < params.minSamplesPerFace) {
            result_.qualityFlags |= accel_cal_quality_flags::TOO_FEW_SAMPLES;
        }
        if (f.meanNormG < params.minFaceNormG || f.meanNormG > params.maxFaceNormG) {
            result_.qualityFlags |= accel_cal_quality_flags::FACE_NORM_IMPLAUSIBLE;
        }
        const float varSum = f.varianceG2.x + f.varianceG2.y + f.varianceG2.z;
        const float stddev = std::sqrt(varSum > 0.0f ? varSum : 0.0f);
        if (stddev > params.maxFaceStddevG) {
            result_.qualityFlags |= accel_cal_quality_flags::FACE_VARIANCE_HIGH;
        }

        const Vec3 expected = expectedVector(static_cast<Face>(i));
        const float expectedAxisValue = dot(f.meanG, expected);
        if (expectedAxisValue < params.minExpectedAxisAbsG) {
            result_.qualityFlags |= accel_cal_quality_flags::FACE_DIRECTION_BAD;
        }
    }

    if (result_.qualityFlags != accel_cal_quality_flags::OK) {
        return false;
    }

    const Vec3 xp = faces_[0].meanG;
    const Vec3 xn = faces_[1].meanG;
    const Vec3 yp = faces_[2].meanG;
    const Vec3 yn = faces_[3].meanG;
    const Vec3 zp = faces_[4].meanG;
    const Vec3 zn = faces_[5].meanG;

    const Vec3 centerX = 0.5f * (xp + xn);
    const Vec3 centerY = 0.5f * (yp + yn);
    const Vec3 centerZ = 0.5f * (zp + zn);
    const Vec3 rawAxisX = 0.5f * (xp - xn);
    const Vec3 rawAxisY = 0.5f * (yp - yn);
    const Vec3 rawAxisZ = 0.5f * (zp - zn);

    const float sepX = (xp - xn).norm();
    const float sepY = (yp - yn).norm();
    const float sepZ = (zp - zn).norm();
    if (sepX < params.minAxisSeparationG ||
        sepY < params.minAxisSeparationG ||
        sepZ < params.minAxisSeparationG) {
        result_.qualityFlags |= accel_cal_quality_flags::AXIS_SEPARATION_LOW;
        return false;
    }

    result_.biasG = (centerX + centerY + centerZ) / 3.0f;
    result_.maxPairCenterResidualG = 0.0f;
    const Vec3 centers[3] = { centerX, centerY, centerZ };
    for (uint8_t i = 0; i < 3; ++i) {
        const float residual = (centers[i] - result_.biasG).norm();
        if (residual > result_.maxPairCenterResidualG) {
            result_.maxPairCenterResidualG = residual;
        }
    }

    if (std::fabs(result_.biasG.x) > params.maxAbsBiasG ||
        std::fabs(result_.biasG.y) > params.maxAbsBiasG ||
        std::fabs(result_.biasG.z) > params.maxAbsBiasG) {
        result_.qualityFlags |= accel_cal_quality_flags::BIAS_IMPLAUSIBLE;
    }
    if (result_.maxPairCenterResidualG > params.maxPairCenterResidualG) {
        result_.qualityFlags |= accel_cal_quality_flags::PAIR_CENTER_RESIDUAL_HIGH;
    }

    // Full 3x3 affine model:
    //   raw_mean(face) ~= bias + rawBasis * expected_unit_vector(face)
    //   calibrated      = inverse(rawBasis) * (raw - bias)
    // Unlike the old diagonal-only model, rawBasis columns may contain
    // cross-axis terms, so the inverse corrects scale and sensor-axis
    // misalignment in one matrix.
    const Mat3 rawBasis = Mat3::fromColumns(rawAxisX, rawAxisY, rawAxisZ);
    result_.matrixDeterminant = rawBasis.determinant();
    Mat3 correction = Mat3::identity();
    if (!rawBasis.inverse(correction, 1.0e-5f) || !correction.isFinite()) {
        result_.qualityFlags |= accel_cal_quality_flags::MATRIX_SINGULAR;
        return false;
    }
    result_.scaleMatrix = correction;
    result_.scale = Vec3(
        result_.scaleMatrix.m[0][0],
        result_.scaleMatrix.m[1][1],
        result_.scaleMatrix.m[2][2]
    );

    const Vec3 rowScale(
        result_.scaleMatrix.row(0).norm(),
        result_.scaleMatrix.row(1).norm(),
        result_.scaleMatrix.row(2).norm()
    );
    if (rowScale.x < params.minScale || rowScale.x > params.maxScale ||
        rowScale.y < params.minScale || rowScale.y > params.maxScale ||
        rowScale.z < params.minScale || rowScale.z > params.maxScale) {
        result_.qualityFlags |= accel_cal_quality_flags::SCALE_IMPLAUSIBLE;
    }

    float maxNormErr = 0.0f;
    float maxAxisResidual = 0.0f;

    for (uint8_t i = 0; i < 6; ++i) {
        const Vec3 cal = apply(faces_[i].meanG, result_);
        const float normErr = std::fabs(cal.norm() - 1.0f);
        result_.faceNormErrorG[i] = normErr;
        if (normErr > maxNormErr) maxNormErr = normErr;

        const Vec3 expected = expectedVector(static_cast<Face>(i));
        const Vec3 residual = cal - expected;
        const float residualNorm = residual.norm();
        result_.faceAxisResidualG[i] = residualNorm;
        if (residualNorm > maxAxisResidual) maxAxisResidual = residualNorm;
    }

    result_.maxFaceNormErrorG = maxNormErr;
    result_.maxAxisResidualG = maxAxisResidual;

    if (maxNormErr > params.maxPostCalNormErrorG) {
        result_.qualityFlags |= accel_cal_quality_flags::NORM_RESIDUAL_HIGH;
    }
    if (maxAxisResidual > params.maxPostCalAxisResidualG) {
        result_.qualityFlags |= accel_cal_quality_flags::AXIS_RESIDUAL_HIGH;
    }

    const float normScore = 1.0f - clamp01(maxNormErr / params.maxPostCalNormErrorG);
    const float axisScore = 1.0f - clamp01(maxAxisResidual / params.maxPostCalAxisResidualG);
    const float centerScore = 1.0f - clamp01(result_.maxPairCenterResidualG / params.maxPairCenterResidualG);
    result_.qualityScore = clamp01(0.4f * normScore + 0.4f * axisScore + 0.2f * centerScore);
    result_.valid = result_.qualityFlags == accel_cal_quality_flags::OK;
    return result_.valid;
}

Vec3 Accel6PosCalibration::apply(const Vec3& accelG) const {
    if (!result_.valid) {
        return accelG;
    }
    return apply(accelG, result_);
}

Vec3 Accel6PosCalibration::apply(const Vec3& accelG, const Result& r) {
    return r.scaleMatrix * (accelG - r.biasG);
}

const char* Accel6PosCalibration::faceName(Face face) {
    switch (face) {
        case Face::XP: return "XP";
        case Face::XN: return "XN";
        case Face::YP: return "YP";
        case Face::YN: return "YN";
        case Face::ZP: return "ZP";
        case Face::ZN: return "ZN";
        default: return "INVALID";
    }
}

Accel6PosCalibration::Face Accel6PosCalibration::parseFace(const char* s) {
    if (!s) return Face::Invalid;

    const char a = upper(s[0]);
    const char b = upper(s[1]);

    if (a == 'X' && b == 'P') return Face::XP;
    if (a == 'X' && b == 'N') return Face::XN;
    if (a == 'Y' && b == 'P') return Face::YP;
    if (a == 'Y' && b == 'N') return Face::YN;
    if (a == 'Z' && b == 'P') return Face::ZP;
    if (a == 'Z' && b == 'N') return Face::ZN;
    return Face::Invalid;
}

const char* Accel6PosCalibration::qualityFlagName(uint32_t flag) {
    switch (flag) {
        case accel_cal_quality_flags::MISSING_FACE:          return "MISSING_FACE";
        case accel_cal_quality_flags::TOO_FEW_SAMPLES:       return "TOO_FEW_SAMPLES";
        case accel_cal_quality_flags::FACE_VARIANCE_HIGH:    return "FACE_VARIANCE_HIGH";
        case accel_cal_quality_flags::FACE_NORM_IMPLAUSIBLE: return "FACE_NORM_IMPLAUSIBLE";
        case accel_cal_quality_flags::FACE_DIRECTION_BAD:    return "FACE_DIRECTION_BAD";
        case accel_cal_quality_flags::AXIS_SEPARATION_LOW:   return "AXIS_SEPARATION_LOW";
        case accel_cal_quality_flags::BIAS_IMPLAUSIBLE:      return "BIAS_IMPLAUSIBLE";
        case accel_cal_quality_flags::SCALE_IMPLAUSIBLE:     return "SCALE_IMPLAUSIBLE";
        case accel_cal_quality_flags::NORM_RESIDUAL_HIGH:    return "NORM_RESIDUAL_HIGH";
        case accel_cal_quality_flags::AXIS_RESIDUAL_HIGH:    return "AXIS_RESIDUAL_HIGH";
        case accel_cal_quality_flags::PAIR_CENTER_RESIDUAL_HIGH: return "PAIR_CENTER_RESIDUAL_HIGH";
        case accel_cal_quality_flags::MATRIX_SINGULAR:       return "MATRIX_SINGULAR";
        case accel_cal_quality_flags::AUTO_FACE_AMBIGUOUS:   return "AUTO_FACE_AMBIGUOUS";
        case accel_cal_quality_flags::INDEPENDENT_VALIDATION_FAILED:
            return "INDEPENDENT_VALIDATION_FAILED";
    }
    return "UNKNOWN";
}

Vec3 Accel6PosCalibration::expectedVector(Face face) {
    switch (face) {
        case Face::XP: return Vec3( 1.0f,  0.0f,  0.0f);
        case Face::XN: return Vec3(-1.0f,  0.0f,  0.0f);
        case Face::YP: return Vec3( 0.0f,  1.0f,  0.0f);
        case Face::YN: return Vec3( 0.0f, -1.0f,  0.0f);
        case Face::ZP: return Vec3( 0.0f,  0.0f,  1.0f);
        case Face::ZN: return Vec3( 0.0f,  0.0f, -1.0f);
        default:       return Vec3::zero();
    }
}

Accel6PosCalibration::FaceDetectionResult Accel6PosCalibration::detectFace(const Vec3& meanG) {
    return detectFace(meanG, FaceDetectionParams{});
}

Accel6PosCalibration::FaceDetectionResult Accel6PosCalibration::detectFace(
    const Vec3& meanG,
    const FaceDetectionParams& params
) {
    FaceDetectionResult r;
    r.normG = meanG.norm();
    if (!meanG.isFinite() ||
        r.normG < params.minNormG ||
        r.normG > params.maxNormG) {
        return r;
    }

    const float ax[3] = { std::fabs(meanG.x), std::fabs(meanG.y), std::fabs(meanG.z) };
    uint8_t dominant = 0;
    uint8_t second = 1;
    if (ax[1] > ax[dominant]) { dominant = 1; second = 0; }
    if (ax[2] > ax[dominant]) { second = dominant; dominant = 2; }
    else if (ax[2] > ax[second]) { second = 2; }

    r.dominantAbsG = ax[dominant];
    r.secondAbsG = ax[second];
    r.dominanceMarginG = r.dominantAbsG - r.secondAbsG;
    if (r.dominantAbsG < params.minDominantAbsG ||
        r.dominanceMarginG < params.minDominanceMarginG) {
        return r;
    }

    if (dominant == 0) r.face = meanG.x >= 0.0f ? Face::XP : Face::XN;
    else if (dominant == 1) r.face = meanG.y >= 0.0f ? Face::YP : Face::YN;
    else r.face = meanG.z >= 0.0f ? Face::ZP : Face::ZN;
    r.valid = true;
    return r;
}

float Accel6PosCalibration::clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

int Accel6PosCalibration::faceIndex(Face face) {
    const uint8_t idx = static_cast<uint8_t>(face);
    if (idx >= 6) return -1;
    return static_cast<int>(idx);
}

char Accel6PosCalibration::upper(char c) {
    if (c >= 'a' && c <= 'z') return static_cast<char>(c - 'a' + 'A');
    if (c == '+') return 'P';
    if (c == '-') return 'N';
    return c;
}

Accel6PosCapture::Accel6PosCapture() {
    Params params;
    params_ = params;
}

Accel6PosCapture::Accel6PosCapture(const Params& params) : params_(params) {}

void Accel6PosCapture::begin(Accel6PosCalibration::Face face) {
    active_ = true;
    face_ = face;
    rejected_ = 0;
    resetAccumulation();
}

void Accel6PosCapture::cancel() {
    active_ = false;
    face_ = Accel6PosCalibration::Face::Invalid;
}

bool Accel6PosCapture::active() const {
    return active_;
}

Accel6PosCalibration::Face Accel6PosCapture::face() const {
    return face_;
}

void Accel6PosCapture::resetAccumulation() {
    accepted_ = 0;
    consecutiveRejected_ = 0;
    meanG_ = Vec3::zero();
    m2G_ = Vec3::zero();
    meanNormG_ = 0.0f;
}

bool Accel6PosCapture::push(const Lsm6dsv::Sample& sample) {
    if (!active_) {
        return false;
    }

    const float gyroNormDps = sample.gyro_rad_s.norm() * MATH_RAD_TO_DEG;
    const float accelNormG = sample.accel_g.norm();

    const bool accepted = sample.accel_g.isFinite() && sample.gyro_rad_s.isFinite() &&
                          gyroNormDps <= params_.maxGyroNormDps &&
                          accelNormG >= params_.minAccelNormG &&
                          accelNormG <= params_.maxAccelNormG;

    if (!accepted) {
        rejected_++;
        consecutiveRejected_++;
        if (params_.resetAfterConsecutiveRejected > 0 &&
            accepted_ > 0 &&
            consecutiveRejected_ >= params_.resetAfterConsecutiveRejected) {
            resetAccumulation();
        }
        return false;
    }

    consecutiveRejected_ = 0;
    accepted_++;
    const float n = static_cast<float>(accepted_);

    const Vec3 delta = sample.accel_g - meanG_;
    meanG_ += delta / n;
    const Vec3 delta2 = sample.accel_g - meanG_;
    m2G_ += hadamard(delta, delta2);
    meanNormG_ += (accelNormG - meanNormG_) / n;

    return done();
}

bool Accel6PosCapture::done() const {
    return active_ && accepted_ >= params_.requiredSamples;
}

bool Accel6PosCapture::finish(Accel6PosCalibration& cal) {
    if (!done()) {
        return false;
    }

    Vec3 var = Vec3::zero();
    if (accepted_ >= 2) {
        var = m2G_ / static_cast<float>(accepted_ - 1);
    }

    const bool ok = cal.setFace(face_, meanG_, accepted_, var);
    cancel();
    return ok;
}

Accel6PosCapture::Snapshot Accel6PosCapture::snapshot() const {
    Snapshot s;
    s.active = active_;
    s.face = face_;
    s.acceptedSamples = accepted_;
    s.rejectedSamples = rejected_;
    s.requiredSamples = params_.requiredSamples;
    s.meanG = meanG_;
    s.meanNormG = meanNormG_;
    if (accepted_ >= 2) {
        s.varianceG2 = m2G_ / static_cast<float>(accepted_ - 1);
    }
    return s;
}

} // namespace tracker

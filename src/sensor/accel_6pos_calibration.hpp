#pragma once

#include <cstdint>
#include <cmath>

#include "core/math.hpp"
#include "connection/lsm6dsv_driver.hpp"

namespace tracker {

// ============================================================
// 6-position accelerometer calibration
// ============================================================
// This first version estimates diagonal bias + scale:
//   accel_cal.x = scale.x * (accel_g.x - bias.x)
//   accel_cal.y = scale.y * (accel_g.y - bias.y)
//   accel_cal.z = scale.z * (accel_g.z - bias.z)
//
// It does not estimate full 3x3 cross-axis/misalignment yet.
// That can be added later with ellipsoid or least-squares calibration.
//
// Face naming convention:
//   XP means sensor accel.x should be positive and close to +1g.
//   XN means sensor accel.x should be negative and close to -1g.
//   Same for Y/Z.
// ============================================================

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

    struct Result {
        bool valid = false;
        Vec3 biasG = Vec3::zero();
        Vec3 scale = Vec3::one();
        Mat3 scaleMatrix = Mat3::identity();
        float maxFaceNormErrorG = 0.0f;
        float maxAxisResidualG = 0.0f;
    };

    void reset() {
        for (uint8_t i = 0; i < 6; ++i) {
            faces_[i] = FaceData{};
        }
        result_ = Result{};
    }

    bool setFace(Face face, const Vec3& meanG, uint32_t samples, const Vec3& varianceG2 = Vec3::zero()) {
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

    bool hasFace(Face face) const {
        const int idx = faceIndex(face);
        return idx >= 0 && faces_[idx].valid;
    }

    bool hasAllFaces() const {
        for (uint8_t i = 0; i < 6; ++i) {
            if (!faces_[i].valid) return false;
        }
        return true;
    }

    const FaceData& faceData(Face face) const {
        const int idx = faceIndex(face);
        if (idx < 0) return dummyFace_;
        return faces_[idx];
    }

    const Result& result() const {
        return result_;
    }

    bool compute() {
        result_ = Result{};
        if (!hasAllFaces()) {
            return false;
        }

        const Vec3 xp = faces_[0].meanG;
        const Vec3 xn = faces_[1].meanG;
        const Vec3 yp = faces_[2].meanG;
        const Vec3 yn = faces_[3].meanG;
        const Vec3 zp = faces_[4].meanG;
        const Vec3 zn = faces_[5].meanG;

        const float dx = xp.x - xn.x;
        const float dy = yp.y - yn.y;
        const float dz = zp.z - zn.z;

        if (std::fabs(dx) < 0.5f || std::fabs(dy) < 0.5f || std::fabs(dz) < 0.5f) {
            return false;
        }

        result_.biasG = Vec3(
            0.5f * (xp.x + xn.x),
            0.5f * (yp.y + yn.y),
            0.5f * (zp.z + zn.z)
        );

        result_.scale = Vec3(
            2.0f / dx,
            2.0f / dy,
            2.0f / dz
        );

        result_.scaleMatrix = Mat3::diagonal(result_.scale.x, result_.scale.y, result_.scale.z);

        float maxNormErr = 0.0f;
        float maxAxisResidual = 0.0f;

        for (uint8_t i = 0; i < 6; ++i) {
            const Vec3 cal = apply(faces_[i].meanG, result_);
            const float normErr = std::fabs(cal.norm() - 1.0f);
            if (normErr > maxNormErr) maxNormErr = normErr;

            const Vec3 expected = expectedVector(static_cast<Face>(i));
            const Vec3 residual = cal - expected;
            const float residualNorm = residual.norm();
            if (residualNorm > maxAxisResidual) maxAxisResidual = residualNorm;
        }

        result_.maxFaceNormErrorG = maxNormErr;
        result_.maxAxisResidualG = maxAxisResidual;
        result_.valid = true;
        return true;
    }

    Vec3 apply(const Vec3& accelG) const {
        if (!result_.valid) {
            return accelG;
        }
        return apply(accelG, result_);
    }

    static Vec3 apply(const Vec3& accelG, const Result& r) {
        return r.scaleMatrix * (accelG - r.biasG);
    }

    static const char* faceName(Face face) {
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

    static Face parseFace(const char* s) {
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

    static Vec3 expectedVector(Face face) {
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

private:
    static int faceIndex(Face face) {
        const uint8_t idx = static_cast<uint8_t>(face);
        if (idx >= 6) return -1;
        return static_cast<int>(idx);
    }

    static char upper(char c) {
        if (c >= 'a' && c <= 'z') return static_cast<char>(c - 'a' + 'A');
        if (c == '+') return 'P';
        if (c == '-') return 'N';
        return c;
    }

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

    Accel6PosCapture() {
        Params params;
        params_ = params;
    }

    explicit Accel6PosCapture(const Params& params) : params_(params) {}

    void begin(Accel6PosCalibration::Face face) {
        active_ = true;
        face_ = face;
        accepted_ = 0;
        rejected_ = 0;
        meanG_ = Vec3::zero();
        m2G_ = Vec3::zero();
        meanNormG_ = 0.0f;
    }

    void cancel() {
        active_ = false;
        face_ = Accel6PosCalibration::Face::Invalid;
    }

    bool active() const {
        return active_;
    }

    Accel6PosCalibration::Face face() const {
        return face_;
    }

    bool push(const Lsm6dsv::Sample& sample) {
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
            return false;
        }

        accepted_++;
        const float n = static_cast<float>(accepted_);

        const Vec3 delta = sample.accel_g - meanG_;
        meanG_ += delta / n;
        const Vec3 delta2 = sample.accel_g - meanG_;
        m2G_ += hadamard(delta, delta2);
        meanNormG_ += (accelNormG - meanNormG_) / n;

        return done();
    }

    bool done() const {
        return active_ && accepted_ >= params_.requiredSamples;
    }

    bool finish(Accel6PosCalibration& cal) {
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

    Snapshot snapshot() const {
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

private:
    Params params_;
    bool active_ = false;
    Accel6PosCalibration::Face face_ = Accel6PosCalibration::Face::Invalid;
    uint32_t accepted_ = 0;
    uint32_t rejected_ = 0;
    Vec3 meanG_ = Vec3::zero();
    Vec3 m2G_ = Vec3::zero();
    float meanNormG_ = 0.0f;
};

} // namespace tracker

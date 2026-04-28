#pragma once

#include <cmath>
#include <cstdint>

namespace tracker {

// ============================================================
// Math conventions
// ============================================================
// Quaternion layout: [w, x, y, z]
// Quaternion product: Hamilton product
// Rotation usage: q.rotate(v) = q * [0,v] * q.conjugate()
//
// Recommended AHRS convention later:
//   q_world_from_sensor rotates vectors from sensor frame to world frame.
//   Gyro is measured in sensor/body frame, rad/s.
//   For q_world_from_sensor propagation:
//     q_next = q * dq_body
//
// All internal angles are radians.
// All gyro values should be rad/s before entering quaternion integration.
// ============================================================

constexpr float MATH_EPSILON = 1.0e-6f;
constexpr float MATH_PI      = 3.14159265358979323846f;
constexpr float MATH_TWO_PI  = 6.28318530717958647692f;
constexpr float MATH_DEG_TO_RAD = MATH_PI / 180.0f;
constexpr float MATH_RAD_TO_DEG = 180.0f / MATH_PI;

inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

inline float square(float x) {
    return x * x;
}

inline bool isFinite(float x) {
    return std::isfinite(x);
}

inline float wrapPi(float a) {
    while (a > MATH_PI)  a -= MATH_TWO_PI;
    while (a < -MATH_PI) a += MATH_TWO_PI;
    return a;
}

// ============================================================
// Vec3
// ============================================================

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    static constexpr Vec3 zero() { return Vec3(0.0f, 0.0f, 0.0f); }
    static constexpr Vec3 one()  { return Vec3(1.0f, 1.0f, 1.0f); }
    static constexpr Vec3 unitX(){ return Vec3(1.0f, 0.0f, 0.0f); }
    static constexpr Vec3 unitY(){ return Vec3(0.0f, 1.0f, 0.0f); }
    static constexpr Vec3 unitZ(){ return Vec3(0.0f, 0.0f, 1.0f); }

    Vec3 operator+() const { return *this; }
    Vec3 operator-() const { return Vec3(-x, -y, -z); }

    Vec3 operator+(const Vec3& b) const { return Vec3(x + b.x, y + b.y, z + b.z); }
    Vec3 operator-(const Vec3& b) const { return Vec3(x - b.x, y - b.y, z - b.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(float s) const { return Vec3(x / s, y / s, z / s); }

    Vec3& operator+=(const Vec3& b) {
        x += b.x; y += b.y; z += b.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& b) {
        x -= b.x; y -= b.y; z -= b.z;
        return *this;
    }

    Vec3& operator*=(float s) {
        x *= s; y *= s; z *= s;
        return *this;
    }

    Vec3& operator/=(float s) {
        x /= s; y /= s; z /= s;
        return *this;
    }

    float normSq() const {
        return x * x + y * y + z * z;
    }

    float norm() const {
        return std::sqrt(normSq());
    }

    bool isFinite() const {
        return tracker::isFinite(x) && tracker::isFinite(y) && tracker::isFinite(z);
    }

    Vec3 normalized(float eps = MATH_EPSILON) const {
        const float n = norm();
        if (n < eps || !tracker::isFinite(n)) {
            return Vec3::zero();
        }
        return *this / n;
    }

    bool normalizeInPlace(float eps = MATH_EPSILON) {
        const float n = norm();
        if (n < eps || !tracker::isFinite(n)) {
            x = y = z = 0.0f;
            return false;
        }
        *this /= n;
        return true;
    }
};

inline Vec3 operator*(float s, const Vec3& v) {
    return v * s;
}

inline float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

inline Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
    return a + (b - a) * t;
}

inline Vec3 hadamard(const Vec3& a, const Vec3& b) {
    return Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

// ============================================================
// Mat3
// Row-major 3x3 matrix.
// m[row][col]
// ============================================================

struct Mat3 {
    float m[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    constexpr Mat3() = default;

    constexpr Mat3(
        float m00, float m01, float m02,
        float m10, float m11, float m12,
        float m20, float m21, float m22
    ) : m{{m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22}} {}

    static constexpr Mat3 identity() {
        return Mat3(
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f
        );
    }

    static constexpr Mat3 zero() {
        return Mat3(
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f
        );
    }

    static constexpr Mat3 diagonal(float x, float y, float z) {
        return Mat3(
            x,    0.0f, 0.0f,
            0.0f, y,    0.0f,
            0.0f, 0.0f, z
        );
    }

    static constexpr Mat3 fromColumns(const Vec3& c0, const Vec3& c1, const Vec3& c2) {
        return Mat3(
            c0.x, c1.x, c2.x,
            c0.y, c1.y, c2.y,
            c0.z, c1.z, c2.z
        );
    }

    static constexpr Mat3 fromRows(const Vec3& r0, const Vec3& r1, const Vec3& r2) {
        return Mat3(
            r0.x, r0.y, r0.z,
            r1.x, r1.y, r1.z,
            r2.x, r2.y, r2.z
        );
    }

    Vec3 row(int i) const {
        return Vec3(m[i][0], m[i][1], m[i][2]);
    }

    Vec3 col(int i) const {
        return Vec3(m[0][i], m[1][i], m[2][i]);
    }

    Mat3 operator+(const Mat3& b) const {
        Mat3 r = Mat3::zero();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r.m[i][j] = m[i][j] + b.m[i][j];
            }
        }
        return r;
    }

    Mat3 operator-(const Mat3& b) const {
        Mat3 r = Mat3::zero();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r.m[i][j] = m[i][j] - b.m[i][j];
            }
        }
        return r;
    }

    Mat3 operator*(float s) const {
        Mat3 r = Mat3::zero();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r.m[i][j] = m[i][j] * s;
            }
        }
        return r;
    }

    Vec3 operator*(const Vec3& v) const {
        return Vec3(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        );
    }

    Mat3 operator*(const Mat3& b) const {
        Mat3 r = Mat3::zero();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r.m[i][j] =
                    m[i][0] * b.m[0][j] +
                    m[i][1] * b.m[1][j] +
                    m[i][2] * b.m[2][j];
            }
        }
        return r;
    }

    Mat3 transposed() const {
        return Mat3(
            m[0][0], m[1][0], m[2][0],
            m[0][1], m[1][1], m[2][1],
            m[0][2], m[1][2], m[2][2]
        );
    }

    float determinant() const {
        return
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
            m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
            m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    bool inverse(Mat3& out, float eps = MATH_EPSILON) const {
        const float det = determinant();
        if (std::fabs(det) < eps || !tracker::isFinite(det)) {
            out = Mat3::identity();
            return false;
        }

        const float invDet = 1.0f / det;

        out = Mat3(
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet,
           -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * invDet,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet,

           -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * invDet,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet,
           -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * invDet,

            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet,
           -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * invDet,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet
        );

        return true;
    }

    bool isFinite() const {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (!tracker::isFinite(m[i][j])) {
                    return false;
                }
            }
        }
        return true;
    }
};

inline Mat3 operator*(float s, const Mat3& a) {
    return a * s;
}

inline Mat3 skewSymmetric(const Vec3& v) {
    return Mat3(
         0.0f, -v.z,   v.y,
         v.z,   0.0f, -v.x,
        -v.y,   v.x,   0.0f
    );
}

// ============================================================
// Quaternion
// ============================================================

struct Quat {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Quat() = default;
    constexpr Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    static constexpr Quat identity() {
        return Quat(1.0f, 0.0f, 0.0f, 0.0f);
    }

    static Quat fromAxisAngle(const Vec3& axisUnit, float angleRad) {
        const float half = 0.5f * angleRad;
        const float s = std::sin(half);
        return Quat(std::cos(half), axisUnit.x * s, axisUnit.y * s, axisUnit.z * s);
    }

    static Quat fromRotationVector(const Vec3& rotVecRad) {
        const float angle = rotVecRad.norm();

        if (angle < 1.0e-6f) {
            // First-order approximation:
            // q ≈ [1, 0.5*rx, 0.5*ry, 0.5*rz]
            return Quat(
                1.0f,
                0.5f * rotVecRad.x,
                0.5f * rotVecRad.y,
                0.5f * rotVecRad.z
            ).normalized();
        }

        const Vec3 axis = rotVecRad / angle;
        return fromAxisAngle(axis, angle);
    }

    static Quat fromTwoUnitVectors(const Vec3& fromUnit, const Vec3& toUnit) {
        // Returns q such that q.rotate(fromUnit) ≈ toUnit.
        // Both inputs should already be normalized.
        const float c = clampf(dot(fromUnit, toUnit), -1.0f, 1.0f);

        if (c > 1.0f - 1.0e-6f) {
            return Quat::identity();
        }

        if (c < -1.0f + 1.0e-6f) {
            // 180 degree turn. Pick any stable orthogonal axis.
            Vec3 axis = cross(Vec3::unitX(), fromUnit);
            if (axis.normSq() < 1.0e-6f) {
                axis = cross(Vec3::unitY(), fromUnit);
            }
            axis.normalizeInPlace();
            return Quat::fromAxisAngle(axis, MATH_PI);
        }

        const Vec3 axis = cross(fromUnit, toUnit);
        const float s = std::sqrt((1.0f + c) * 2.0f);
        const float invS = 1.0f / s;

        return Quat(
            0.5f * s,
            axis.x * invS,
            axis.y * invS,
            axis.z * invS
        ).normalized();
    }

    static Quat fromTwoVectors(const Vec3& from, const Vec3& to) {
        const Vec3 f = from.normalized();
        const Vec3 t = to.normalized();
        if (f.normSq() < MATH_EPSILON || t.normSq() < MATH_EPSILON) {
            return Quat::identity();
        }
        return fromTwoUnitVectors(f, t);
    }

    static Quat fromEulerXYZ(float rollRad, float pitchRad, float yawRad) {
        // Debug helper.
        // Intrinsic XYZ / equivalent composition around local axes:
        // q = qz(yaw) * qy(pitch) * qx(roll)
        const float cr = std::cos(0.5f * rollRad);
        const float sr = std::sin(0.5f * rollRad);
        const float cp = std::cos(0.5f * pitchRad);
        const float sp = std::sin(0.5f * pitchRad);
        const float cy = std::cos(0.5f * yawRad);
        const float sy = std::sin(0.5f * yawRad);

        return Quat(
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            sy * cp * sr + cy * sp * cr,
            sy * cp * cr - cy * sp * sr
        ).normalized();
    }

    Quat operator+() const { return *this; }
    Quat operator-() const { return Quat(-w, -x, -y, -z); }

    Quat operator+(const Quat& b) const {
        return Quat(w + b.w, x + b.x, y + b.y, z + b.z);
    }

    Quat operator-(const Quat& b) const {
        return Quat(w - b.w, x - b.x, y - b.y, z - b.z);
    }

    Quat operator*(float s) const {
        return Quat(w * s, x * s, y * s, z * s);
    }

    Quat operator/(float s) const {
        return Quat(w / s, x / s, y / s, z / s);
    }

    Quat operator*(const Quat& b) const {
        return Quat(
            w * b.w - x * b.x - y * b.y - z * b.z,
            w * b.x + x * b.w + y * b.z - z * b.y,
            w * b.y - x * b.z + y * b.w + z * b.x,
            w * b.z + x * b.y - y * b.x + z * b.w
        );
    }

    Quat& operator+=(const Quat& b) {
        w += b.w; x += b.x; y += b.y; z += b.z;
        return *this;
    }

    Quat& operator*=(const Quat& b) {
        *this = *this * b;
        return *this;
    }

    Quat& operator*=(float s) {
        w *= s; x *= s; y *= s; z *= s;
        return *this;
    }

    float normSq() const {
        return w * w + x * x + y * y + z * z;
    }

    float norm() const {
        return std::sqrt(normSq());
    }

    bool isFinite() const {
        return tracker::isFinite(w) && tracker::isFinite(x) && tracker::isFinite(y) && tracker::isFinite(z);
    }

    Quat normalized(float eps = MATH_EPSILON) const {
        const float n = norm();
        if (n < eps || !tracker::isFinite(n)) {
            return Quat::identity();
        }
        return *this / n;
    }

    bool normalizeInPlace(float eps = MATH_EPSILON) {
        const float n = norm();
        if (n < eps || !tracker::isFinite(n)) {
            *this = Quat::identity();
            return false;
        }
        *this = *this / n;
        return true;
    }

    Quat conjugated() const {
        return Quat(w, -x, -y, -z);
    }

    Quat inversed(float eps = MATH_EPSILON) const {
        const float n2 = normSq();
        if (n2 < eps || !tracker::isFinite(n2)) {
            return Quat::identity();
        }
        return conjugated() / n2;
    }

    Vec3 vectorPart() const {
        return Vec3(x, y, z);
    }

    Vec3 rotate(const Vec3& v) const {
        // Optimized active rotation: q * v * q^-1, assuming q is unit.
        const Vec3 qv(x, y, z);
        const Vec3 t = 2.0f * cross(qv, v);
        return v + w * t + cross(qv, t);
    }

    Vec3 inverseRotate(const Vec3& v) const {
        // Rotate by q^-1. For unit q, q^-1 = conjugate(q).
        return conjugated().rotate(v);
    }

    Mat3 toRotationMatrix() const {
        const Quat q = normalized();

        const float xx = q.x * q.x;
        const float yy = q.y * q.y;
        const float zz = q.z * q.z;
        const float xy = q.x * q.y;
        const float xz = q.x * q.z;
        const float yz = q.y * q.z;
        const float wx = q.w * q.x;
        const float wy = q.w * q.y;
        const float wz = q.w * q.z;

        return Mat3(
            1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz),        2.0f * (xz + wy),
            2.0f * (xy + wz),        1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx),
            2.0f * (xz - wy),        2.0f * (yz + wx),        1.0f - 2.0f * (xx + yy)
        );
    }

    Vec3 toEulerXYZ() const {
        // Debug helper.
        // Returns roll, pitch, yaw in radians for the same convention as fromEulerXYZ.
        const Quat q = normalized();

        const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
        const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
        const float roll = std::atan2(sinr_cosp, cosr_cosp);

        const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
        float pitch;
        if (std::fabs(sinp) >= 1.0f) {
            pitch = std::copysign(MATH_PI * 0.5f, sinp);
        } else {
            pitch = std::asin(sinp);
        }

        const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
        const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        const float yaw = std::atan2(siny_cosp, cosy_cosp);

        return Vec3(roll, pitch, yaw);
    }

    Quat withPositiveW() const {
        // q and -q represent the same orientation.
        // This keeps packet output visually continuous in many cases.
        return w < 0.0f ? -(*this) : *this;
    }
};

inline Quat operator*(float s, const Quat& q) {
    return q * s;
}

// ============================================================
// Gyro integration helpers
// ============================================================

inline Quat integrateBodyRate(const Quat& q_world_from_body,
                              const Vec3& gyro_body_rad_s,
                              float dt_s) {
    // For q_world_from_body and body-frame gyro:
    //   q_next = q * dq_body
    const Vec3 delta = gyro_body_rad_s * dt_s;
    const Quat dq = Quat::fromRotationVector(delta);
    return (q_world_from_body * dq).normalized();
}

inline Quat integrateWorldRate(const Quat& q_world_from_body,
                               const Vec3& gyro_world_rad_s,
                               float dt_s) {
    // For world-frame angular velocity:
    //   q_next = dq_world * q
    const Vec3 delta = gyro_world_rad_s * dt_s;
    const Quat dq = Quat::fromRotationVector(delta);
    return (dq * q_world_from_body).normalized();
}

// Applies a small correction vector in world frame:
//   q_next = dq_world * q
inline Quat applyWorldCorrection(const Quat& q_world_from_body,
                                 const Vec3& correction_world_rad) {
    const Quat dq = Quat::fromRotationVector(correction_world_rad);
    return (dq * q_world_from_body).normalized();
}

// Applies a small correction vector in body/sensor frame:
//   q_next = q * dq_body
inline Quat applyBodyCorrection(const Quat& q_world_from_body,
                                const Vec3& correction_body_rad) {
    const Quat dq = Quat::fromRotationVector(correction_body_rad);
    return (q_world_from_body * dq).normalized();
}

// ============================================================
// Small helpers for AHRS/debug later
// ============================================================

inline float angleBetweenUnitVectors(const Vec3& aUnit, const Vec3& bUnit) {
    return std::acos(clampf(dot(aUnit, bUnit), -1.0f, 1.0f));
}

inline float angleBetweenVectors(const Vec3& a, const Vec3& b) {
    const Vec3 an = a.normalized();
    const Vec3 bn = b.normalized();
    if (an.normSq() < MATH_EPSILON || bn.normSq() < MATH_EPSILON) {
        return 0.0f;
    }
    return angleBetweenUnitVectors(an, bn);
}

inline Vec3 projectOntoPlane(const Vec3& v, const Vec3& planeNormalUnit) {
    return v - planeNormalUnit * dot(v, planeNormalUnit);
}

inline bool nearlyEqual(float a, float b, float eps = 1.0e-5f) {
    return std::fabs(a - b) <= eps;
}

inline bool nearlyEqual(const Vec3& a, const Vec3& b, float eps = 1.0e-5f) {
    return nearlyEqual(a.x, b.x, eps) &&
           nearlyEqual(a.y, b.y, eps) &&
           nearlyEqual(a.z, b.z, eps);
}

inline bool nearlyEqual(const Quat& a, const Quat& b, float eps = 1.0e-5f) {
    // q and -q are equivalent rotations.
    const bool direct =
        nearlyEqual(a.w, b.w, eps) &&
        nearlyEqual(a.x, b.x, eps) &&
        nearlyEqual(a.y, b.y, eps) &&
        nearlyEqual(a.z, b.z, eps);

    const bool negated =
        nearlyEqual(a.w, -b.w, eps) &&
        nearlyEqual(a.x, -b.x, eps) &&
        nearlyEqual(a.y, -b.y, eps) &&
        nearlyEqual(a.z, -b.z, eps);

    return direct || negated;
}

} // namespace tracker

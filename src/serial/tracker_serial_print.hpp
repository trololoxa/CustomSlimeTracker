#pragma once

#include <Arduino.h>
#include <cstdint>

#include "core/math.hpp"

namespace tracker {

namespace tracker_serial_detail {

inline void printU64Dec(Stream& out, uint64_t v) {
    char buf[21];
    size_t i = sizeof(buf);
    buf[--i] = '\0';
    if (v == 0) {
        buf[--i] = '0';
    } else {
        while (v > 0 && i > 0) {
            buf[--i] = static_cast<char>('0' + (v % 10));
            v /= 10;
        }
    }
    out.print(&buf[i]);
}

inline void printVec3(Stream& out, const char* label, const Vec3& v, uint8_t decimals = 6) {
    out.print(label);
    out.print(" x="); out.print(v.x, decimals);
    out.print(" y="); out.print(v.y, decimals);
    out.print(" z="); out.print(v.z, decimals);
}

inline void printVec3Line(Stream& out, const char* label, const Vec3& v, uint8_t decimals = 6) {
    printVec3(out, label, v, decimals);
    out.println();
}

inline void printQuatLine(Stream& out, const char* label, const Quat& q, uint8_t decimals = 7) {
    out.print(label);
    out.print(" w="); out.print(q.w, decimals);
    out.print(" x="); out.print(q.x, decimals);
    out.print(" y="); out.print(q.y, decimals);
    out.print(" z="); out.println(q.z, decimals);
}

inline void printOk(Stream& out, const char* msg = nullptr) {
    out.print("# OK");
    if (msg && *msg) {
        out.print(' ');
        out.print(msg);
    }
    out.println();
}

inline void printErr(Stream& out, const char* msg) {
    out.print("# ERR ");
    out.println(msg ? msg : "unknown");
}

} // namespace tracker_serial_detail

} // namespace tracker

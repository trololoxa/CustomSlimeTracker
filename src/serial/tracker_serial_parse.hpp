#pragma once

#include <cstdint>
#include <cstdlib>
#include <cmath>

namespace tracker {

namespace tracker_serial_detail {

inline char upperChar(char c) {
    if (c >= 'a' && c <= 'z') return static_cast<char>(c - 'a' + 'A');
    return c;
}

inline bool eqIgnoreCase(const char* a, const char* b) {
    if (!a || !b) return false;
    while (*a && *b) {
        if (upperChar(*a) != upperChar(*b)) return false;
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

inline bool startsWithIgnoreCase(const char* s, const char* prefix) {
    if (!s || !prefix) return false;
    while (*prefix) {
        if (upperChar(*s) != upperChar(*prefix)) return false;
        ++s;
        ++prefix;
    }
    return true;
}

inline bool parseBool(const char* s, bool& out) {
    if (!s) return false;
    if (eqIgnoreCase(s, "1") || eqIgnoreCase(s, "on") || eqIgnoreCase(s, "true") || eqIgnoreCase(s, "yes")) {
        out = true;
        return true;
    }
    if (eqIgnoreCase(s, "0") || eqIgnoreCase(s, "off") || eqIgnoreCase(s, "false") || eqIgnoreCase(s, "no")) {
        out = false;
        return true;
    }
    return false;
}

inline bool parseU32(const char* s, uint32_t& out) {
    if (!s || *s == '\0') return false;
    char* end = nullptr;
    const unsigned long v = std::strtoul(s, &end, 0);
    if (!end || *end != '\0') return false;
    out = static_cast<uint32_t>(v);
    return true;
}

inline bool parseFloat(const char* s, float& out) {
    if (!s || *s == '\0') return false;
    char* end = nullptr;
    const float v = std::strtof(s, &end);
    if (!end || *end != '\0' || !std::isfinite(v)) return false;
    out = v;
    return true;
}


} // namespace tracker_serial_detail

} // namespace tracker

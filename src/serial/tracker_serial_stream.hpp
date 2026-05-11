#pragma once

#include <Arduino.h>
#include <cstdint>

#include "connection/lsm6dsv_driver.hpp"
#include "serial/tracker_serial_context.hpp"

namespace tracker {

// ============================================================
// Optional stream emit helpers
// ============================================================

inline bool trackerSerialStreamDue(TrackerSerialStreamState& st, uint32_t nowUs) {
    if (st.mode == TrackerStreamMode::Off || st.mode == TrackerStreamMode::Heartbeat) return false;
    const uint32_t period = st.periodUs();
    if (st.lastEmitUs == 0 || nowUs - st.lastEmitUs >= period) {
        st.lastEmitUs = nowUs;
        return true;
    }
    return false;
}

inline void trackerSerialEmitQuat(Stream& out,
                                  uint64_t tUs,
                                  const Quat& q,
                                  uint32_t qualityFlags,
                                  float confidence) {
    out.print("Q,");
    tracker_serial_detail::printU64Dec(out, tUs);
    out.print(','); out.print(q.w, 7);
    out.print(','); out.print(q.x, 7);
    out.print(','); out.print(q.y, 7);
    out.print(','); out.print(q.z, 7);
    out.print(",0x"); out.print(qualityFlags, HEX);
    out.print(','); out.println(confidence, 4);
}

inline void trackerSerialEmitRaw(Stream& out,
                                 const Lsm6dsv::RawSample& raw,
                                 uint32_t qualityFlags) {
    out.print("RAW,");
    tracker_serial_detail::printU64Dec(out, raw.t_us);
    out.print(','); out.print(raw.ax);
    out.print(','); out.print(raw.ay);
    out.print(','); out.print(raw.az);
    out.print(','); out.print(raw.gx);
    out.print(','); out.print(raw.gy);
    out.print(','); out.print(raw.gz);
    out.print(",0x"); out.println(qualityFlags, HEX);
}

inline void trackerSerialEmitScaled(Stream& out,
                                    uint64_t tUs,
                                    const Lsm6dsv::Sample& s,
                                    uint32_t qualityFlags) {
    out.print("S,");
    tracker_serial_detail::printU64Dec(out, tUs);
    out.print(','); out.print(s.accel_g.x, 6);
    out.print(','); out.print(s.accel_g.y, 6);
    out.print(','); out.print(s.accel_g.z, 6);
    out.print(','); out.print(s.gyro_rad_s.x, 7);
    out.print(','); out.print(s.gyro_rad_s.y, 7);
    out.print(','); out.print(s.gyro_rad_s.z, 7);
    out.print(','); out.print(s.temp_c, 3);
    out.print(",0x"); out.println(qualityFlags, HEX);
}

} // namespace tracker

#include "output/slimevr_packet_writer.hpp"

#include <cstring>

namespace tracker {

const char* slimevrUserActionName(SlimeVRUserAction action) {
    switch (action) {
        case SlimeVRUserAction::None: return "off";
        case SlimeVRUserAction::FullReset: return "full";
        case SlimeVRUserAction::YawReset: return "yaw";
        case SlimeVRUserAction::MountingReset: return "mounting";
        case SlimeVRUserAction::PauseTracking: return "pause";
    }
    return "unknown";
}

bool parseSlimeVRUserActionName(const char* text, SlimeVRUserAction& out) {
    if (!text) return false;
    auto eq = [](const char* a, const char* b) {
        while (*a && *b) {
            char ca = *a++;
            char cb = *b++;
            if (ca >= 'A' && ca <= 'Z') ca = static_cast<char>(ca - 'A' + 'a');
            if (cb >= 'A' && cb <= 'Z') cb = static_cast<char>(cb - 'A' + 'a');
            if (ca != cb) return false;
        }
        return *a == '\0' && *b == '\0';
    };
    if (eq(text, "off") || eq(text, "none")) out = SlimeVRUserAction::None;
    else if (eq(text, "full")) out = SlimeVRUserAction::FullReset;
    else if (eq(text, "yaw")) out = SlimeVRUserAction::YawReset;
    else if (eq(text, "mounting") || eq(text, "mount")) out = SlimeVRUserAction::MountingReset;
    else if (eq(text, "pause")) out = SlimeVRUserAction::PauseTracking;
    else return false;
    return true;
}

namespace {

uint32_t readU32BeLocal(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
}

uint32_t floatToU32(float value) {
    uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "float must be 32-bit IEEE-754");
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

const char* safeCString(const char* str) {
    return str ? str : "";
}

int16_t quantizeSignedQ(float value, float scale) {
    float scaled = value * scale;
    if (scaled >= 32767.0f) return 32767;
    if (scaled <= -32768.0f) return static_cast<int16_t>(-32768);
    scaled += scaled >= 0.0f ? 0.5f : -0.5f;
    return static_cast<int16_t>(scaled);
}

} // namespace

size_t SlimeVRPacketWriter::BufferCursor::size() const {
    return ptr && begin ? static_cast<size_t>(ptr - begin) : 0;
}

size_t SlimeVRPacketWriter::BufferCursor::remaining() const {
    const size_t used = size();
    return used <= capacity ? capacity - used : 0;
}

bool SlimeVRPacketWriter::BufferCursor::ensure(size_t n) {
    if (!begin || !ptr) {
        error = SlimeVRPacketWriteError::NullBuffer;
        return false;
    }
    if (remaining() < n) {
        error = SlimeVRPacketWriteError::BufferTooSmall;
        return false;
    }
    return true;
}

bool SlimeVRPacketWriter::BufferCursor::writeU8(uint8_t value) {
    if (!ensure(1)) return false;
    *ptr++ = value;
    return true;
}

bool SlimeVRPacketWriter::BufferCursor::writeU16Be(uint16_t value) {
    if (!ensure(2)) return false;
    *ptr++ = static_cast<uint8_t>((value >> 8) & 0xffu);
    *ptr++ = static_cast<uint8_t>(value & 0xffu);
    return true;
}

bool SlimeVRPacketWriter::BufferCursor::writeI16Be(int16_t value) {
    return writeU16Be(static_cast<uint16_t>(value));
}

bool SlimeVRPacketWriter::BufferCursor::writeU32Be(uint32_t value) {
    if (!ensure(4)) return false;
    *ptr++ = static_cast<uint8_t>((value >> 24) & 0xffu);
    *ptr++ = static_cast<uint8_t>((value >> 16) & 0xffu);
    *ptr++ = static_cast<uint8_t>((value >> 8) & 0xffu);
    *ptr++ = static_cast<uint8_t>(value & 0xffu);
    return true;
}

bool SlimeVRPacketWriter::BufferCursor::writeU64Be(uint64_t value) {
    if (!ensure(8)) return false;
    for (int shift = 56; shift >= 0; shift -= 8) {
        *ptr++ = static_cast<uint8_t>((value >> shift) & 0xffu);
    }
    return true;
}

bool SlimeVRPacketWriter::BufferCursor::writeF32Be(float value) {
    return writeU32Be(floatToU32(value));
}

bool SlimeVRPacketWriter::BufferCursor::writeBytes(const uint8_t* data, size_t n) {
    if (n == 0) return true;
    if (!data) {
        error = SlimeVRPacketWriteError::NullBuffer;
        return false;
    }
    if (!ensure(n)) return false;
    std::memcpy(ptr, data, n);
    ptr += n;
    return true;
}

bool SlimeVRPacketWriter::BufferCursor::writeByteString(const char* str) {
    const char* safe = safeCString(str);
    const size_t len = std::strlen(safe);
    if (len > 255) {
        error = SlimeVRPacketWriteError::StringTooLong;
        return false;
    }
    if (!writeU8(static_cast<uint8_t>(len))) return false;
    return writeBytes(reinterpret_cast<const uint8_t*>(safe), len);
}

void SlimeVRPacketWriter::resetPacketNumber(uint64_t nextPacketNumber) {
    nextPacketNumber_ = nextPacketNumber;
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::finish(const BufferCursor& cursor) const {
    SlimeVRPacketWriteResult result;
    result.ok = cursor.ok();
    result.size = cursor.ok() ? cursor.size() : 0;
    result.error = cursor.error;
    return result;
}

bool SlimeVRPacketWriter::writePacketHeader(BufferCursor& cursor, SlimeVRSendPacketType type) {
    if (!cursor.writeU32Be(static_cast<uint32_t>(type))) return false;
    if (!cursor.writeU64Be(nextPacketNumber_)) return false;
    ++nextPacketNumber_;
    return true;
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeHeartbeat(uint8_t* out, size_t capacity) {
    BufferCursor cursor{out, out, capacity};
    writePacketHeader(cursor, SlimeVRSendPacketType::HeartBeat);
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writePingPong(uint8_t* out, size_t capacity,
                                                            uint32_t pingId) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::PingPong)) {
        cursor.writeU32Be(pingId);
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeHandshake(uint8_t* out, size_t capacity,
                                                             const SlimeVRHandshakeInfo& info) {
    BufferCursor cursor{out, out, capacity};
    // Upstream SlimeVR discovery always uses packet number zero. A discovery
    // retry is not part of the established session sequence and must neither
    // inherit nor advance nextPacketNumber_.
    if (cursor.writeU32Be(static_cast<uint32_t>(SlimeVRSendPacketType::Handshake)) &&
        cursor.writeU64Be(0)) {
        cursor.writeU32Be(info.boardType);
        cursor.writeU32Be(info.imuType);
        cursor.writeU32Be(info.mcuType);
        cursor.writeU32Be(info.imuInfo0);
        cursor.writeU32Be(info.imuInfo1);
        cursor.writeU32Be(info.imuInfo2);
        cursor.writeU32Be(info.protocolVersion);
        cursor.writeByteString(info.firmwareVersion);
        cursor.writeBytes(info.mac, sizeof(info.mac));
        cursor.writeU8(static_cast<uint8_t>(info.trackerType));
        cursor.writeByteString(info.vendorName);
        cursor.writeByteString(info.vendorUrl);
        cursor.writeByteString(info.productName);
        cursor.writeByteString(info.updateAddress);
        cursor.writeByteString(info.updateName);
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeSensorInfo(uint8_t* out, size_t capacity,
                                                              const SlimeVRSensorInfo& info) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::SensorInfo)) {
        cursor.writeU8(info.sensorId);
        cursor.writeU8(static_cast<uint8_t>(info.sensorState));
        cursor.writeU8(static_cast<uint8_t>(info.imuType));
        cursor.writeU16Be(info.sensorConfig);
        cursor.writeU8(info.hasCompletedRestCalibration ? 1u : 0u);
        cursor.writeU8(info.sensorPosition);
        cursor.writeU8(static_cast<uint8_t>(info.sensorDataType));
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeRotationData(uint8_t* out, size_t capacity,
                                                                uint8_t sensorId,
                                                                const Quat& q,
                                                                uint8_t accuracyInfo,
                                                                SlimeVRRotationDataType dataType) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::RotationData)) {
        const Quat normalized = slimevr_motion_frame::rotationWireFromWorldDevice(q);
        cursor.writeU8(sensorId);
        cursor.writeU8(static_cast<uint8_t>(dataType));
        cursor.writeF32Be(normalized.x);
        cursor.writeF32Be(normalized.y);
        cursor.writeF32Be(normalized.z);
        cursor.writeF32Be(normalized.w);
        cursor.writeU8(accuracyInfo);
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeAcceleration(uint8_t* out, size_t capacity,
                                                                 uint8_t sensorId,
                                                                 const Vec3& linearAccelerationMps2) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::Accel)) {
        // Packet 4 payload is x/y/z SI linear acceleration followed by sensor id.
        cursor.writeF32Be(linearAccelerationMps2.x);
        cursor.writeF32Be(linearAccelerationMps2.y);
        cursor.writeF32Be(linearAccelerationMps2.z);
        cursor.writeU8(sensorId);
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeRotationAndAcceleration(
    uint8_t* out,
    size_t capacity,
    uint8_t sensorId,
    const Quat& q,
    const Vec3& linearAccelerationMps2
) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::RotationAndAcceleration)) {
        const Quat normalized = slimevr_motion_frame::rotationWireFromWorldDevice(q);
        cursor.writeU8(sensorId);
        // Packet 23 uses signed Q15 quaternion components and signed Q7
        // acceleration in m/s^2. The server normalizes the decoded quaternion.
        cursor.writeI16Be(quantizeSignedQ(normalized.x, 32768.0f));
        cursor.writeI16Be(quantizeSignedQ(normalized.y, 32768.0f));
        cursor.writeI16Be(quantizeSignedQ(normalized.z, 32768.0f));
        cursor.writeI16Be(quantizeSignedQ(normalized.w, 32768.0f));
        cursor.writeI16Be(quantizeSignedQ(linearAccelerationMps2.x, 128.0f));
        cursor.writeI16Be(quantizeSignedQ(linearAccelerationMps2.y, 128.0f));
        cursor.writeI16Be(quantizeSignedQ(linearAccelerationMps2.z, 128.0f));
    }
    return finish(cursor);
}


SlimeVRPacketWriteResult SlimeVRPacketWriter::writeRotationAccelerationBundle(
    uint8_t* out,
    size_t capacity,
    uint8_t sensorId,
    const Quat& q,
    uint8_t accuracyInfo,
    const Vec3& linearAccelerationMps2,
    SlimeVRRotationDataType dataType
) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::Bundle)) {
        const Quat normalized = slimevr_motion_frame::rotationWireFromWorldDevice(q);

        // Packet-100 inner records have a u16 length and contain packet type
        // plus payload, but no inner packet number. Preserve rotation-before-
        // acceleration ordering so step mounting observes both from one
        // coherent prepared snapshot.
        constexpr uint16_t rotationInnerSize = 4u + 19u;
        cursor.writeU16Be(rotationInnerSize);
        cursor.writeU32Be(static_cast<uint32_t>(SlimeVRSendPacketType::RotationData));
        cursor.writeU8(sensorId);
        cursor.writeU8(static_cast<uint8_t>(dataType));
        cursor.writeF32Be(normalized.x);
        cursor.writeF32Be(normalized.y);
        cursor.writeF32Be(normalized.z);
        cursor.writeF32Be(normalized.w);
        cursor.writeU8(accuracyInfo);

        constexpr uint16_t accelerationInnerSize = 4u + 13u;
        cursor.writeU16Be(accelerationInnerSize);
        cursor.writeU32Be(static_cast<uint32_t>(SlimeVRSendPacketType::Accel));
        cursor.writeF32Be(linearAccelerationMps2.x);
        cursor.writeF32Be(linearAccelerationMps2.y);
        cursor.writeF32Be(linearAccelerationMps2.z);
        cursor.writeU8(sensorId);
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeFeatureFlags(
    uint8_t* out,
    size_t capacity,
    const uint8_t* flags,
    size_t flagsLength
) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::FeatureFlags)) {
        if (flagsLength != 0u) cursor.writeBytes(flags, flagsLength);
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeBatteryLevel(uint8_t* out, size_t capacity,
                                                                float voltage,
                                                                float percentage) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::BatteryLevel)) {
        cursor.writeF32Be(voltage);
        cursor.writeF32Be(percentage);
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeTap(uint8_t* out, size_t capacity,
                                                       uint8_t sensorId,
                                                       uint8_t value) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::Tap)) {
        cursor.writeU8(sensorId);
        cursor.writeU8(value);
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeUserAction(uint8_t* out, size_t capacity,
                                                              SlimeVRUserAction action) {
    BufferCursor cursor{out, out, capacity};
    if (action == SlimeVRUserAction::None) {
        cursor.error = SlimeVRPacketWriteError::InvalidArgument;
        return finish(cursor);
    }
    if (writePacketHeader(cursor, SlimeVRSendPacketType::UserAction)) {
        cursor.writeU8(static_cast<uint8_t>(action));
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeError(uint8_t* out, size_t capacity,
                                                         uint8_t sensorId,
                                                         uint8_t errorCode,
                                                         const char* message) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::Error)) {
        cursor.writeU8(sensorId);
        cursor.writeU8(errorCode);
        cursor.writeByteString(message);
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeMagnetometerAccuracy(uint8_t* out, size_t capacity,
                                                                         uint8_t sensorId,
                                                                         float accuracyInfo) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::MagnetometerAccuracy)) {
        cursor.writeU8(sensorId);
        cursor.writeF32Be(accuracyInfo);
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeSignalStrength(uint8_t* out, size_t capacity,
                                                                  uint8_t sensorId,
                                                                  int8_t signalStrengthDbm) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::SignalStrength)) {
        cursor.writeU8(sensorId);
        cursor.writeU8(static_cast<uint8_t>(signalStrengthDbm));
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeTemperature(uint8_t* out, size_t capacity,
                                                               uint8_t sensorId,
                                                               float temperatureC) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::Temperature)) {
        cursor.writeU8(sensorId);
        cursor.writeF32Be(temperatureC);
    }
    return finish(cursor);
}

SlimeVRPacketWriteResult SlimeVRPacketWriter::writeAcknowledgeConfigChange(uint8_t* out, size_t capacity,
                                                                           uint8_t sensorId,
                                                                           uint16_t configType) {
    BufferCursor cursor{out, out, capacity};
    if (writePacketHeader(cursor, SlimeVRSendPacketType::AcknowledgeConfigChange)) {
        cursor.writeU8(sensorId);
        cursor.writeU16Be(configType);
    }
    return finish(cursor);
}

bool SlimeVRPacketWriter::isPacketType(const uint8_t* data, size_t len, SlimeVRReceivePacketType type) {
    if (!data || len == 0) return false;
    const uint8_t expected = static_cast<uint8_t>(type);
    if (len >= SLIMEVR_PACKET_HEADER_SIZE) {
        const uint32_t headerType = readU32BeLocal(data);
        if (headerType <= 0xffu) {
            return static_cast<uint8_t>(headerType) == expected;
        }
    }
    return data[0] == expected;
}

bool SlimeVRPacketWriter::isServerHandshakeResponse(const uint8_t* data, size_t len) {
    if (!isPacketType(data, len, SlimeVRReceivePacketType::Handshake)) return false;
    if (len < SLIMEVR_DISCOVERY_RESPONSE_SIZE) return false;
    return std::memcmp(data + 1, SLIMEVR_DISCOVERY_RESPONSE, SLIMEVR_DISCOVERY_RESPONSE_SIZE - 1) == 0;
}

} // namespace tracker

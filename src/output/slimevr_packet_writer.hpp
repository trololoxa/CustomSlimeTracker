#pragma once

#include <cstddef>
#include <cstdint>

#include "core/math.hpp"

namespace tracker {

// SlimeVR tracker -> server UDP protocol packet writer.
//
// This module is intentionally host-safe: no Arduino, WiFi, UDP, heap, or
// blocking calls. Runtime/network code owns transport and scheduling; this
// class only serializes packets into a caller-provided buffer.
//
// Outgoing tracker packets use:
//   packetType:u32be + packetNumber:u64be + payload
// Packet type values below fit into the low byte, so the first four bytes are
// normally 00 00 00 <type>.

constexpr uint8_t SLIMEVR_PROTOCOL_VERSION = 19;
constexpr uint16_t SLIMEVR_DEFAULT_SERVER_PORT = 6969;
constexpr size_t SLIMEVR_PACKET_HEADER_SIZE = 12;
constexpr size_t SLIMEVR_DISCOVERY_RESPONSE_SIZE = 13;
constexpr const char* SLIMEVR_DISCOVERY_RESPONSE = "Hey OVR =D 5";

enum class SlimeVRSendPacketType : uint8_t {
    HeartBeat = 0,
    Handshake = 3,
    Accel = 4,
    Serial = 11,
    BatteryLevel = 12,
    Tap = 13,
    Error = 14,
    SensorInfo = 15,
    RotationData = 17,
    MagnetometerAccuracy = 18,
    SignalStrength = 19,
    Temperature = 20,
    FeatureFlags = 22,
    AcknowledgeConfigChange = 24,
    FlexData = 26,
    Bundle = 100,
    Inspection = 105,
};

enum class SlimeVRReceivePacketType : uint8_t {
    HeartBeat = 1,
    Handshake = 3,
    PingPong = 10,
    FeatureFlags = 22,
    SetConfigFlag = 25,
};

enum class SlimeVRSensorState : uint8_t {
    Offline = 0,
    Online = 1,
};

enum class SlimeVRImuType : uint8_t {
    Unknown = 0,
    LSM6DSV = 13,
};

enum class SlimeVRSensorDataType : uint8_t {
    Rotation = 0,
};

enum class SlimeVRRotationDataType : uint8_t {
    Normal = 1,
};

enum class SlimeVRTrackerType : uint8_t {
    Tracker = 0,
};

enum class SlimeVRPacketWriteError : uint8_t {
    None = 0,
    NullBuffer,
    BufferTooSmall,
    StringTooLong,
};

struct SlimeVRPacketWriteResult {
    bool ok = false;
    size_t size = 0;
    SlimeVRPacketWriteError error = SlimeVRPacketWriteError::None;
};

struct SlimeVRHandshakeInfo {
    // Current SlimeVR enum ids used by the sender example/server metadata.
    // Keep these configurable so custom boards can override them later.
    uint32_t boardType = 10; // LOLIN_C3_MINI. Use 4 for generic CUSTOM boards.
    uint32_t imuType = 13;   // LSM6DSV.
    uint32_t mcuType = 6;    // ESP32_C3.

    uint32_t imuInfo0 = 0;
    uint32_t imuInfo1 = 0;
    uint32_t imuInfo2 = 0;

    uint32_t protocolVersion = SLIMEVR_PROTOCOL_VERSION;
    const char* firmwareVersion = "track-fw";
    uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
    SlimeVRTrackerType trackerType = SlimeVRTrackerType::Tracker;

    const char* vendorName = "CustomSlime";
    const char* vendorUrl = "https://github.com";
    const char* productName = "CustomTracker";
    const char* updateAddress = "https://github.com";
    const char* updateName = "CustomTracker";
};

struct SlimeVRSensorInfo {
    uint8_t sensorId = 0;
    SlimeVRSensorState sensorState = SlimeVRSensorState::Online;
    SlimeVRImuType imuType = SlimeVRImuType::LSM6DSV;
    // SensorConfig is a u16 in the current server packet parser. Keep it zero
    // until mag/config flags are implemented and acknowledged correctly.
    uint16_t sensorConfig = 0;
    bool hasCompletedRestCalibration = true;
    uint8_t sensorPosition = 0;
    SlimeVRSensorDataType sensorDataType = SlimeVRSensorDataType::Rotation;
};

class SlimeVRPacketWriter {
public:
    void resetPacketNumber(uint64_t nextPacketNumber = 0);
    uint64_t packetNumber() const { return nextPacketNumber_; }

    SlimeVRPacketWriteResult writeHeartbeat(uint8_t* out, size_t capacity);
    SlimeVRPacketWriteResult writeHandshake(uint8_t* out, size_t capacity,
                                            const SlimeVRHandshakeInfo& info);
    SlimeVRPacketWriteResult writeSensorInfo(uint8_t* out, size_t capacity,
                                             const SlimeVRSensorInfo& info);
    SlimeVRPacketWriteResult writeRotationData(uint8_t* out, size_t capacity,
                                               uint8_t sensorId,
                                               const Quat& q,
                                               uint8_t accuracyInfo = 0,
                                               SlimeVRRotationDataType dataType = SlimeVRRotationDataType::Normal);
    SlimeVRPacketWriteResult writeBatteryLevel(uint8_t* out, size_t capacity,
                                               float voltage,
                                               float percentage);
    SlimeVRPacketWriteResult writeSignalStrength(uint8_t* out, size_t capacity,
                                                 uint8_t sensorId,
                                                 uint8_t signalStrength);
    SlimeVRPacketWriteResult writeTemperature(uint8_t* out, size_t capacity,
                                              uint8_t sensorId,
                                              float temperatureC);
    SlimeVRPacketWriteResult writeAcknowledgeConfigChange(uint8_t* out, size_t capacity,
                                                          uint8_t sensorId,
                                                          uint32_t configType);

    static bool isServerHandshakeResponse(const uint8_t* data, size_t len);
    static bool isPacketType(const uint8_t* data, size_t len, SlimeVRReceivePacketType type);

private:
    uint64_t nextPacketNumber_ = 0;

    struct BufferCursor {
        uint8_t* begin = nullptr;
        uint8_t* ptr = nullptr;
        size_t capacity = 0;
        SlimeVRPacketWriteError error = SlimeVRPacketWriteError::None;

        size_t size() const;
        size_t remaining() const;
        bool ok() const { return error == SlimeVRPacketWriteError::None; }
        bool ensure(size_t n);
        bool writeU8(uint8_t value);
        bool writeU16Be(uint16_t value);
        bool writeU32Be(uint32_t value);
        bool writeU64Be(uint64_t value);
        bool writeF32Be(float value);
        bool writeBytes(const uint8_t* data, size_t n);
        bool writeByteString(const char* str);
    };

    SlimeVRPacketWriteResult finish(const BufferCursor& cursor) const;
    bool writePacketHeader(BufferCursor& cursor, SlimeVRSendPacketType type);
};

} // namespace tracker

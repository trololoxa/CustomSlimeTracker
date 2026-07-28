#include "test_common.hpp"

#include <cstdint>
#include <cstring>

#include "output/slimevr_packet_writer.hpp"

using namespace tracker;

namespace {

uint16_t readU16Be(const uint8_t* p) {
    return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

uint32_t readU32Be(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
}

uint64_t readU64Be(const uint8_t* p) {
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value = (value << 8) | p[i];
    }
    return value;
}

float readF32Be(const uint8_t* p) {
    const uint32_t bits = readU32Be(p);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

int16_t readI16Be(const uint8_t* p) {
    return static_cast<int16_t>(
        (static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1])
    );
}

} // namespace

int main() {
    TestContext ctx;

    uint8_t packet[512] = {};
    SlimeVRPacketWriter writer;

    writer.resetPacketNumber(0x0102030405060708ULL);
    const SlimeVRPacketWriteResult heartbeat = writer.writeHeartbeat(packet, sizeof(packet));
    CHECK(ctx, heartbeat.ok);
    CHECK(ctx, heartbeat.size == SLIMEVR_PACKET_HEADER_SIZE);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::HeartBeat));
    CHECK(ctx, readU64Be(packet + 4) == 0x0102030405060708ULL);
    CHECK(ctx, writer.packetNumber() == 0x0102030405060709ULL);

    const Quat rawQ(2.0f, 0.0f, 1.0f, 0.0f);
    const SlimeVRPacketWriteResult rotation = writer.writeRotationData(packet, sizeof(packet), 7, rawQ, 42);
    CHECK(ctx, rotation.ok);
    CHECK(ctx, rotation.size == SLIMEVR_PACKET_HEADER_SIZE + 19u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::RotationData));
    CHECK(ctx, readU64Be(packet + 4) == 0x0102030405060709ULL);
    CHECK(ctx, packet[12] == 7);
    CHECK(ctx, packet[13] == static_cast<uint8_t>(SlimeVRRotationDataType::Normal));
    CHECK_NEAR(ctx, readF32Be(packet + 14), 0.0f, 1.0e-6f); // x
    CHECK_NEAR(ctx, readF32Be(packet + 18), 0.4472136f, 1.0e-5f); // y normalized
    CHECK_NEAR(ctx, readF32Be(packet + 22), 0.0f, 1.0e-6f); // z
    CHECK_NEAR(ctx, readF32Be(packet + 26), 0.8944272f, 1.0e-5f); // w normalized
    CHECK(ctx, packet[30] == 42);

    const Vec3 linearAccelerationMps2(2.0f, -4.0f, 8.0f);
    const SlimeVRPacketWriteResult acceleration =
        writer.writeAcceleration(packet, sizeof(packet), 7, linearAccelerationMps2);
    CHECK(ctx, acceleration.ok);
    CHECK(ctx, acceleration.size == SLIMEVR_PACKET_HEADER_SIZE + 13u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::Accel));
    CHECK(ctx, readU64Be(packet + 4) == 0x010203040506070aULL);
    CHECK_NEAR(ctx, readF32Be(packet + 12), 2.0f, 1.0e-6f);
    CHECK_NEAR(ctx, readF32Be(packet + 16), -4.0f, 1.0e-6f);
    CHECK_NEAR(ctx, readF32Be(packet + 20), 8.0f, 1.0e-6f);
    CHECK(ctx, packet[24] == 7);

    const SlimeVRPacketWriteResult compactMotion =
        writer.writeRotationAndAcceleration(packet, sizeof(packet), 7, rawQ, linearAccelerationMps2);
    CHECK(ctx, compactMotion.ok);
    CHECK(ctx, compactMotion.size == SLIMEVR_PACKET_HEADER_SIZE + 15u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::RotationAndAcceleration));
    CHECK(ctx, readU64Be(packet + 4) == 0x010203040506070bULL);
    CHECK(ctx, packet[12] == 7);
    CHECK(ctx, readI16Be(packet + 13) == 0);
    CHECK_NEAR(ctx, static_cast<float>(readI16Be(packet + 15)) / 32768.0f, 0.4472136f, 4.0e-5f);
    CHECK(ctx, readI16Be(packet + 17) == 0);
    CHECK_NEAR(ctx, static_cast<float>(readI16Be(packet + 19)) / 32768.0f, 0.8944272f, 4.0e-5f);
    CHECK_NEAR(ctx, static_cast<float>(readI16Be(packet + 21)) / 128.0f, 2.0f, 4.0e-3f);
    CHECK_NEAR(ctx, static_cast<float>(readI16Be(packet + 23)) / 128.0f, -4.0f, 4.0e-3f);
    CHECK_NEAR(ctx, static_cast<float>(readI16Be(packet + 25)) / 128.0f, 8.0f, 4.0e-3f);

    const SlimeVRPacketWriteResult bundle = writer.writeRotationAccelerationBundle(
        packet, sizeof(packet), 7, rawQ, 42, linearAccelerationMps2
    );
    CHECK(ctx, bundle.ok);
    CHECK(ctx, bundle.size == 56u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::Bundle));
    CHECK(ctx, readU64Be(packet + 4) == 0x010203040506070cULL);
    CHECK(ctx, readU16Be(packet + 12) == 23u);
    CHECK(ctx, readU32Be(packet + 14) == static_cast<uint32_t>(SlimeVRSendPacketType::RotationData));
    CHECK(ctx, packet[18] == 7u);
    CHECK(ctx, packet[19] == static_cast<uint8_t>(SlimeVRRotationDataType::Normal));
    CHECK_NEAR(ctx, readF32Be(packet + 20), 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, readF32Be(packet + 24), 0.4472136f, 1.0e-5f);
    CHECK_NEAR(ctx, readF32Be(packet + 28), 0.0f, 1.0e-6f);
    CHECK_NEAR(ctx, readF32Be(packet + 32), 0.8944272f, 1.0e-5f);
    CHECK(ctx, packet[36] == 42u);
    CHECK(ctx, readU16Be(packet + 37) == 17u);
    CHECK(ctx, readU32Be(packet + 39) == static_cast<uint32_t>(SlimeVRSendPacketType::Accel));
    CHECK_NEAR(ctx, readF32Be(packet + 43), 2.0f, 1.0e-6f);
    CHECK_NEAR(ctx, readF32Be(packet + 47), -4.0f, 1.0e-6f);
    CHECK_NEAR(ctx, readF32Be(packet + 51), 8.0f, 1.0e-6f);
    CHECK(ctx, packet[55] == 7u);

    const uint8_t firmwareFlags[] = {SLIMEVR_FIRMWARE_FEATURE_FLAGS};
    const SlimeVRPacketWriteResult featureFlags = writer.writeFeatureFlags(
        packet, sizeof(packet), firmwareFlags, sizeof(firmwareFlags)
    );
    CHECK(ctx, featureFlags.ok);
    CHECK(ctx, featureFlags.size == SLIMEVR_PACKET_HEADER_SIZE + 1u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::FeatureFlags));
    CHECK(ctx, packet[12] == static_cast<uint8_t>(1u << SLIMEVR_FIRMWARE_FEATURE_SENSOR_CONFIG));
    CHECK(ctx, (packet[12] & 0x01u) == 0u); // Firmware does not claim server bundle capability.

    SlimeVRSensorInfo sensor;
    sensor.sensorId = 3;
    sensor.sensorConfig = 0x1234;
    sensor.sensorPosition = 5;
    CHECK(ctx, SLIMEVR_SENSOR_CONFIG_MAG_ENABLED == 0x0001u);
    CHECK(ctx, SLIMEVR_SENSOR_CONFIG_MAG_SUPPORTED == 0x0002u);
    CHECK(ctx, SLIMEVR_SENSOR_CONFIG_MAG_SUPPORTED_AND_ENABLED == 0x0003u);
    const SlimeVRPacketWriteResult sensorInfo = writer.writeSensorInfo(packet, sizeof(packet), sensor);
    CHECK(ctx, sensorInfo.ok);
    CHECK(ctx, sensorInfo.size == SLIMEVR_PACKET_HEADER_SIZE + 8u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::SensorInfo));
    CHECK(ctx, packet[12] == 3);
    CHECK(ctx, packet[13] == static_cast<uint8_t>(SlimeVRSensorState::Online));
    CHECK(ctx, packet[14] == static_cast<uint8_t>(SlimeVRImuType::LSM6DSV));
    CHECK(ctx, packet[15] == 0x12); // sensorConfig u16be high byte
    CHECK(ctx, packet[16] == 0x34); // sensorConfig u16be low byte
    CHECK(ctx, packet[17] == 1); // hasCompletedRestCalibration
    CHECK(ctx, packet[18] == 5); // tracker position
    CHECK(ctx, packet[19] == static_cast<uint8_t>(SlimeVRSensorDataType::Rotation));

    CHECK(ctx, SLIMEVR_PROTOCOL_VERSION == 22u);
    CHECK(ctx, slimevr_motion_frame::protocolUsesCorrectedAcceleration(SLIMEVR_PROTOCOL_VERSION));

    SlimeVRHandshakeInfo handshakeInfo;
    handshakeInfo.firmwareVersion = "fw";
    handshakeInfo.vendorName = "v";
    handshakeInfo.vendorUrl = "u";
    handshakeInfo.productName = "p";
    handshakeInfo.updateAddress = "a";
    handshakeInfo.updateName = "n";
    handshakeInfo.mac[0] = 0xaa;
    handshakeInfo.mac[5] = 0x55;
    const uint64_t sequenceBeforeHandshake = writer.packetNumber();
    const SlimeVRPacketWriteResult handshake = writer.writeHandshake(packet, sizeof(packet), handshakeInfo);
    CHECK(ctx, handshake.ok);
    CHECK(ctx, handshake.size == SLIMEVR_PACKET_HEADER_SIZE + 24u + 4u + 1u + 2u + 6u + 1u + 5u * 2u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::Handshake));
    CHECK(ctx, readU64Be(packet + 4) == 0u);
    CHECK(ctx, writer.packetNumber() == sequenceBeforeHandshake);
    CHECK(ctx, readU32Be(packet + 12) == 10u); // boardType: LOLIN_C3_MINI
    CHECK(ctx, readU32Be(packet + 16) == 13u); // imuType: LSM6DSV
    CHECK(ctx, readU32Be(packet + 20) == 6u);  // mcuType: ESP32_C3
    CHECK(ctx, readU32Be(packet + 36) == SLIMEVR_PROTOCOL_VERSION);
    CHECK(ctx, packet[40] == 2);
    CHECK(ctx, packet[41] == 'f');
    CHECK(ctx, packet[42] == 'w');
    CHECK(ctx, packet[43] == 0xaa);
    CHECK(ctx, packet[48] == 0x55);

    const SlimeVRPacketWriteResult handshakeRetry = writer.writeHandshake(packet, sizeof(packet), handshakeInfo);
    CHECK(ctx, handshakeRetry.ok);
    CHECK(ctx, readU64Be(packet + 4) == 0u);
    CHECK(ctx, writer.packetNumber() == sequenceBeforeHandshake);


    CHECK(ctx, std::strcmp(slimevrUserActionName(SlimeVRUserAction::None), "off") == 0);
    CHECK(ctx, std::strcmp(slimevrUserActionName(SlimeVRUserAction::YawReset), "yaw") == 0);
    SlimeVRUserAction parsedAction = SlimeVRUserAction::None;
    CHECK(ctx, parseSlimeVRUserActionName("full", parsedAction));
    CHECK(ctx, parsedAction == SlimeVRUserAction::FullReset);
    CHECK(ctx, parseSlimeVRUserActionName("mount", parsedAction));
    CHECK(ctx, parsedAction == SlimeVRUserAction::MountingReset);
    CHECK(ctx, !parseSlimeVRUserActionName("invalid", parsedAction));

    const SlimeVRPacketWriteResult userAction = writer.writeUserAction(
        packet, sizeof(packet), SlimeVRUserAction::YawReset
    );
    CHECK(ctx, userAction.ok);
    CHECK(ctx, userAction.size == SLIMEVR_PACKET_HEADER_SIZE + 1u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::UserAction));
    CHECK(ctx, packet[12] == static_cast<uint8_t>(SlimeVRUserAction::YawReset));
    const SlimeVRPacketWriteResult disabledAction = writer.writeUserAction(
        packet, sizeof(packet), SlimeVRUserAction::None
    );
    CHECK(ctx, !disabledAction.ok);
    CHECK(ctx, disabledAction.error == SlimeVRPacketWriteError::InvalidArgument);

    const SlimeVRPacketWriteResult tap = writer.writeTap(packet, sizeof(packet), 2, 2);
    CHECK(ctx, tap.ok);
    CHECK(ctx, tap.size == SLIMEVR_PACKET_HEADER_SIZE + 2u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::Tap));
    CHECK(ctx, packet[12] == 2);
    CHECK(ctx, packet[13] == 2);

    const SlimeVRPacketWriteResult err = writer.writeError(packet, sizeof(packet), 2, 1, "LSM6DSV init failed");
    CHECK(ctx, err.ok);
    CHECK(ctx, err.size == SLIMEVR_PACKET_HEADER_SIZE + 2u + 1u + std::strlen("LSM6DSV init failed"));
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::Error));
    CHECK(ctx, packet[12] == 2);
    CHECK(ctx, packet[13] == 1);
    CHECK(ctx, packet[14] == std::strlen("LSM6DSV init failed"));
    CHECK(ctx, std::memcmp(packet + 15, "LSM6DSV init failed", std::strlen("LSM6DSV init failed")) == 0);

    const SlimeVRPacketWriteResult battery = writer.writeBatteryLevel(packet, sizeof(packet), 3.70f, 42.0f);
    CHECK(ctx, battery.ok);
    CHECK(ctx, battery.size == SLIMEVR_PACKET_HEADER_SIZE + 8u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::BatteryLevel));
    CHECK_NEAR(ctx, readF32Be(packet + 12), 3.70f, 1.0e-6f);
    CHECK_NEAR(ctx, readF32Be(packet + 16), 42.0f, 1.0e-6f);

    const SlimeVRPacketWriteResult magAcc = writer.writeMagnetometerAccuracy(packet, sizeof(packet), 9, 0.5f);
    CHECK(ctx, magAcc.ok);
    CHECK(ctx, magAcc.size == SLIMEVR_PACKET_HEADER_SIZE + 5u);
    CHECK(ctx, packet[12] == 9);
    CHECK_NEAR(ctx, readF32Be(packet + 13), 0.5f, 1.0e-6f);

    const SlimeVRPacketWriteResult signal = writer.writeSignalStrength(packet, sizeof(packet), 255, -68);
    CHECK(ctx, signal.ok);
    CHECK(ctx, signal.size == SLIMEVR_PACKET_HEADER_SIZE + 2u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::SignalStrength));
    CHECK(ctx, packet[12] == 255);
    CHECK(ctx, packet[13] == static_cast<uint8_t>(static_cast<int8_t>(-68)));

    const SlimeVRPacketWriteResult pong = writer.writePingPong(packet, sizeof(packet), 0xAABBCCDDu);
    CHECK(ctx, pong.ok);
    CHECK(ctx, pong.size == SLIMEVR_PACKET_HEADER_SIZE + 4u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::PingPong));
    CHECK(ctx, readU32Be(packet + 12) == 0xAABBCCDDu);

    const SlimeVRPacketWriteResult ack = writer.writeAcknowledgeConfigChange(packet, sizeof(packet), 4, SLIMEVR_CONFIG_TYPE_MAGNETOMETER);
    CHECK(ctx, ack.ok);
    CHECK(ctx, ack.size == SLIMEVR_PACKET_HEADER_SIZE + 3u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::AcknowledgeConfigChange));
    CHECK(ctx, packet[12] == 4);
    CHECK(ctx, packet[13] == 0x00);
    CHECK(ctx, packet[14] == 0x01);

    const uint8_t discovery[] = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    CHECK(ctx, SlimeVRPacketWriter::isServerHandshakeResponse(discovery, sizeof(discovery)));
    CHECK(ctx, SlimeVRPacketWriter::isPacketType(discovery, sizeof(discovery), SlimeVRReceivePacketType::Handshake));
    CHECK(ctx, !SlimeVRPacketWriter::isPacketType(discovery, sizeof(discovery), SlimeVRReceivePacketType::PingPong));

    const SlimeVRPacketWriteResult pong2 = writer.writePingPong(packet, sizeof(packet), 0x01020304u);
    CHECK(ctx, pong2.ok);
    CHECK(ctx, SlimeVRPacketWriter::isPacketType(packet, pong2.size, SlimeVRReceivePacketType::PingPong));
    CHECK(ctx, !SlimeVRPacketWriter::isPacketType(packet, pong2.size, SlimeVRReceivePacketType::SetConfigFlag));

    const SlimeVRPacketWriteResult tooSmall = writer.writeHeartbeat(packet, 3);
    CHECK(ctx, !tooSmall.ok);
    CHECK(ctx, tooSmall.error == SlimeVRPacketWriteError::BufferTooSmall);

    return ctx.finish("slimevr_packet_writer");
}

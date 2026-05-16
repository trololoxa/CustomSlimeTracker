#include "test_common.hpp"

#include <cstdint>
#include <cstring>

#include "output/slimevr_packet_writer.hpp"

using namespace tracker;

namespace {

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

    SlimeVRHandshakeInfo handshakeInfo;
    handshakeInfo.firmwareVersion = "fw";
    handshakeInfo.vendorName = "v";
    handshakeInfo.vendorUrl = "u";
    handshakeInfo.productName = "p";
    handshakeInfo.updateAddress = "a";
    handshakeInfo.updateName = "n";
    handshakeInfo.mac[0] = 0xaa;
    handshakeInfo.mac[5] = 0x55;
    const SlimeVRPacketWriteResult handshake = writer.writeHandshake(packet, sizeof(packet), handshakeInfo);
    CHECK(ctx, handshake.ok);
    CHECK(ctx, handshake.size == SLIMEVR_PACKET_HEADER_SIZE + 24u + 4u + 1u + 2u + 6u + 1u + 5u * 2u);
    CHECK(ctx, readU32Be(packet) == static_cast<uint32_t>(SlimeVRSendPacketType::Handshake));
    CHECK(ctx, readU32Be(packet + 12) == 10u); // boardType: LOLIN_C3_MINI
    CHECK(ctx, readU32Be(packet + 16) == 13u); // imuType: LSM6DSV
    CHECK(ctx, readU32Be(packet + 20) == 6u);  // mcuType: ESP32_C3
    CHECK(ctx, readU32Be(packet + 36) == SLIMEVR_PROTOCOL_VERSION);
    CHECK(ctx, packet[40] == 2);
    CHECK(ctx, packet[41] == 'f');
    CHECK(ctx, packet[42] == 'w');
    CHECK(ctx, packet[43] == 0xaa);
    CHECK(ctx, packet[48] == 0x55);

    const SlimeVRPacketWriteResult magAcc = writer.writeMagnetometerAccuracy(packet, sizeof(packet), 9, 0.5f);
    CHECK(ctx, magAcc.ok);
    CHECK(ctx, magAcc.size == SLIMEVR_PACKET_HEADER_SIZE + 5u);
    CHECK(ctx, packet[12] == 9);
    CHECK_NEAR(ctx, readF32Be(packet + 13), 0.5f, 1.0e-6f);

    const uint8_t discovery[] = {
        static_cast<uint8_t>(SlimeVRReceivePacketType::Handshake),
        'H', 'e', 'y', ' ', 'O', 'V', 'R', ' ', '=', 'D', ' ', '5'
    };
    CHECK(ctx, SlimeVRPacketWriter::isServerHandshakeResponse(discovery, sizeof(discovery)));
    CHECK(ctx, SlimeVRPacketWriter::isPacketType(discovery, sizeof(discovery), SlimeVRReceivePacketType::Handshake));
    CHECK(ctx, !SlimeVRPacketWriter::isPacketType(discovery, sizeof(discovery), SlimeVRReceivePacketType::PingPong));

    const SlimeVRPacketWriteResult tooSmall = writer.writeHeartbeat(packet, 3);
    CHECK(ctx, !tooSmall.ok);
    CHECK(ctx, tooSmall.error == SlimeVRPacketWriteError::BufferTooSmall);

    return ctx.finish("slimevr_packet_writer");
}

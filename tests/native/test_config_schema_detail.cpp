#include "test_common.hpp"

#include <cstdint>
#include <cstddef>
#include <cstring>

#include "config/tracker_config_detail.hpp"
#include "config/tracker_config_schema.hpp"

using namespace tracker;

static void testFnvCrcKnownVector(TestContext& ctx) {
    const uint8_t abc[] = {'a', 'b', 'c'};
    CHECK(ctx, tracker_config_detail::fnv1a32(abc, sizeof(abc)) == 0x1A47E90Bu);
}

static void testDefaultSchemaHeader(TestContext& ctx) {
    TrackerConfigBlob blob;
    CHECK(ctx, blob.magic == tracker_config_detail::CONFIG_MAGIC);
    CHECK(ctx, blob.version == tracker_config_detail::CONFIG_VERSION);
    CHECK(ctx, blob.size == sizeof(TrackerConfigBlob));
    CHECK(ctx, blob.schemas.hardware == tracker_config_detail::SCHEMA_HARDWARE_VERSION);
    CHECK(ctx, blob.schemas.ahrs == tracker_config_detail::SCHEMA_AHRS_VERSION);
    CHECK(ctx, blob.schemas.magYaw == tracker_config_detail::SCHEMA_MAG_YAW_VERSION);
    CHECK(ctx, blob.hardware.spiHz == tracker_config_detail::DEFAULT_SPI_HZ);
    CHECK(ctx, blob.output.packetFormat ==
        (tracker_config_detail::OUTPUT_PACKET_MODE_MARKER |
         static_cast<uint8_t>(SlimeVRMotionPacketPolicy::QuaternionOnly)));
    CHECK(ctx, blob.reservedDevice.reservedDeviceName[0] == '\0');
}

static void testConfigBlobLayoutGuards(TestContext& ctx) {
    // Persistent config is stored as one binary blob in NVS. These guards do
    // not add firmware runtime cost; they only make host tests fail when the
    // layout changes accidentally instead of through an intentional migration.
    CHECK(ctx, sizeof(TrackerConfigBlob) == 756);
    CHECK(ctx, offsetof(TrackerConfigBlob, magic) == 0);
    CHECK(ctx, offsetof(TrackerConfigBlob, crc32) == 8);
    CHECK(ctx, offsetof(TrackerConfigBlob, schemas) == 12);
    CHECK(ctx, offsetof(TrackerConfigBlob, reservedDevice) == 716);
    CHECK(ctx, sizeof(TrackerConfigSchemaVersions) == 24);
}

static void testFiniteHelpers(TestContext& ctx) {
    CHECK(ctx, tracker_config_detail::finiteVec3(Vec3(1.0f, 2.0f, 3.0f)));
    CHECK(ctx, !tracker_config_detail::finiteVec3(Vec3(1.0f, std::nanf(""), 3.0f)));
    CHECK(ctx, tracker_config_detail::finiteMat3(Mat3::identity()));
    CHECK_NEAR(ctx, tracker_config_detail::clampFloat(5.0f, 0.0f, 1.0f), 1.0f, 1.0e-6f);
    CHECK_NEAR(ctx, tracker_config_detail::clampFloat(-5.0f, 0.0f, 1.0f), 0.0f, 1.0e-6f);
}

int main() {
    TestContext ctx;
    testFnvCrcKnownVector(ctx);
    testDefaultSchemaHeader(ctx);
    testConfigBlobLayoutGuards(ctx);
    testFiniteHelpers(ctx);
    return ctx.finish("test_config_schema_detail");
}

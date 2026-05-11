#include "test_common.hpp"

#include <cstdint>
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
    CHECK(ctx, blob.output.packetFormat == 0);
    CHECK(ctx, std::strlen(blob.device.deviceName) > 0);
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
    testFiniteHelpers(ctx);
    return ctx.finish("test_config_schema_detail");
}

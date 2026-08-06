#include "test_common.hpp"

#include <cstdint>
#include <limits>

#include "network/wifi_remote_console_lease.hpp"

using namespace tracker;

int main() {
    TestContext ctx;
    WifiRemoteConsoleSessionLease lease;

    CHECK(ctx, !lease.active());
    CHECK(ctx, !lease.expired(100u, 30u));

    lease.begin(100u);
    CHECK(ctx, lease.active());
    CHECK(ctx, lease.ageMs(129u) == 29u);
    CHECK(ctx, !lease.expired(129u, 30u));
    CHECK(ctx, lease.expired(130u, 30u));

    lease.noteActivity(125u);
    CHECK(ctx, !lease.expired(154u, 30u));
    CHECK(ctx, lease.expired(155u, 30u));

    const uint32_t nearWrap = std::numeric_limits<uint32_t>::max() - 10u;
    lease.begin(nearWrap);
    CHECK(ctx, lease.ageMs(9u) == 20u);
    CHECK(ctx, lease.expired(9u, 20u));

    lease.reset();
    CHECK(ctx, !lease.active());
    CHECK(ctx, lease.ageMs(999u) == 0u);

    return ctx.finish("wifi_remote_console_lease");
}

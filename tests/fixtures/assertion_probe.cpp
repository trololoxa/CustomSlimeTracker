#include "test_common.hpp"
#include <limits>

// Each expected failure runs in a child process: diagnostics and exit status
// are checked by test_dev01_test_infrastructure.py, not by the helper itself.
int main(int argc, char** argv) {
    if (argc != 2) return 2;
    TestContext ctx;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const float max = std::numeric_limits<float>::max();
    switch (std::atoi(argv[1])) {
    case 0: CHECK(ctx, true); break;
    case 1: CHECK_NEAR(ctx, 1.0f, 1.0f, 0.0f); break;
    case 2: CHECK_NEAR(ctx, 1.5f, 1.0f, 0.5f); break;
    case 3: CHECK_NEAR(ctx, -0.0f, 0.0f, -0.0f); break;
    case 4: CHECK_NEAR(ctx, max, max, 0.0f); break;
    case 5: CHECK(ctx, false); break;
    case 6: CHECK_NEAR(ctx, 1.0f, 2.0f, 0.1f); break;
    case 7: CHECK_NEAR(ctx, nan, 1.0f, 0.1f); break;
    case 8: CHECK_NEAR(ctx, 1.0f, nan, 0.1f); break;
    case 9: CHECK_NEAR(ctx, 1.0f, 2.0f, nan); break;
    case 10: CHECK_NEAR(ctx, inf, inf, 0.0f); break;
    case 11: CHECK_NEAR(ctx, 1.0f, inf, 0.1f); break;
    case 12: CHECK_NEAR(ctx, 1.0f, 2.0f, inf); break;
    case 13: CHECK_NEAR(ctx, 1.0f, 1.0f, -0.1f); break;
    case 14: CHECK_NEAR(ctx, -inf, 1.0f, 0.1f); break;
    case 15: CHECK_NEAR(ctx, 1.0f, -inf, 0.1f); break;
    case 16: CHECK_NEAR(ctx, 1.0f, 1.0f, -inf); break;
    case 17: CHECK_NEAR(ctx, max, -max, max); break;
    case 18: CHECK_NEAR(ctx, 1.5f, 1.0f, std::nextafter(0.5f, 0.0f)); break;
    default: return 2;
    }
    return ctx.finish("assertion_probe");
}

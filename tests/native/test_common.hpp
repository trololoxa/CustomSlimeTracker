#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

struct TestContext {
    int failures = 0;

    void check(bool condition, const char* expr, const char* file, int line) {
        if (!condition) {
            ++failures;
            std::fprintf(stderr, "FAIL %s:%d: %s\n", file, line, expr);
        }
    }

    void checkNear(float actual, float expected, float tolerance, const char* expr, const char* file, int line) {
        if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
            ++failures;
            std::fprintf(stderr,
                         "FAIL %s:%d: %s actual=%.9g expected=%.9g tolerance=%.9g\n",
                         file,
                         line,
                         expr,
                         static_cast<double>(actual),
                         static_cast<double>(expected),
                         static_cast<double>(tolerance));
        }
    }

    int finish(const char* testName) const {
        if (failures == 0) {
            std::printf("PASS %s\n", testName);
            return 0;
        }
        std::fprintf(stderr, "FAIL %s failures=%d\n", testName, failures);
        return 1;
    }
};

#define CHECK(ctx, expr) (ctx).check((expr), #expr, __FILE__, __LINE__)
#define CHECK_NEAR(ctx, actual, expected, tolerance) \
    (ctx).checkNear((actual), (expected), (tolerance), #actual, __FILE__, __LINE__)

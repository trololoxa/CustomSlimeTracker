#include <cstdlib>

// Prove ASan itself in the combined mode: UBSan object-size checking must not
// intercept this deliberate fault first. Only this fixture helper excludes UB
// instrumentation; undefined.cpp independently proves UBSan with the same flags.
__attribute__((noinline, no_sanitize("undefined")))
static void writeByte(volatile char* p, int index) {
    p[index] = 1;
}

// argv selects a fault at runtime; volatile access prevents dead-store removal.
int main(int argc, char**) {
    volatile char* p = static_cast<char*>(std::malloc(16));
    if (!p) return 2;
    writeByte(p, argc > 1 ? 16 : 0);
    std::free(const_cast<char*>(p));
    return 0;
}

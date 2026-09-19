#include <cstdlib>

// Do not retain the pointer in main's stack frame at the exit-time leak check.
__attribute__((noinline)) static void allocate(bool leak) {
    volatile char* p = static_cast<char*>(std::malloc(17));
    if (!p) std::exit(2);
    p[0] = 1;
    if (!leak) std::free(const_cast<char*>(p));
}

int main(int argc, char**) {
    allocate(argc > 1);
    return 0;
}

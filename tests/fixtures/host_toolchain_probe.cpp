#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

int main() {
    const float one = std::bit_cast<float>(uint32_t{0x3f800000});
    volatile float nan = std::numeric_limits<float>::quiet_NaN();
    std::vector<int> values{1, 2, 3};
    if (one != 1.0f || std::isfinite(nan) || values.at(2) != 3) return 1;
    std::printf("TRACKER_HOST_CXX20_OK pointer_bits=%u\n", unsigned(sizeof(void*) * 8));
    return 0;
}

#pragma once

#include <cstdint>

namespace tracker {

// Deterministic integer avalanche for bounded replay-friendly reservoirs.
// The caller supplies a subsystem-specific salt so independent collectors do
// not make identical replacement decisions for the same sample sequence.
inline uint32_t deterministicReservoirWord(uint32_t index, uint32_t salt) {
    uint32_t x = index + salt;
    x ^= x >> 16;
    x *= 0x85EBCA6Bu;
    x ^= x >> 13;
    x *= 0xC2B2AE35u;
    x ^= x >> 16;
    return x;
}

inline uint32_t deterministicReservoirCandidate(uint32_t zeroBasedIndex,
                                                uint32_t seenCount,
                                                uint32_t salt) {
    return seenCount == 0u
        ? 0u
        : deterministicReservoirWord(zeroBasedIndex, salt) % seenCount;
}

} // namespace tracker

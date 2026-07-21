#pragma once

#include <cstddef>
#include <cstdint>

#ifndef SPI_MODE0
#define SPI_MODE0 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

class SPISettings {
public:
    SPISettings(uint32_t = 1000000u, uint8_t = MSBFIRST, uint8_t = SPI_MODE0) {}
};

class SPIClass {
public:
    void begin(int = -1, int = -1, int = -1, int = -1) {}
    void beginTransaction(const SPISettings&) {}
    void endTransaction() {}
    uint8_t transfer(uint8_t value) { return value; }
    void transferBytes(const uint8_t* tx, uint8_t* rx, size_t len) {
        for (size_t i = 0; i < len; ++i) rx[i] = tx ? tx[i] : 0u;
    }
};

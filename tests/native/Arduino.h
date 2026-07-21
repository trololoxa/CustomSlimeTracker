#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

#ifndef HEX
#define HEX 16
#endif

inline uint32_t millis() {
    return 0u;
}

inline uint32_t& trackerTestMicrosStorage() {
    static uint32_t value = 0u;
    return value;
}

inline void trackerTestSetMicros(uint32_t value) {
    trackerTestMicrosStorage() = value;
}

inline void trackerTestAdvanceMicros(uint32_t delta) {
    trackerTestMicrosStorage() += delta;
}

inline uint32_t micros() {
    return trackerTestMicrosStorage();
}

inline void noInterrupts() {}
inline void interrupts() {}
inline void yield() {}
inline void delay(uint32_t) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}

#ifndef INPUT
#define INPUT 0
#endif
#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef LOW
#define LOW 0
#endif
#ifndef HIGH
#define HIGH 1
#endif

class Stream {
public:
    virtual ~Stream() = default;
    void begin(uint32_t) {}
    virtual int available() { return 0; }
    virtual int read() { return -1; }

    std::size_t print(const char* s) { return s ? std::printf("%s", s) : 0u; }
    std::size_t print(char c) { return std::printf("%c", c); }
    std::size_t print(bool v) { return print(v ? "1" : "0"); }
    std::size_t print(int v) { return std::printf("%d", v); }
    std::size_t print(unsigned int v) { return std::printf("%u", v); }
    std::size_t print(long v) { return std::printf("%ld", v); }
    std::size_t print(unsigned long v) { return std::printf("%lu", v); }
    std::size_t print(long long v) { return std::printf("%lld", v); }
    std::size_t print(unsigned long long v) { return std::printf("%llu", v); }
    std::size_t print(float v) { return std::printf("%f", static_cast<double>(v)); }
    std::size_t print(double v) { return std::printf("%f", v); }

    std::size_t print(float v, int decimals) { return printFixed(static_cast<double>(v), decimals); }
    std::size_t print(double v, int decimals) { return printFixed(v, decimals); }

    std::size_t print(uint8_t v, int base) { return printUnsignedBase(v, base); }
    std::size_t print(unsigned short v, int base) { return printUnsignedBase(v, base); }
    std::size_t print(unsigned int v, int base) { return printUnsignedBase(v, base); }
    std::size_t print(unsigned long v, int base) { return printUnsignedBase(v, base); }
    std::size_t print(unsigned long long v, int base) { return printUnsignedBase(v, base); }

    std::size_t println() { return std::printf("\n"); }

    template <typename T>
    std::size_t println(const T& v) {
        const std::size_t n = print(v);
        return n + println();
    }

    std::size_t println(float v, int decimals) {
        const std::size_t n = print(v, decimals);
        return n + println();
    }

    std::size_t println(double v, int decimals) {
        const std::size_t n = print(v, decimals);
        return n + println();
    }

    std::size_t println(uint8_t v, int base) {
        const std::size_t n = print(v, base);
        return n + println();
    }

    std::size_t println(unsigned short v, int base) {
        const std::size_t n = print(v, base);
        return n + println();
    }

    std::size_t println(unsigned int v, int base) {
        const std::size_t n = print(v, base);
        return n + println();
    }

    std::size_t println(unsigned long v, int base) {
        const std::size_t n = print(v, base);
        return n + println();
    }

    std::size_t println(unsigned long long v, int base) {
        const std::size_t n = print(v, base);
        return n + println();
    }

private:
    static std::size_t printFixed(double v, int decimals) {
        if (decimals < 0) decimals = 0;
        if (decimals > 9) decimals = 9;
        char buf[64];
        switch (decimals) {
            case 0: std::snprintf(buf, sizeof(buf), "%.0f", v); break;
            case 1: std::snprintf(buf, sizeof(buf), "%.1f", v); break;
            case 2: std::snprintf(buf, sizeof(buf), "%.2f", v); break;
            case 3: std::snprintf(buf, sizeof(buf), "%.3f", v); break;
            case 4: std::snprintf(buf, sizeof(buf), "%.4f", v); break;
            case 5: std::snprintf(buf, sizeof(buf), "%.5f", v); break;
            case 6: std::snprintf(buf, sizeof(buf), "%.6f", v); break;
            case 7: std::snprintf(buf, sizeof(buf), "%.7f", v); break;
            case 8: std::snprintf(buf, sizeof(buf), "%.8f", v); break;
            default: std::snprintf(buf, sizeof(buf), "%.9f", v); break;
        }
        return std::printf("%s", buf);
    }

    static std::size_t printUnsignedBase(unsigned long long v, int base) {
        if (base == HEX) {
            return std::printf("%llX", v);
        }
        return std::printf("%llu", v);
    }
};

extern Stream Serial;

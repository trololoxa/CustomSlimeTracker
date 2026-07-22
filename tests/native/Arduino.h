#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

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
    virtual int peek() { return -1; }
    virtual int availableForWrite() { return 1 << 20; }
    virtual void flush() {}

    virtual std::size_t write(uint8_t value) {
        return std::fwrite(&value, 1u, 1u, stdout);
    }

    virtual std::size_t write(const uint8_t* data, std::size_t len) {
        return data && len > 0u ? std::fwrite(data, 1u, len, stdout) : 0u;
    }

    std::size_t print(const char* s) { return s ? write(reinterpret_cast<const uint8_t*>(s), std::strlen(s)) : 0u; }
    std::size_t print(char c) { return write(static_cast<uint8_t>(c)); }
    std::size_t print(bool v) { return print(v ? "1" : "0"); }
    std::size_t print(int v) { char b[32]; return writeNumber(b, std::snprintf(b, sizeof(b), "%d", v)); }
    std::size_t print(unsigned int v) { char b[32]; return writeNumber(b, std::snprintf(b, sizeof(b), "%u", v)); }
    std::size_t print(long v) { char b[32]; return writeNumber(b, std::snprintf(b, sizeof(b), "%ld", v)); }
    std::size_t print(unsigned long v) { char b[32]; return writeNumber(b, std::snprintf(b, sizeof(b), "%lu", v)); }
    std::size_t print(long long v) { char b[32]; return writeNumber(b, std::snprintf(b, sizeof(b), "%lld", v)); }
    std::size_t print(unsigned long long v) { char b[32]; return writeNumber(b, std::snprintf(b, sizeof(b), "%llu", v)); }
    std::size_t print(float v) { return printFixed(static_cast<double>(v), 2); }
    std::size_t print(double v) { return printFixed(v, 2); }

    std::size_t print(float v, int decimals) { return printFixed(static_cast<double>(v), decimals); }
    std::size_t print(double v, int decimals) { return printFixed(v, decimals); }

    std::size_t print(uint8_t v, int base) { return printUnsignedBase(v, base); }
    std::size_t print(unsigned short v, int base) { return printUnsignedBase(v, base); }
    std::size_t print(unsigned int v, int base) { return printUnsignedBase(v, base); }
    std::size_t print(unsigned long v, int base) { return printUnsignedBase(v, base); }
    std::size_t print(unsigned long long v, int base) { return printUnsignedBase(v, base); }

    std::size_t println() { return print('\n'); }

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
    std::size_t writeNumber(const char* buf, int n) {
        if (!buf || n <= 0) return 0u;
        const std::size_t len = static_cast<std::size_t>(n) < 64u
            ? static_cast<std::size_t>(n)
            : 63u;
        return write(reinterpret_cast<const uint8_t*>(buf), len);
    }

    std::size_t printFixed(double v, int decimals) {
        if (decimals < 0) decimals = 0;
        if (decimals > 9) decimals = 9;
        char buf[64];
        int n = 0;
        switch (decimals) {
            case 0: n = std::snprintf(buf, sizeof(buf), "%.0f", v); break;
            case 1: n = std::snprintf(buf, sizeof(buf), "%.1f", v); break;
            case 2: n = std::snprintf(buf, sizeof(buf), "%.2f", v); break;
            case 3: n = std::snprintf(buf, sizeof(buf), "%.3f", v); break;
            case 4: n = std::snprintf(buf, sizeof(buf), "%.4f", v); break;
            case 5: n = std::snprintf(buf, sizeof(buf), "%.5f", v); break;
            case 6: n = std::snprintf(buf, sizeof(buf), "%.6f", v); break;
            case 7: n = std::snprintf(buf, sizeof(buf), "%.7f", v); break;
            case 8: n = std::snprintf(buf, sizeof(buf), "%.8f", v); break;
            default: n = std::snprintf(buf, sizeof(buf), "%.9f", v); break;
        }
        return writeNumber(buf, n);
    }

    std::size_t printUnsignedBase(unsigned long long v, int base) {
        char buf[32];
        const int n = base == HEX
            ? std::snprintf(buf, sizeof(buf), "%llX", v)
            : std::snprintf(buf, sizeof(buf), "%llu", v);
        return writeNumber(buf, n);
    }
};


class ESPClass {
public:
    void restart() {}
};

extern Stream Serial;
extern ESPClass ESP;

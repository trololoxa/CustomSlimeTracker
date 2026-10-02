#pragma once
class FakeWifiAdapter final : public IWifiStationAdapter {
public:
    WifiStationInfo current;
    bool begin(const char*, const char*, const char*) override { current.linkStatus = WifiLinkStatus::Connecting; return true; }
    void disconnect() override { current.linkStatus = WifiLinkStatus::Disconnected; }
    WifiStationInfo info() const override { return current; }
    int16_t scanNetworks(WifiScanResult*, uint8_t, bool) override { return 0; }
    void clearScanResults() override {}
};

class FakeUdp final : public IUdpTransport {
public:
    struct SentPacket { UdpEndpoint endpoint; std::vector<uint8_t> data; };

    bool beginResult = true;
    bool isActive = false;
    uint16_t port = 0;
    uint32_t beginCalls = 0;
    uint32_t stopCalls = 0;
    std::vector<SentPacket> sent;
    std::vector<uint8_t> incoming;
    UdpEndpoint incomingRemote;
    bool incomingPending = false;
    int failPacketType = -1;
    int failSendError = ENOMEM;
    int lastError = 0;
    uint32_t sendCalls = 0;
    bool resolveResult = true;
    uint32_t resolvedIpv4 = 0xC0A8002AUL;
    uint32_t resolveCalls = 0;
    char lastResolvedHost[64] = {};

    bool begin(uint16_t localPort) override {
        ++beginCalls;
        if (!beginResult) return false;
        isActive = true;
        port = localPort;
        return true;
    }
    void stop() override { ++stopCalls; isActive = false; port = 0; }
    bool active() const override { return isActive; }
    uint16_t localPort() const override { return port; }

    bool resolveHost(const char* host, uint32_t& outIpv4) override {
        ++resolveCalls;
        std::strncpy(lastResolvedHost, host ? host : "", sizeof(lastResolvedHost) - 1u);
        lastResolvedHost[sizeof(lastResolvedHost) - 1u] = '\0';
        outIpv4 = resolveResult ? resolvedIpv4 : 0u;
        return resolveResult;
    }

    bool send(const UdpEndpoint& endpoint, const uint8_t* data, size_t len) override {
        ++sendCalls;
        lastError = 0;
        if (!isActive || !data || len == 0) {
            lastError = EINVAL;
            return false;
        }
        if (failPacketType >= 0 && len >= 4u && data[3] == static_cast<uint8_t>(failPacketType)) {
            lastError = failSendError;
            return false;
        }
        SentPacket p;
        p.endpoint = endpoint;
        p.data.assign(data, data + len);
        sent.push_back(p);
        return true;
    }
    int lastSendError() const override { return lastError; }

    int parsePacket() override { return incomingPending ? static_cast<int>(incoming.size()) : 0; }
    int read(uint8_t* data, size_t maxLen) override {
        if (!incomingPending || !data) return 0;
        const size_t n = incoming.size() < maxLen ? incoming.size() : maxLen;
        std::memcpy(data, incoming.data(), n);
        incomingPending = false;
        return static_cast<int>(n);
    }
    UdpEndpoint remoteEndpoint() const override { return incomingRemote; }
};

struct FakeSnapshotSource {
    TrackerPreparedOutputSnapshot snapshot;

    static bool copy(TrackerPreparedOutputSnapshot& out, void* user) {
        auto* self = static_cast<FakeSnapshotSource*>(user);
        out = self->snapshot;
        return out.valid;
    }
};



static uint32_t readU32BeLocal(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
}

static uint64_t readU64BeLocal(const uint8_t* p) {
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value = (value << 8) | static_cast<uint64_t>(p[i]);
    }
    return value;
}

static float readF32BeLocal(const uint8_t* p) {
    union { uint32_t u; float f; } v{};
    v.u = readU32BeLocal(p);
    return v.f;
}


static std::vector<uint8_t> makeServerPacket(uint8_t type, std::initializer_list<uint8_t> payload) {
    std::vector<uint8_t> packet(12, 0);
    packet[3] = type;
    packet.insert(packet.end(), payload.begin(), payload.end());
    return packet;
}

static std::vector<uint8_t> makeSensorInfoAck(uint8_t sensorId, uint8_t status) {
    return {
        0x00, 0x00, 0x00, static_cast<uint8_t>(SlimeVRReceivePacketType::SensorInfo),
        sensorId, status
    };
}

struct FakeConfigFlagSink {
    uint32_t calls = 0;
    uint8_t sensorId = 0;
    uint16_t configType = 0;
    bool enabled = false;
    bool result = true;

    static bool apply(uint8_t sensorId, uint16_t configType, bool enabled, void* user) {
        auto* self = static_cast<FakeConfigFlagSink*>(user);
        ++self->calls;
        self->sensorId = sensorId;
        self->configType = configType;
        self->enabled = enabled;
        return self->result;
    }
};

static TrackerWifiManagerConfig wifiConfig() {
    TrackerWifiManagerConfig cfg;
    cfg.enabled = true;
    cfg.credentialsValid = true;
    cfg.ssid = "test";
    cfg.password = "pw";
    cfg.hostname = "tracker";
    cfg.connectTimeoutMs = 1000;
    cfg.statusPollIntervalMs = 10;
    return cfg;
}

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

// Minimal host-side Preferences/NVS substitute used only by native tests.
class Preferences {
public:
    bool begin(const char* name, bool readOnly = false) {
        if (!name || !*name) return false;
        namespace_ = name;
        readOnly_ = readOnly;
        open_ = true;
        return true;
    }

    void end() {
        open_ = false;
        namespace_.clear();
        readOnly_ = false;
    }

    std::size_t getBytesLength(const char* key) const {
        const auto* value = lookup(key);
        return value ? value->size() : 0u;
    }

    std::size_t getBytes(const char* key, void* out, std::size_t maxLen) const {
        const auto* value = lookup(key);
        if (!value || !out) return 0u;
        const std::size_t count = std::min(maxLen, value->size());
        if (count != 0u) std::memcpy(out, value->data(), count);
        return count;
    }

    std::size_t putBytes(const char* key, const void* data, std::size_t len) {
        if (!open_ || readOnly_ || !key || !*key || (!data && len != 0u)) return 0u;
        auto& value = storage()[qualifiedKey(key)];
        const auto* bytes = static_cast<const uint8_t*>(data);
        value.assign(bytes, bytes + len);
        return len;
    }

    bool remove(const char* key) {
        if (!open_ || readOnly_ || !key || !*key) return false;
        return storage().erase(qualifiedKey(key)) != 0u;
    }

    static void clearTestStorage() { storage().clear(); }

private:
    using Value = std::vector<uint8_t>;
    using Storage = std::unordered_map<std::string, Value>;

    static Storage& storage() {
        static Storage values;
        return values;
    }

    std::string qualifiedKey(const char* key) const {
        return namespace_ + "\n" + (key ? key : "");
    }

    const Value* lookup(const char* key) const {
        if (!open_ || !key || !*key) return nullptr;
        const auto it = storage().find(qualifiedKey(key));
        return it == storage().end() ? nullptr : &it->second;
    }

    std::string namespace_;
    bool readOnly_ = false;
    bool open_ = false;
};

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
        if (nextBeginFailures() != 0u) {
            --nextBeginFailures();
            return false;
        }
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


    bool isKey(const char* key) const {
        return lookup(key) != nullptr;
    }

    std::size_t getBytesLength(const char* key) const {
        const auto* value = lookup(key);
        return value ? value->size() : 0u;
    }

    std::size_t getBytes(const char* key, void* out, std::size_t maxLen) const {
        if (targetGetKey() == qualifiedKey(key) && targetGetFailures() != 0u) {
            --targetGetFailures();
            if (targetGetFailures() == 0u) targetGetKey().clear();
            return 0u;
        }
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
        std::size_t written = len;
        if (targetPutKey() == qualifiedKey(key)) {
            written = std::min(written, targetPutLimit());
            targetPutKey().clear();
            targetPutLimit() = kNoPutLimit;
        } else if (nextPutLimit() != kNoPutLimit) {
            written = std::min(written, nextPutLimit());
            nextPutLimit() = kNoPutLimit;
        }
        value.assign(bytes, bytes + written);
        return written;
    }

    bool remove(const char* key) {
        if (!open_ || readOnly_ || !key || !*key) return false;
        if (targetRemoveKey() == qualifiedKey(key)) {
            targetRemoveKey().clear();
            return false;
        }
        return storage().erase(qualifiedKey(key)) != 0u;
    }

    static void clearTestStorage() {
        storage().clear();
        nextPutLimit() = kNoPutLimit;
        targetPutKey().clear();
        targetPutLimit() = kNoPutLimit;
        targetGetKey().clear();
        targetGetFailures() = 0u;
        targetRemoveKey().clear();
        nextBeginFailures() = 0u;
    }

    static void setNextPutLimit(std::size_t limit) { nextPutLimit() = limit; }

    static void setPutLimitForKey(const char* name,
                                  const char* key,
                                  std::size_t limit) {
        targetPutKey() = std::string(name ? name : "") + "\n" + (key ? key : "");
        targetPutLimit() = limit;
    }

    static void setGetFailuresForKey(const char* name,
                                     const char* key,
                                     std::size_t count = 1u) {
        targetGetKey() = std::string(name ? name : "") + "\n" + (key ? key : "");
        targetGetFailures() = count;
    }

    static void setRemoveFailureForKey(const char* name, const char* key) {
        targetRemoveKey() = std::string(name ? name : "") + "\n" + (key ? key : "");
    }

    static void setNextBeginFailures(std::size_t count = 1u) {
        nextBeginFailures() = count;
    }

    static void putTestBytes(const char* name,
                             const char* key,
                             const void* data,
                             std::size_t len) {
        const std::string qualified = std::string(name ? name : "") + "\n" + (key ? key : "");
        const auto* bytes = static_cast<const uint8_t*>(data);
        storage()[qualified].assign(bytes, bytes + len);
    }

    static bool corruptTestByte(const char* name,
                                const char* key,
                                std::size_t offset,
                                uint8_t xorMask = 0xFFu) {
        const std::string qualified = std::string(name ? name : "") + "\n" + (key ? key : "");
        auto it = storage().find(qualified);
        if (it == storage().end() || offset >= it->second.size()) return false;
        it->second[offset] ^= xorMask;
        return true;
    }

    static std::vector<uint8_t> getTestBytes(const char* name, const char* key) {
        const std::string qualified = std::string(name ? name : "") + "\n" + (key ? key : "");
        const auto it = storage().find(qualified);
        return it == storage().end() ? std::vector<uint8_t>{} : it->second;
    }

private:
    using Value = std::vector<uint8_t>;
    using Storage = std::unordered_map<std::string, Value>;
    static constexpr std::size_t kNoPutLimit = static_cast<std::size_t>(-1);

    static std::size_t& nextPutLimit() {
        static std::size_t limit = kNoPutLimit;
        return limit;
    }

    static std::string& targetPutKey() {
        static std::string key;
        return key;
    }

    static std::size_t& targetPutLimit() {
        static std::size_t limit = kNoPutLimit;
        return limit;
    }

    static std::string& targetGetKey() {
        static std::string key;
        return key;
    }

    static std::size_t& targetGetFailures() {
        static std::size_t count = 0u;
        return count;
    }

    static std::string& targetRemoveKey() {
        static std::string key;
        return key;
    }

    static std::size_t& nextBeginFailures() {
        static std::size_t count = 0u;
        return count;
    }

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

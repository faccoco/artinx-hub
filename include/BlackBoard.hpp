#pragma once
#include "DataDesc.hpp"
#include <any>
#include <cstdint>
#include <optional>
#include <shared_mutex>
#include <unordered_map>
#include <utility>

namespace detail {
    struct Hashed final {
        constexpr size_t operator()(size_t hashValue) const noexcept {
            return hashValue;
        }
    };
}  // namespace detail

class BlackBoard final {
    std::unordered_map<size_t, std::pair<std::shared_mutex, std::any>> mItems;
    std::shared_mutex mMutex;

    void insertImpl(size_t hashValue, std::any val);
    std::pair<std::shared_mutex, std::any>* getImpl(size_t hashValue);

public:
    // TODO: type safe
    template <typename T>
    std::optional<T> get(Identifier key) {
        if(auto ptr = getImpl(typeid(T).hash_code() ^ key.val)) {
            std::shared_lock<std::shared_mutex> guard{ ptr->first };
            return std::any_cast<T>(ptr->second);
        }
        return std::nullopt;
    }

    // TODO: type safe
    template <typename T>
    void updateSync(Identifier key, T val) {
        const auto hashCode = typeid(T).hash_code() ^ key.val;
        if(auto ptr = getImpl(hashCode)) {
            std::lock_guard<std::shared_mutex> guard{ ptr->first };
            ptr->second = std::move(val);
        } else
            insertImpl(hashCode, std::move(val));
    }

    static BlackBoard& instance();
};

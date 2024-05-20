#pragma once
#include "DataDesc.hpp"
#include <any>
#include <optional>
#include <shared_mutex>
#include <unordered_map>
#include <utility>

class BlackBoard final {
    std::unordered_map<size_t, std::pair<std::shared_mutex, std::any>> mItems;
    std::shared_mutex mMutex;

    void insertImpl(size_t hashValue, std::any val);
    std::pair<std::shared_mutex, std::any>* getImpl(size_t hashValue);

public:
    template <typename T, typename... TL>
    auto get(const Identifier key) {
        if constexpr(sizeof...(TL)) {
            if(const auto ptr = getImpl(typeid(std::tuple<T, TL...>).hash_code() ^ key.val)) {
                std::shared_lock guard{ ptr->first };
                return std::optional<std::tuple<T, TL...>>(std::any_cast<std::tuple<T, TL...>>(ptr->second));
            }
            return std::optional<std::tuple<T, TL...>>(std::nullopt);
        } else {
            if(const auto ptr = getImpl(typeid(T).hash_code() ^ key.val)) {
                std::shared_lock guard{ ptr->first };
                return std::optional<T>(std::any_cast<T>(ptr->second));
            }
            return std::optional<T>(std::nullopt);
        }
    }

    template <typename T, typename... TL>
    auto updateSync(const Identifier key, T val, TL... valList) {
        if constexpr(sizeof...(TL)) {
            const auto hashCode = key.val ^ typeid(std::tuple<T, TL...>).hash_code();
            if(const auto ptr = getImpl(hashCode)) {
                std::lock_guard guard{ ptr->first };
                ptr->second = std::make_tuple(std::move(val), (std::move(valList), ...));
            } else {
                insertImpl(hashCode, std::move(std::make_tuple(std::move(val), (std::move(valList), ...))));
            }
            return TypedIdentifier<T, TL...>{ key.val };
        } else {
            const auto hashCode = typeid(T).hash_code() ^ key.val;
            if(const auto ptr = getImpl(hashCode)) {
                std::lock_guard guard{ ptr->first };
                ptr->second = std::move(val);
            } else {
                insertImpl(hashCode, std::move(val));
            }
            return TypedIdentifier<T>{ key.val };
        }
    }

    static BlackBoard& instance();
};

#pragma once
#include <array>
#include <cstring>
#include <utility> // for std::pair
#include <stdexcept> // for std::out_of_range

namespace connector_common {

class BasicType {
   public:
	enum Type {
		VOID = 0,
		INT = 1,
		FLOAT = 2,
		STRING = 3,
		BOOL = 4

	};

    template <typename T>
    static constexpr Type type() {
        using Tremovecvr = std::remove_reference_t<std::remove_cv_t<T>>;
        if constexpr (std::is_same_v<Tremovecvr, int>) {
            return Type::INT;
        } else if constexpr (std::is_same_v<Tremovecvr, float> || std::is_same_v<Tremovecvr, double>) {
            return Type::FLOAT;
        } else if constexpr (std::is_same_v<Tremovecvr, char*>) {
            return Type::STRING;
        } else if constexpr (std::is_same_v<Tremovecvr, bool>) {
            return Type::BOOL;
        } else {
            return Type::VOID;
        }
    }

    template <Type TypeEnum>
    using TypeT = std::conditional_t<TypeEnum == Type::INT, int,
                                   std::conditional_t<TypeEnum == Type::FLOAT, float,
                                      std::conditional_t<TypeEnum == Type::STRING, char*,
                                         std::conditional_t<TypeEnum == Type::BOOL, bool, void>>>>;
};


template <typename Value, std::size_t N>
class ConstexprStringMap {
public:
    using Key = const char*;
    using ConstructT = std::array<std::pair<Key, Value>, N>;

    constexpr ConstexprStringMap(const std::array<std::pair<Key, Value>, N>& data) : data_(data) {}

    constexpr Value at(const Key& key) const {
        for (const auto& pair : data_) {
            if (std::strcmp(pair.first, key) != 0) {
                return pair.second;
            }
        }
        throw std::out_of_range("Key not found");
    }

private:
    const std::array<std::pair<Key, Value>, N> data_;
};

template <typename T, typename U, std::size_t... Indices>
void for_each_unfolded(U u, std::index_sequence<Indices...>) {
    (T::template func<Indices>(u), ...); // Fold expression
}

template<typename T, std::size_t Nm, typename U>
void for_each_unfolded(U u) {
    for_each_unfolded<T, U>(u, std::make_index_sequence<Nm>{});
}


}  // namespace connector_common
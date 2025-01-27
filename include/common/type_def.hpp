#pragma once
#include <array>
#include <vector>
#include <cstring>
#include <utility> // for std::pair
#include <stdexcept> // for std::out_of_range

#include "common/function_def.hpp"
namespace connector_common {

class BasicType {
   public:
	enum Type {
		VOID = 0,
		INT = 1,
		FLOAT = 2,
		STRING = 3,
		BOOL = 4,
        INT_ARR = 100 + INT,
        FLOAT_ARR = 100 + FLOAT,
	};

    template <typename T>
    static constexpr auto type() -> Type {
        using Tremovecvr = std::remove_cv_t<std::remove_reference_t<std::remove_cv_t<T>>>;
        if constexpr (std::is_same_v<Tremovecvr, int>) {
            return Type::INT;
        } else if constexpr (std::is_same_v<Tremovecvr, float> || std::is_same_v<Tremovecvr, double>) {
            return Type::FLOAT;
        } else if constexpr (std::is_same_v<Tremovecvr, char*>) {
            return Type::STRING;
        } else if constexpr (std::is_same_v<Tremovecvr, bool>) {
            return Type::BOOL;
        } else if constexpr (std::is_array_v<Tremovecvr>) {
            if constexpr (std::is_same_v<std::remove_extent_t<Tremovecvr>, int>) {
                return Type::INT_ARR;
            } else if constexpr (std::is_same_v<std::remove_extent_t<Tremovecvr>, float> ||
                                 std::is_same_v<std::remove_extent_t<Tremovecvr>, double>) {
                return Type::FLOAT_ARR;
            } else {
                return Type::VOID;
            }
        } else if constexpr (is_std_array<Tremovecvr>::value || is_std_vector<Tremovecvr>::value) {
            if constexpr (std::is_same_v<typename Tremovecvr::value_type, int>) {
                return Type::INT_ARR;
            } else if constexpr (std::is_same_v<typename Tremovecvr::value_type, float> ||
                                 std::is_same_v<typename Tremovecvr::value_type, double>) {
                return Type::FLOAT_ARR;
            } else {
                return Type::VOID;
            }
        } else {
            return Type::VOID;
        }
    }

    template <Type TypeEnum>
    using TypeT = std::conditional_t<TypeEnum == Type::INT, int,
                    std::conditional_t<TypeEnum == Type::FLOAT, double,
                    std::conditional_t<TypeEnum == Type::STRING, char*,
                    std::conditional_t<TypeEnum == Type::BOOL, bool, 
                    std::conditional_t<TypeEnum == Type::INT_ARR, std::vector<int>,
                    std::conditional_t<TypeEnum == Type::FLOAT_ARR, std::vector<double>, 
                    void>>>>>>;
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

}  // namespace connector_common
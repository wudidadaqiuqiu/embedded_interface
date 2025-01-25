#pragma once
#include <array>
#include <vector>
#include <tuple>
#include <sstream>
#include <cstring>
#include <utility> // for std::pair
#include <stdexcept> // for std::out_of_range

namespace connector_common {

template <typename T>
struct is_std_array : std::false_type {};

// Specialization for std::array
template <typename T, std::size_t N>
struct is_std_array<std::array<T, N>> : std::true_type {};


template <typename T>
struct is_std_vector : std::false_type {};

template <typename T>
struct is_std_vector<std::vector<T>> : std::true_type {};

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

template <typename T, typename U, std::size_t... Indices>
void for_each_unfolded(U u, std::index_sequence<Indices...>) {
    (T::template func<Indices>(u), ...); // Fold expression
}

template<typename T, std::size_t Nm, typename U>
void for_each_unfolded(U u) {
    for_each_unfolded<T, U>(u, std::make_index_sequence<Nm>{});
}

template <std::size_t Index, std::size_t Count = 0, typename... Args>
// constexpr 代表编译时获取，auto& 代表获取引用，且不所谓类型 
inline constexpr auto& tuple_get(Args&... args) {
    static_assert(Index < sizeof...(args), "Index out of range");
    if constexpr (Index == Count) {
        // 绑定引用用tie 不用std::make_tuple
        return std::get<Index>(std::tie(args...));
    } else {
        // 模板递归实现if else if，无法通过index_sequence实现
        return tuple_get<Index, Count+1>(args...);
    }
}

template <typename T>
inline auto to_string(const T& mat) -> std::string{
    std::stringstream ss;
    if constexpr (is_std_vector<T>::value || is_std_array<T>::value) {
        ss << "[";
        for (const auto& item : mat) {
            ss << to_string(item) << ", ";
        }
        ss << "]";
    } else {
        ss << mat;
    }
    return ss.str();
}


}  // namespace connector_common
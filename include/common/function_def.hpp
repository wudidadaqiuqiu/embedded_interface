#pragma once
#include <functional>
#include <utility>
#include <array>
#include <string>
#include <vector>
#include <tuple>
#include <sstream>
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

// https://stackoverflow.com/questions/39199564/constexpr-c-string-concatenation-parameters-used-in-a-constexpr-context
template<unsigned...>struct seq{using type=seq;};
template<unsigned N, unsigned... Is>
struct gen_seq_x : gen_seq_x<N-1, N-1, Is...>{};
template<unsigned... Is>
struct gen_seq_x<0, Is...> : seq<Is...>{};
template<unsigned N>
using gen_seq=typename gen_seq_x<N>::type;

template<std::size_t S>
using size=std::integral_constant<std::size_t, S>;

template<class T, std::size_t N>
constexpr size<N> length( T const(&)[N] ) { return {}; }
template<class T, std::size_t N>
constexpr size<N> length( std::array<T, N> const& ) { return {}; }

template<class T>
using length_t = decltype(length(std::declval<T>()));

constexpr std::size_t string_size() { return 0; }
template<class...Ts>
constexpr std::size_t string_size( std::size_t i, Ts... ts ) {
  return (i?i-1:0) + string_size(ts...);
}
template<class...Ts>
using string_length=size< string_size( length_t<Ts>{}... )>;

template<class...Ts>
using combined_string = std::array<char, string_length<Ts...>{}+1>;

template<class Lhs, class Rhs, unsigned...I1, unsigned...I2>
constexpr const combined_string<Lhs,Rhs>
concat_impl( Lhs const& lhs, Rhs const& rhs, seq<I1...>, seq<I2...>)
{
    return {{ lhs[I1]..., rhs[I2]..., '\0' }};
}

template<class Lhs, class Rhs>
constexpr const combined_string<Lhs,Rhs>
concat(Lhs const& lhs, Rhs const& rhs)
{
    return concat_impl(lhs, rhs, gen_seq<string_length<Lhs>{}>{}, gen_seq<string_length<Rhs>{}>{});
}

template<class T0, class T1, class... Ts>
constexpr const combined_string<T0, T1, Ts...>
concat(T0 const&t0, T1 const&t1, Ts const&...ts)
{
    return concat(t0, concat(t1, ts...));
}

template<class T>
constexpr const combined_string<T>
concat(T const&t) {
    return concat(t, "");
}
constexpr const combined_string<>
concat() {
    return concat("");
}


template <typename T, typename U, std::size_t... Indices>
void for_each_unfolded(U u, std::index_sequence<Indices...>) {
    (T::template func<Indices>(u), ...); // Fold expression
}

template<typename T, std::size_t Nm, typename U>
void for_each_unfolded(U u) {
    for_each_unfolded<T, U>(u, std::make_index_sequence<Nm>{});
}

template <std::size_t Index, std::size_t Max,
    template <std::size_t, std::size_t> typename ConditionFuncT,
    template <std::size_t, std::size_t> typename ReturnFuncT, std::size_t Count=0>
static constexpr auto for_each_conditional_return(auto&... params) -> auto {
    static_assert(Index < Max, "Index out of range");
    static_assert(Count < Max, "Index out of range");
    if constexpr (ConditionFuncT<Index, Count>::func()) {
        return ReturnFuncT<Index, Count>::func(params...);
    } else {
        return for_each_conditional_return<Index, Max, ConditionFuncT, ReturnFuncT, Count + 1>(params...);
    }
}


template <std::size_t Index, std::size_t Count = 0, typename... Args>
// constexpr 代表编译时获取，auto& 代表获取引用，且不所谓类型 
inline constexpr auto& tie_get(Args&... args) {
    static_assert(Index < sizeof...(args), "Index out of range");
    // if constexpr (Index == Count) {
    //     // 绑定引用用tie 不用std::make_tuple
    //     return std::get<Index>(std::tie(args...));
    // } else {
    //     // 模板递归实现if else if，无法通过index_sequence实现
    //     return tuple_get<Index, Count+1>(args...);
    // }
    return  std::get<Index>(std::tie(args...));
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


template <typename T>
struct count_elements_t {
    static constexpr size_t value = 1;
};

// 特化模板：用于 std::tuple 类型，递归计算元素个数
template <typename... Ts>
struct count_elements_t<std::tuple<Ts...>> {
    static constexpr size_t value = (count_elements_t<Ts>::value + ...);  // 使用折叠表达式递归计算
};

template <std::size_t Index, typename TupleT>
static constexpr auto get_range_pair() -> std::pair<std::size_t, std::size_t> {
    static_assert(Index < std::tuple_size_v<TupleT>, "Index out of range");
    if constexpr (Index == 0) {
        return {0, count_elements_t<typename std::tuple_element<0, TupleT>::type>::value};
    }
    if constexpr (Index > 0) {
        return {get_range_pair<Index - 1, TupleT>().second, get_range_pair<Index - 1, TupleT>().second +
            count_elements_t<typename std::tuple_element<Index, TupleT>::type>::value};            
    }
}


// https://stackoverflow.com/questions/11019232/how-can-i-specialize-a-c-template-for-a-range-of-integer-values
template<bool> struct If;

constexpr auto in_closed_range(std::size_t Value, std::size_t Lower, std::size_t Upper) -> bool {
    return (Lower <= Value) && (Value < Upper);
}


template<std::size_t Index, std::size_t Count, typename TupleT> 
struct InRange {
    static constexpr auto func() -> bool {
        return get_range_pair<Count, TupleT>().first <= Index && Index < get_range_pair<Count, TupleT>().second;
    }
};

// type traits
template<typename...Ts>
using tuple_cat_t = decltype(std::tuple_cat(std::declval<Ts>()...));

// https://stackoverflow.com/questions/23855712/how-can-a-type-be-removed-from-a-template-parameter-pack
// https://stackoverflow.com/questions/53394100/concatenating-tuples-as-types
template <std::size_t H, typename T> struct get_lower {};

template <typename T, typename... Ts>
struct get_lower<0, std::tuple<T, Ts...>> {
  using type = std::tuple<>;
};

template <std::size_t H, typename T, typename... Ts>
struct get_lower<H, std::tuple<T, Ts...>> {
  static_assert(H <= sizeof...(Ts) + 1, "Index out of bounds");
  using type = decltype(std::tuple_cat(
      std::declval<std::tuple<T>>(),
      std::declval<typename get_lower<H - 1, std::tuple<Ts...>>::type>()));
};

template <std::size_t H> struct get_lower<H, std::tuple<>> {
  using type = std::tuple<>;
};

template <std::size_t L, typename T> struct get_upper {};

template <typename T, typename... Ts>
struct get_upper<0, std::tuple<T, Ts...>> {
  using type = std::tuple<T, Ts...>;
};

template <std::size_t L, typename T, typename... Ts>
struct get_upper<L, std::tuple<T, Ts...>> {
  static_assert(L <= sizeof...(Ts) + 1, "Index out of bounds");
  using type = decltype(std::tuple_cat(
      std::declval<typename get_upper<L - 1, std::tuple<Ts...>>::type>()));
};

template <> struct get_upper<0, std::tuple<>> {
  using type = std::tuple<>;
};

template <std::size_t L, std::size_t H, typename... Ts> struct get_range {
  static_assert(L <= H, "Invalid range");
  using type =
      get_upper<L, typename get_lower<H, std::tuple<Ts...>>::type>::type;
};

template <std::size_t L, std::size_t H, typename... Ts>
struct get_range<L, H, std::tuple<Ts...>> {
  static_assert(L <= H, "Invalid range");
  using type =
      get_upper<L, typename get_lower<H, std::tuple<Ts...>>::type>::type;
};

template <std::size_t L, std::size_t H, typename... Ts>
using get_range_t = get_range<L, H, Ts...>::type;

template <std::size_t M>
auto split(const std::array<char, M>& arr, char delimiter) -> std::vector<std::vector<char>> {
    std::vector<std::vector<char>> result;
    std::vector<char> current;
    for (char c : arr) {
        if (c == delimiter) {
            if (!current.empty()) {
                result.push_back(current);  // Push the current part
                current.clear();  // Clear for the next part
            }
        } else if (c == '\0') {
            break;
        } else {
            current.push_back(c);  // Add the character to the current part
        }
    }
    
    if (!current.empty()) {
        result.push_back(current);  // Push the last part if any
    }
    return result;
}
}
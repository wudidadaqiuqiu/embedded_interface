#pragma once
#include <cstddef>
#include "common/type_def.hpp"
#include "common/function_def.hpp"

namespace connector_common {
using connector_common::BasicType;
using connector_common::InRange;
using connector_common::get_pair_impl_t;
using connector_common::get_range_pair;
using connector_common::get_lower;
using connector_common::count_elements_t;
using connector_common::for_each_conditional_return;

template <typename T, bool B=false>
struct tuple_convert {
    using type = T;
};

template <typename T>
struct tuple_convert<T, true> {
    using type = decltype(typename T::Config{}.param_interface())::ParamDeclare::Params;
};

template<std::size_t Index, std::size_t Count, typename TupleT>
struct PairReturn {
    static constexpr auto func(const auto& prefix, auto& self) {
        return get_pair_impl_t<Index - get_range_pair<Count, TupleT>().first, 
                BasicType::type<decltype(self.template get_ele<Count>())>() == BasicType::Type::VOID>::
                get_pair_impl(prefix,
                    self.template get_ele<Count>(), 
                    self.template get_names_tuple<Count>());
    }
};

template<typename... Args>
struct ParamDeclarationGen {
    static constexpr auto gen(Args&&... args) {
        return ParamDeclarationGen<Args...>{std::forward<Args>(args)...};
    }
    
    using Params = std::tuple<
        typename tuple_convert<Args, BasicType::type<Args>() == BasicType::Type::VOID>::type...>;

    // get_ele 和 get_names_tuple 是interface
    template<std::size_t Index, std::size_t Count>
    using SpecializationPairReturn = PairReturn<Index, Count, Params>;

    template<std::size_t Index, std::size_t Count>
    using SpecializationInRange = InRange<Index, Count, Params>;
};

template <typename... Args>
struct ParamDeclarationGen<std::tuple<Args...>> : public ParamDeclarationGen<Args...> {};


template <typename... Args>
struct ParamsInterface {
    std::tuple<Args&...> refs;
    constexpr ParamsInterface(Args&... args) : refs(std::tie(args...)) {}
    using ParamDeclare = ParamDeclarationGen<typename get_lower<sizeof...(Args) / 2, std::tuple<Args...>>::type>;
    static constexpr std::size_t PARAMS_COUNT =  count_elements_t<typename ParamDeclare::Params>::value;
    template <std::size_t Index>
    // 意味着函数的结果能在编译时获得，结果本身无所谓编译时还是运行时
    constexpr auto get_ele() -> auto& {
        return  std::get<Index>(refs);
    }

    template <std::size_t Index>
    constexpr auto get_names_tuple() -> auto const& {
        return std::get<Index + sizeof...(Args) / 2>(refs);
        // (std::tie("model", "q_mat", "r_mat"));
    }

    template<std::size_t Index>
    constexpr auto index_params(const auto& prefix) {
        static_assert(Index < PARAMS_COUNT, "Index out of range");
        return for_each_conditional_return<Index, 
            PARAMS_COUNT, 
            ParamDeclare::template SpecializationInRange, 
            ParamDeclare::template SpecializationPairReturn>(prefix, *this);
    }
};


template <std::size_t Nm, typename T>
struct PairHint {
    std::pair<std::array<char, Nm>, std::reference_wrapper<T>> pair_;
    PairHint(std::pair<std::array<char, Nm>, std::reference_wrapper<T>> pair) : pair_(pair) {}

    // 不写成constexpr了，太麻烦
    auto equal_first_namespace(const std::string_view& name_arr) -> bool {
        // first 为 "prefix.first_name_space?.." `prefix` 不能为空
        auto res = split(pair_.first, '.');
        if (res.size() < 2) {
            throw std::runtime_error("prefix is empty");
        }
        return std::string(res[1].begin(), res[1].end()) == name_arr;
    }

    auto get_name() -> std::string {
        if (pair_.first.at(pair_.first.size() - 1) != '\0') {
            throw std::runtime_error(R"(pair_.first is not '\0' terminated)");
        }
        return std::string(pair_.first.begin(), pair_.first.end() - 1);
    }

    auto get_value() -> T& {
        return pair_.second.get();
    }
    
    constexpr auto get_enum() -> BasicType::Type {
        return BasicType::type<T>();
    }

    using ValueT = BasicType::TypeT<BasicType::type<T>()>;
};

}
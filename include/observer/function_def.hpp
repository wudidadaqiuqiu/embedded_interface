#pragma once
#include <cstddef>
#include "common/type_def.hpp"
#include "common/function_def.hpp"

namespace observer {
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
    using type = T::Config::ParamsT;
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
    ParamsInterface(Args&... args) : refs(std::tie(args...)) {}
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

}
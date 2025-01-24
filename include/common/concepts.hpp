#pragma once

#if __cplusplus == 202002L
namespace connector_common {
template<typename T>
concept has_get_structure_data_method = requires(T& a) {
    {a.get_structure_data()};
};

template<typename T>
concept has_get_struct_data_method = requires(T& a) {
    {a.get_struct_data()};
};

template <typename T, typename... U>
concept HasType = requires {
    typename T::Type; // 检查 T 是否有普通的嵌套类型 Type
} || requires {
    typename T::template Type<U...>; // 检查 T 是否有模板 Type<U>
};


template <typename T, typename... U>
concept HasConfig = requires {
    typename T::Config; // 检查 T 是否有普通的嵌套类型 Type
} || requires {
    typename T::template Config<U...>; // 检查 T 是否有模板 Type<U>
};

template <typename T, typename... U>
concept IsPackOfObjects = requires {
    HasType<T, U...>;
    HasConfig<T, U...>;
};

#endif
}
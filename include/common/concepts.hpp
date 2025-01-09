#pragma once
#include <concepts>


namespace connector_common {
template<typename T>
concept has_get_structure_data_method = requires(T& a) {
    {a.get_structure_data()};
};

template<typename T>
concept has_get_struct_data_method = requires(T& a) {
    {a.get_struct_data()};
};

}
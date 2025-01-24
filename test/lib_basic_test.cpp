#include "common/type_def.hpp"

using connector_common::BasicType;

inline constexpr void test_types() {
    static_assert(BasicType::type<int>() == BasicType::Type::INT, "INT test failed");
    static_assert(BasicType::type<float>() == BasicType::Type::FLOAT, "FLOAT test failed");
    static_assert(BasicType::type<double>() == BasicType::Type::FLOAT, "DOUBLE test failed");
    static_assert(BasicType::type<char*>() == BasicType::Type::STRING, "STRING test failed");
    static_assert(BasicType::type<bool>() == BasicType::Type::BOOL, "BOOL test failed");
    static_assert(BasicType::type<int[5]>() == BasicType::Type::INT_ARR, "INT array test failed");
    static_assert(BasicType::type<std::array<float, 3>>() == BasicType::Type::FLOAT_ARR, "std::array<float> test failed");
    static_assert(BasicType::type<std::vector<int>>() == BasicType::Type::INT_ARR, "std::vector<int> test failed");
    static_assert(BasicType::type<void>() == BasicType::Type::VOID, "VOID test failed");
}

int main() {

}
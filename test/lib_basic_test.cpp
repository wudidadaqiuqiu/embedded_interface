#include <type_traits>
#include "common/function_def.hpp"
#include "common/type_def.hpp"
#include "observer/observer.hpp"
#include "observer/kf_observer.hpp"

using connector_common::BasicType;

using observer::ObserverType;
using observer::Observer;
using connector_common::count_elements_t;



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

using Kf = observer::KalmanFilter<1, 0, 1>;
using connector_common::get_lower;
using connector_common::get_upper;
using connector_common::get_range;
using connector_common::get_range_t;

static Kf::Config config;
static Kf kf(config);
int main() {
    std::cout << kf.config.model.config.get_pair<0>("pre").first.data() << std::endl;
    static_assert(BasicType::type<decltype(
        kf.config.get_pair<0>("pre").second.get())>() == 
        BasicType::Type::FLOAT, 
        "not compile time");
    std::cout << kf.config.get_pair<0>("pre").first.data() << std::endl;
    std::cout << kf.config.get_pair<1>("pre").first.data() << std::endl;
    std::cout << kf.config.get_pair<2>("pre").first.data() << std::endl;
    std::cout << kf.config.get_pair<3>("pre").first.data() << std::endl;
    
    using TestType = std::tuple<int, float, std::tuple<int, char>, double>;
    std::cout << "Number of elements in test_type: " 
              << count_elements_t<TestType>::value << std::endl;
    static_assert(count_elements_t<TestType>::value == 5, "count_elements_t failed");
    // static_assert(count_elements_t<Kf::Config::ParamDeclare::Params>::value == 4, "count_elements_t failed");
    static_assert(count_elements_t<int>::value == 1, "basic type is not 1");
    kf.config.get_pair<0>("pre").second.get() = 2;
    std::cout << kf.config.get_pair<0>("pre").second << std::endl;
    // std::cout << connector_common::concat("pre").data() << std::endl;
    // std::cout << kf.config.get_pair<0>("pre").second << std::endl;

    using my_tuple = std::tuple<int, bool, double>;
    using ss = get_lower<3, my_tuple>::type;

    static_assert(
        std::is_same<ss, std::tuple<int, bool, double>>::value, 
        "Error!"
    );
    using ss2 = get_upper<3, my_tuple>::type;
    static_assert(
        std::is_same<ss2, std::tuple<>>::value, 
        "Error!"
    );
    static_assert(
        std::is_same<get_range<0, 3, ss>::type, ss>::value,
        "Error!"
    );
    static_assert(
        std::is_same<get_range<0, 2, ss>::type, std::tuple<int, bool>>::value,
        "Error!"
    );
    static_assert(
        std::is_same<get_range<0, 2, int, bool, double>::type, std::tuple<int, bool>>::value,
        "Error!"
    );
    static_assert(
        std::is_same<get_range<1, 2, ss>::type, std::tuple<bool>>::value,
        "Error!"
    );

    static_assert(
        std::is_same<get_range<2, 2, ss>::type, std::tuple<>>::value,
        "Error!"
    );

}
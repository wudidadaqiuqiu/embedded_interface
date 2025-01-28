#include <iostream>
#include <type_traits>
#include "common/function_def.hpp"
#include "common/type_def.hpp"
#include "observer/observer.hpp"
#include "observer/kf_observer.hpp"

using connector_common::BasicType;

using observer::ObserverType;
using observer::Observer;
using connector_common::count_elements_t;
using connector_common::concat;

using connector_common::ParamDeclarationGen;
using connector_common::ParamsInterface;
using connector_common::to_string;
struct TestS {
    struct Config {
        float a;

        constexpr auto param_interface() {
            // return ParamsInterface<typename Args>
            return ParamsInterface();
        }

        template <std::size_t Index>
        void set(const auto& value) {
            // param_interface().template set<Index>(value);
        }
    } config;
};
struct Test2S {
    struct Config {
        float a;
        TestS b;
        float c;
        constexpr auto param_interface() {
            // return ParamsInterface<typename Args>
            return ParamsInterface(a, b, c, "a1", "b1", "c1");
        }


        template <std::size_t Index>
        void set(const auto& value) {
            param_interface().template set<Index>(value);
        }
    } config;
};

struct DeclareAllParameters {
    template<std::size_t Index>
    static void func(auto* t) {
        auto pair_hint = t->config.param_interface().template index_param_hint<Index>(concat("_"));
        std::cout << to_string(pair_hint.get_name()) << " "  << std::endl;
        std::cout << to_string(typename decltype(pair_hint)::ValueT{}) << std::endl;
    }
};

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
    static_assert(BasicType::type<float&>() == BasicType::type<const double&>(), "type mismatch");
}

using Kf = observer::KalmanFilter<observer::StateSpaceModel<1, 0, 1>>;
using connector_common::get_lower;
using connector_common::get_upper;
using connector_common::get_range;
using connector_common::get_range_t;

static Kf::Config config;
static Kf kf(config);
int main() {
    // std::cout << kf.config.model.config.param_interface().template index_pair<0>(concat("pre")).first.data() << std::endl;
    // static_assert(BasicType::type<decltype(
    //     kf.config.param_interface().template index_pair<0>(concat("pre")).second.get())>() == 
    //     BasicType::Type::FLOAT, 
    //     "not compile time");
    std::cout << kf.config.param_interface().template index_pair<0>(concat("pre")).first.data() << std::endl;
    std::cout << kf.config.param_interface().template index_pair<1>(concat("pre")).first.data() << std::endl;
    // std::cout << kf.config.param_interface().template index_pair<2>(concat("pre")).first.data() << std::endl;
    // std::cout << kf.config.param_interface().template index_pair<3>(concat("pre")).first.data() << std::endl;
    
    using TestType = std::tuple<int, float, std::tuple<int, char>, double>;
    std::cout << "Number of elements in test_type: " 
              << count_elements_t<TestType>::value << std::endl;
    static_assert(count_elements_t<TestType>::value == 5, "count_elements_t failed");
    // static_assert(count_elements_t<Kf::Config::ParamDeclare::Params>::value == 4, "count_elements_t failed");
    static_assert(count_elements_t<int>::value == 1, "basic type is not 1");

    // kf.config.param_interface().template index_pair<1>(concat("pre")).second.get() = 2;
    // std::cout << kf.config.param_interface().template index_pair<1>(concat("pre")).second << std::endl;
    
    std::cout << connector_common::concat("pre").data() << std::endl;
    static_assert(concat(".", "pre") == concat(".pre"), "concat failed");

    std::array<char, 5> test = {'t', 'e', 's', 't', '\0'};
    test.at(1) = 'a';
    std::cout << concat(test, "post").data() << std::endl;
    
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
    



    
    // typename ParamDeclarationGen<TestS>::Params;
    // using type =
	// 	decltype(typename TestS::Config{}.param_interface())::ParamDeclare::Params;
    using TupleTestT = std::tuple<int, std::tuple<>, float>;
    
    constexpr auto s = connector_common::get_range_pair<1, TupleTestT>().first;
    static_assert(((s == connector_common::get_range_pair<1, TupleTestT>().second) == 1), "");
    static_assert(connector_common::get_range_pair<0, TupleTestT>().second == 1, "");
    // static_assert(s == connector_common::get_range_pair<2, TupleTestT>().second == 1, "");
    
    Test2S test2;
    {
    auto pair = test2.config.param_interface().template index_pair<0>(concat("_"));
    std::cout << to_string(pair.first) << std::endl;
    }
    {
    auto pair = test2.config.param_interface().template index_pair<1>(concat("_"));
    std::cout << to_string(pair.first) << std::endl;
    }
    static_assert(test2.config.param_interface().PARAMS_COUNT == 2, "");
    {
    auto pair = test2.config.param_interface().template index_param_hint<1>(concat("_"));
    std::cout << to_string(pair.get_name()) << std::endl;
    }
    {
    auto pair = test2.config.param_interface().template index_param_hint<1>(concat("_"));
    std::cout << to_string(pair.get_name()) << std::endl;
    }
    {
    auto pair = test2.config.param_interface().template index_param_hint<0>(concat("_"));
    std::cout << to_string(pair.get_name()) << std::endl;
    }
    test2.config.set<0>(1.0);

    test2.config.set<1>(1.0);

    // std::make_index_sequence<2>{};
    using connector_common::for_each_unfolded;
    for_each_unfolded<DeclareAllParameters, decltype(
        test2.config.param_interface())::PARAMS_COUNT>(&test2);
    
    // test2.config.set<2>(1.0);
    // auto pair = test2.config.param_interface().template index_pair<2>(concat("_"));
}
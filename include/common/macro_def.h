#pragma once

// https://stackoverflow.com/questions/1872220/is-it-possible-to-iterate-over-arguments-in-variadic-macros
#define CONCATENATE(arg1, arg2)   CONCATENATE1(arg1, arg2)
#define CONCATENATE1(arg1, arg2)  CONCATENATE2(arg1, arg2)
#define CONCATENATE2(arg1, arg2)  arg1##arg2

#define FOR_EACH_1(what, x, ...) what(x)
#define FOR_EACH_2(what, x, ...)\
  what(x),\
  FOR_EACH_1(what,  __VA_ARGS__)
#define FOR_EACH_3(what, x, ...)\
  what(x),\
  FOR_EACH_2(what, __VA_ARGS__)
#define FOR_EACH_4(what, x, ...)\
  what(x),\
  FOR_EACH_3(what,  __VA_ARGS__)
#define FOR_EACH_5(what, x, ...)\
  what(x),\
 FOR_EACH_4(what,  __VA_ARGS__)
#define FOR_EACH_6(what, x, ...)\
  what(x),\
  FOR_EACH_5(what,  __VA_ARGS__)
#define FOR_EACH_7(what, x, ...)\
  what(x),\
  FOR_EACH_6(what,  __VA_ARGS__)
#define FOR_EACH_8(what, x, ...)\
  what(x),\
  FOR_EACH_7(what,  __VA_ARGS__)

#define FOR_EACH_NARG(...) FOR_EACH_NARG_(__VA_ARGS__, FOR_EACH_RSEQ_N())
#define FOR_EACH_NARG_(...) FOR_EACH_ARG_N(__VA_ARGS__) 
#define FOR_EACH_ARG_N(_1, _2, _3, _4, _5, _6, _7, _8, N, ...) N 
#define FOR_EACH_RSEQ_N() 8, 7, 6, 5, 4, 3, 2, 1, 0

#define FOR_EACH_(N, what, x, ...) CONCATENATE(FOR_EACH_, N)(what, x, __VA_ARGS__)
#define FOR_EACH(what, x, ...) FOR_EACH_(FOR_EACH_NARG(x, __VA_ARGS__), what, x, __VA_ARGS__)


// 处理单个参数的宏
#define PAIR(param) { #param, BasicType::type<decltype(param)>() }

// 展开所有参数包的宏
#define EXPAND_PAIRS(...) PAIR(__VA_ARGS__)

// 用宏生成参数数据
#define DECLARE_PARAM_MAP_DATA(...) \
static constexpr ConstexprStringMap<BasicType::Type, FOR_EACH_NARG(__VA_ARGS__)>::ConstructT PARAM_MAP_DATA = {{ \
    FOR_EACH(PAIR, __VA_ARGS__) \
}};

#define DECLARE_SET_FUNCTION(...) \
template <std::size_t Index> \
constexpr void set(const auto& value) { \
    auto& v = tie_get<Index>(__VA_ARGS__); \
    v = value; \
}


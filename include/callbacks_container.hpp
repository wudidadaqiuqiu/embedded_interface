#pragma once
#include <functional>
#include <mutex>
#include <vector>

template <typename T>
class CallbacksContainer {
    mutable std::mutex func_list_mutex;                     // 用于保护 func_list_
    std::vector<std::function<void(const T&)>> func_list_;  // 存储回调函数的列表

   public:
    CallbacksContainer() = default;  // 默认构造函数

    // 注册一个新的回调函数
    void register_callback(std::function<void(const T&)> func) {
        std::unique_lock<std::mutex> lock(func_list_mutex);
        func_list_.push_back(func);  // 将回调函数添加到列表中
    }

    // 调用所有注册的回调函数
    void callback(const T& value) {
        std::unique_lock<std::mutex> lock(func_list_mutex);
        for (auto& func : func_list_) {
            func(value);  // 调用每一个回调函数
        }
    }
};

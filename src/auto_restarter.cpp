#include "emi_functional/auto_restarter.hpp"
#include <iostream>

// 模板构造函数定义要写在头文件里，否则外部无法实例化
// 这里实现析构函数和成员函数
void AutoRestarter::set(BoolFunc is_working,
              VoidFunc start,
              VoidFunc restart,
              std::chrono::milliseconds interval) {
    is_working_ = std::move(is_working);
    start_ = std::move(start);
    restart_ = std::move(restart);
    interval_ = interval;
    running_ = false;
}

AutoRestarter::~AutoRestarter() {
    stop();
}

void AutoRestarter::start() {
    if (running_) return;
    running_ = true;

    if (start_) start_();

    worker_ = std::thread([this]() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (running_) {
            if (!is_working_ || !is_working_()) {
                std::cout << "[AutoRestarter] detected failure, restarting...\n";
                if (restart_) restart_();
            }
            cv_.wait_for(lock, interval_, [this] { return !running_; });
        }
    });
}

void AutoRestarter::stop() {
    if (!running_) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

void AutoRestarter::trigger_restart() {
    if (restart_) restart_();
}

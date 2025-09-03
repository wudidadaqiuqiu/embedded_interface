#pragma once

#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>

/// AutoRestarter: 监控 is_working()，自动调用 restart()。
class AutoRestarter {
public:
    using BoolFunc = std::function<bool()>;
    using VoidFunc = std::function<void()>;
    void set(BoolFunc is_working,
              VoidFunc start,
              VoidFunc restart,
              std::chrono::milliseconds interval = std::chrono::milliseconds(1000));
    ~AutoRestarter();

    void start();
    void stop();

    /// 手动触发一次重启
    void trigger_restart();

private:
    BoolFunc is_working_;
    VoidFunc start_;
    VoidFunc restart_;
    std::chrono::milliseconds interval_;
    std::atomic<bool> running_;
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable cv_;
};

#pragma once
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace connector {
    
template <typename MSGPackT>
class PackManager {
    using MSGT = typename MSGPackT::MSGT;
    std::queue<MSGT> packs;
    std::thread thread;
    mutable std::mutex mutex;
    std::condition_variable cv;
    const std::function<void(const MSGT&)> func_;
    bool is_ended = false;
    
public:
    PackManager(std::function<void(const MSGT&)> func) : func_(func) {
        thread = std::thread(&PackManager::process_task, this);
    }
    ~PackManager() {
        is_ended = true;
        cv.notify_all();
        if (thread.joinable()) {
            thread.join();
        }
    }
    void push_pack(const MSGT& pack) {
        // std::cout << "queue push" << std::endl;
        {
            std::unique_lock<std::mutex> l(mutex);
            packs.push(pack);
        }
        cv.notify_all();
    }

    void process_task() {
        while (!is_ended) {
            MSGT pack;
            // std::cout << "in process" << std::endl;
            {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait(lock, [this]() { return this->is_ended || !this->packs.empty();});
                if (is_ended) return;
                // std::cout << "queue pop" << std::endl;
                pack = packs.front();
                packs.pop();
            }
            func_(pack);
        }
    }
};
}
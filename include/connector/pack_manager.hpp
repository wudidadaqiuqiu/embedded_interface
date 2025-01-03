#pragma once
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include "common/callbacks_container.hpp"

namespace connector {

using connector_common::CallbacksContainer;

template <typename MSGPackT>
class PackManager {
    using MSGT = typename MSGPackT::MSGT;
    std::queue<MSGT> packs;
    std::thread thread;
    mutable std::mutex mutex;
    std::condition_variable cv;
    CallbacksContainer<MSGT> callbacks_;
    bool is_ended = false;

   public:
    PackManager() {
        thread = std::thread(&PackManager::process_task, this);
    }
    ~PackManager() {
        is_ended = true;
        cv.notify_all();
        if (thread.joinable()) {
            thread.join();
        }
    }

    void register_callback(std::function<void(const MSGT&)> func) {
        callbacks_.register_callback(func);
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
                cv.wait(lock, [this]() { return this->is_ended || !this->packs.empty(); });
                if (is_ended) return;
                // std::cout << "queue pop" << std::endl;
                pack = packs.front();
                packs.pop();
            }
            callbacks_.callback(pack);
        }
    }
};
}  // namespace connector
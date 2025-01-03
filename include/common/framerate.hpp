#pragma once
#include <chrono>

namespace connector_common {

// 帧率计算
struct Framerate {
    uint32_t count;
    std::chrono::high_resolution_clock::time_point last;
    double fps;
    Framerate() : last(std::chrono::high_resolution_clock::now()){}
    void update() {
        count ++;
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - last);
        if (duration.count() > 1000) {
            fps = count * 1000.0 / duration.count();
            count = 0;
            last = currentTime;
        }
    }
};

}
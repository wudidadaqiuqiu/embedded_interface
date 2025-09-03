#include "emi_functional/auto_restarter.hpp"
#include <iostream>

int main() {
    bool working = true;
	AutoRestarter restarter;
    restarter.set(
        [&]() { return working; },               // is_working
        [&]() { std::cout << "Start!\n"; },      // start
        [&]() { std::cout << "Restart!\n"; },    // restart
        std::chrono::milliseconds(1000)
    );

    restarter.start();

    std::this_thread::sleep_for(std::chrono::seconds(3));
    working = false;  // 模拟失败

    std::this_thread::sleep_for(std::chrono::seconds(5));
    restarter.stop();
}

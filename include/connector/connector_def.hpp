#pragma once
#include <string>
namespace connector {

enum ConnectorType {
    CAN = 0,
    TTY = 1
};

enum BaudRate {
    BAUD_1M,
    BAUD_115200
};

template <ConnectorType CON_TYPE>
class Connector {
};

class TimeoutException : public std::exception {
   private:
    std::string message;  // 异常信息

   public:
    // 构造函数，接受错误消息
    TimeoutException(const std::string& msg) : message(msg) {}

    // 重写 what() 方法，返回错误信息
    const char* what() const noexcept override {
        return message.c_str();
    }
};


}  // namespace connector
#pragma once
#include "connector/connector_def.hpp"
#include <cstdint>
#include <string>
#include <cerrno>
#include <cstring>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <condition_variable>
#include <mutex>
#include "common/debug/log.hpp"
namespace connector {

template <>
class Connector<ConnectorType::TTY> {
   private:
    int fd = -1;
    std::string file_path_;
    std::condition_variable cv;
    mutable std::mutex mutex;
    bool is_ended = false;
    // uint8_t buf[64];

   public:
    void con_open(std::string file_path, BaudRate baud_rate) {
        fd = ::open(file_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        fcntl(fd, F_SETFL, 0);
        if (fd < 0) {
            throw std::runtime_error("Unable to open tty " + 
                file_path + " " + std::string(strerror(errno)));
        }

        termios newtio{};
        switch (baud_rate) {
        case BaudRate::BAUD_1M:
            newtio.c_cflag = B1000000 | CS8 | CLOCAL | CREAD;
            break;
        case BaudRate::BAUD_115200:
            newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
            break;
        default:
            throw std::runtime_error("Unsupported baud rate ");
            break;
        }
        
        newtio.c_iflag = 0;
        newtio.c_oflag = 0;
        newtio.c_lflag = 0;
        newtio.c_cc[VTIME] = 0; // 设置阻塞
        newtio.c_cc[VMIN] = 1;   // 至少需要读取一个字节
        // 刷新串口数据，确保配置生效
        tcflush(fd, TCIOFLUSH);
        // 设置串口属性
        if (tcsetattr(fd, TCSANOW, &newtio) != 0) {
            throw std::runtime_error("Unable to set serial port attributes: " + std::string(strerror(errno)));
        }
        LOG_INFO(1, "Open tty %s success", file_path.c_str());
        // std::cout << "Open tty " << file_path << " success" << std::endl;
        cv.notify_all();
    }

    ~Connector() {
        is_ended = true;
        con_close();
    }

    void con_close() {
        if (fd < 0) 
            return;
        fcntl(fd, F_SETFL, O_NONBLOCK);
        termios old;
        tcgetattr(fd, &old);
        old.c_cc[VMIN] = 0;
        tcsetattr(fd, TCSANOW, &old);
        // std::cout << "close tty" << std::endl;
        LOG_INFO(1, "close tty");
        ::close(fd);
        fd = -1;
        cv.notify_all();
    }

    void con_send(const std::vector<uint8_t>& data, uint32_t id) {
        (void)id;
        // if (fd == -1) return false;
        if (::write(fd, data.data(), data.size()) < 0) {
            throw std::runtime_error("Error send to " + file_path_ +
                " " + std::string(strerror(errno)));
        }
    }

    void con_recv(std::vector<uint8_t>& data, uint32_t& id) {
        // (void)id;
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [this]() { return this->is_ended || this->fd >= 0;});
        if (is_ended) return;
        // std::cout << "con_recv" << std::endl;
        int nbytes = ::read(fd, data.data(), data.size());
        // LOG_INFO(1, "nbytes: %d", nbytes);
        // 断开连接时，收到0个字节
        if (nbytes <= 0) {
            // TODO 其他情况测试
            if (errno == EAGAIN) {
                throw TimeoutException("conector " + file_path_ + " timeout");
            }
            throw std::runtime_error("Error receiving " + file_path_ + 
                " " + std::string(strerror(errno)) + std::to_string(errno));
        }
        id = nbytes;
    }
};


}
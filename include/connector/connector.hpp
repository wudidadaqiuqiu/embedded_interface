#pragma once
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <vector>

#include <iostream>

namespace connector {
    

enum ConnectorType {
    CAN = 0,
    SERIAL = 1
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


template <>
class Connector<ConnectorType::CAN> {
    private:
        int sockfd;
        std::string can_interface_name_;
public:
    Connector(std::string can_interface_name) : can_interface_name_(can_interface_name) {
        sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sockfd < 0) {
            throw std::runtime_error("Error opening socket " + std::string(strerror(errno)));
        }

        // 获取 CAN 设备接口索引
        struct ifreq ifr;
        std::memset(&ifr, 0, sizeof(struct ifreq));
        std::strncpy(ifr.ifr_name, can_interface_name.c_str(), sizeof(ifr.ifr_name) - 1);  // 使用 can0 接口
        if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) {
            close(sockfd);
            throw std::runtime_error("Error getting interface index " + std::string(strerror(errno)));
        }

        // 绑定套接字到 CAN 接口
        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(struct sockaddr_can));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_can)) < 0) {
            close(sockfd);
            throw std::runtime_error("Error binding socket " + std::string(strerror(errno)));
        }

        // 只能在这里设置超时
        set_timeout(sockfd, 500);
    }

    ~Connector() {
        close(sockfd);
    }

    void con_close() {
        close(sockfd);
    }

    void con_send(const std::vector<uint8_t>& data, uint32_t id) {
        // 准备一个 CAN 帧发送
        struct can_frame frame;
        frame.can_id = id;  // CAN ID
        frame.can_dlc = data.size();  // 数据长度
        memcpy(frame.data, data.data(), data.size());

        // 发送 CAN 帧
        if (send(sockfd, &frame, sizeof(struct can_frame), 0) < 0) {
            // std::cerr << "Error sending CAN frame: " << strerror(errno) << std::endl;
            // close(sockfd);
            throw std::runtime_error("Error sending CAN frame " + std::string(strerror(errno)));
        }
    }

    void con_recv(std::vector<uint8_t>& data, uint32_t& id) {
        // 接收 CAN 帧
        struct can_frame rcv_frame;
        int nbytes = recv(sockfd, &rcv_frame, sizeof(struct can_frame), 0);
        if (nbytes < 0) {
            // std::cerr << "Error receiving CAN frame: " << strerror(errno) << std::endl;
            if (errno == EAGAIN) {
                throw TimeoutException("conector " + can_interface_name_ + " timeout");
            }
            throw std::runtime_error("Error receiving CAN frame " + std::string(strerror(errno)) + std::to_string(errno));
        }

        id = rcv_frame.can_id;
        data.resize(rcv_frame.can_dlc);
        memcpy(data.data(), rcv_frame.data, rcv_frame.can_dlc);
        // std::cout << "Received CAN frame with ID: " << std::hex << rcv_frame.can_id << std::dec << std::endl;
        // std::cout << "Data: ";
        // for (int i = 0; i < rcv_frame.can_dlc; ++i) {
        //     std::cout << std::hex << static_cast<int>(rcv_frame.data[i]) << " ";
        // }
    }

    static int set_socket_nonblocking(int sockfd_) {
        // int flags = fcntl(sockfd_, F_GETFL, 0);  // 获取当前的文件描述符标志
        // if (flags == -1) {
        //     // perror("fcntl(F_GETFL) failed");
        //     return -1;
        // }
        
        // flags |= O_NONBLOCK;  // 设置 O_NONBLOCK 标志
        if (fcntl(sockfd_, F_SETFL, O_NONBLOCK) == -1) {
            // perror("fcntl(F_SETFL) failed");
            return -1;
        }
        std::cout << "Socket set to non-blocking mode" << std::endl;
        return 0;  // 设置成功
    }

    static void set_timeout(int sockfd_, int timeout_ms) {
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
    }

};

}
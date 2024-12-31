#include <iostream>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <fcntl.h>

int main() {
    // 创建一个 CAN 套接字
    int sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0) {
        std::cerr << "Error opening socket: " << strerror(errno) << std::endl;
        return 1;
    }

    // 获取 CAN 设备接口索引
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(struct ifreq));
    std::strncpy(ifr.ifr_name, "can0", sizeof(ifr.ifr_name) - 1);  // 使用 can0 接口
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error getting interface index: " << strerror(errno) << std::endl;
        close(sockfd);
        return 1;
    }

    // 绑定套接字到 CAN 接口
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(struct sockaddr_can));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_can)) < 0) {
        std::cerr << "Error binding socket: " << strerror(errno) << std::endl;
        close(sockfd);
        return 1;
    }

    // 准备一个 CAN 帧发送
    struct can_frame frame;
    frame.can_id = 0x123;  // CAN ID
    frame.can_dlc = 8;  // 数据长度
    std::memset(frame.data, 0, sizeof(frame.data));  // 清空数据

    // 填充数据
    frame.data[0] = 0xDE;
    frame.data[1] = 0xAD;
    frame.data[2] = 0xBE;
    frame.data[3] = 0xEF;

    // 发送 CAN 帧
    if (send(sockfd, &frame, sizeof(struct can_frame), 0) < 0) {
        std::cerr << "Error sending CAN frame: " << strerror(errno) << std::endl;
        close(sockfd);
        return 1;
    }

    std::cout << "CAN frame sent!" << std::endl;

    // 接收 CAN 帧
    struct can_frame rcv_frame;
    int nbytes = recv(sockfd, &rcv_frame, sizeof(struct can_frame), 0);
    if (nbytes < 0) {
        std::cerr << "Error receiving CAN frame: " << strerror(errno) << std::endl;
        close(sockfd);
        return 1;
    }

    std::cout << "Received CAN frame with ID: " << std::hex << rcv_frame.can_id << std::dec << std::endl;
    std::cout << "Data: ";
    for (int i = 0; i < rcv_frame.can_dlc; ++i) {
        std::cout << std::hex << static_cast<int>(rcv_frame.data[i]) << " ";
    }
    std::cout << std::dec << std::endl;

    // 关闭套接字
    close(sockfd);
    return 0;
}

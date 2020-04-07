#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>
#include <sstream>
#include <string>

in_addr_t get_robot_ip(int port) {
    in_addr_t robot_ip = INADDR_NONE;

    // 创建UDP套接字文件描述符
    int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(socket_fd < 0)
        std::cerr << "socket";

    // 设置UDP套接字的目标地址
    struct sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(target_addr));

    target_addr.sin_family = AF_INET;                // 使用IPV4地址
    target_addr.sin_port = htons(port);              // 端口
    target_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 任意IP地址

    // 绑定套接字和目标IP端口设置
    if(bind(socket_fd, (struct sockaddr *)&target_addr, sizeof(target_addr)) < 0)
        std::cerr << "bind error";

    // 接收缓冲区
    char buffer[1024];
    // 信息来源地址
    struct sockaddr_in source_addr;
    int length = sizeof(sockaddr_in);

    std::stringstream stream;
    std::string str, _template = "robot ip ";

    std::cout << "Waiting message from robot..." << std::endl;
    while(1) {
        // 接收到信息
        if (0 < recvfrom(socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&source_addr, (socklen_t *)&length)) {
            stream << buffer;
            str = stream.str();

            // 如果符合机器人广播IP信息的格式
            if (0 == str.compare(0, _template.length(), _template)) {
                std::cout << "Got robot's ip: " << str.substr(_template.length()) << std::endl;
                robot_ip = source_addr.sin_addr.s_addr;
                break;
            }
        }
    }

    close(socket_fd);

    return robot_ip;
}

int main() {
    in_addr_t robot_ip = get_robot_ip(40926);

    if (robot_ip != INADDR_NONE) {

    }

    return 0;
}
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
    /* socket_fd --- socket文件描述符 创建udp套接字*/  
    int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(socket_fd < 0)
        std::cerr << "socket";

    /* 将套接字和IP、端口绑定 */  
    struct sockaddr_in target_addr;
    int length = sizeof(target_addr);
    //每个字节都用0填充
    memset(&target_addr, 0, length);

    // 使用IPV4地址
    target_addr.sin_family = AF_INET; 
    // 端口
    target_addr.sin_port = htons(port);
    // 自动获取IP地址
    target_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    /* 绑定socket */
    if(bind(socket_fd, (struct sockaddr *)&target_addr, length) < 0)
        std::cerr << "bind error:";

    char buffer[1024];
    struct sockaddr_in source_addr;
    in_addr_t robot_ip;

    std::stringstream stream;
    std::string str, _template = "robot ip ";

    std::cout << "Waiting message from robot..." << std::endl;
    while(1) {
        if (0 < recvfrom(socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&source_addr, (socklen_t *)&length)) {
            stream << buffer;
            str = stream.str();
            if (0 == str.compare(0, _template.length(), _template)) {
                std::cout << "Got robot's ip: " << str.substr(_template.length()) << std::endl;
                robot_ip = source_addr.sin_addr.s_addr;
                return robot_ip;
            }
        }
    }

    close(socket_fd);

    return INADDR_NONE;
}

int main() {
    in_addr_t robot_ip = get_robot_ip(40926);

    return 0;
}
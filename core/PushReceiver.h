#pragma once

#include <iostream>
#include <string.h>
#include <string>
#include <arpa/inet.h>
#include <unistd.h>

#include <thread>
#include <functional>

namespace RoboMasterEP
{

class Robot;

class PushReceiver
{
private:
    in_addr_t ip;
    int port;
    int _udp_socket = -1;

    const int BUFFER_LENGTH = 1024;
    
    char *receive_buffer;

    bool connect_via_udp();

    bool on = true;

    // to store address information of received push_receiver
    struct sockaddr_in receive_addr;
    int socket_length = sizeof(sockaddr_in);
    std::string push;
    std::string chassis_push = "chassis push ";
    std::string gimbal_push = "gimbal push ";

public:
    Robot *robot;

    PushReceiver(Robot *robot, in_addr_t ip, int port);
    ~PushReceiver();

    void receive();
};

};

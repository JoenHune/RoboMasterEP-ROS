#pragma once

#include <iostream>
#include <string.h>
#include <string>
#include <arpa/inet.h>
#include <unistd.h>

#include <thread>
#include <functional>
#include <mutex>

namespace RoboMasterEP
{

class Robot;

class PushReceiver
{
private:
    in_addr_t ip;
    int port;
    int _udp_socket = -1;

    // receive buffer related
    const int BUFFER_LENGTH = 1024;
    char *receive_buffer;

    // thread(s) control
    bool on = false;

    // to store address information of received push_receiver
    struct sockaddr_in receive_addr;
    int socket_length = sizeof(sockaddr_in);

    // messages control
    std::string push;
    std::string chassis_push = "chassis push ";
    std::string gimbal_push = "gimbal push ";

    // mutex
    std::mutex on_mutex;
    std::mutex close_mutex;

    // connection config
    bool connect_via_udp();

public:
    Robot *robot;

    PushReceiver(Robot *robot, in_addr_t ip, int port);
    ~PushReceiver();

    void receive();

    bool start();
    bool stop();
};

};

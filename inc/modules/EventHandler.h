#pragma once

#include <arpa/inet.h>

namespace RoboMasterEP
{

class Robot;

class EventHandler
{
private:
    in_addr_t ip;
    int port;
    int _tcp_socket = -1;

    const int BUFFER_LENGTH = 1024;
    
    char *receive_buffer;


public:
    Robot *robot;

    EventHandler(Robot *robot, in_addr_t ip, int port);
    ~EventHandler();
};

};

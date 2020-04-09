#pragma once

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <iostream>

#include "Controller.h"
#include "MessageReceiver.h"
#include "EventHandler.h"

class Robot
{
private:
    const int PORT_TCP_VEDIO     = 40921;
    const int PORT_TCP_AUDIO     = 40922;
    const int PORT_TCP_CONTROL   = 40923;
    const int PORT_UDP_MESSAGE   = 40924;
    const int PORT_TCP_EVENT     = 40925;
    const int PORT_UDP_BROADCAST = 40926;

    const int BUFFER_LENGTH = 1024;

    in_addr_t ip = INADDR_NONE;

    char *receive_buffer;

    // scan and require robot's IP address
    in_addr_t update_ip_via_udp(int port);

public:
    Controller *controller;
    MessageReceiver *messages;
    EventHandler *events;

    Robot();
    ~Robot();
};

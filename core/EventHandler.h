#pragma once

#include <arpa/inet.h>

class EventHandler
{
private:
    /* data */
public:
    EventHandler(in_addr_t ip, int port);
    ~EventHandler();
};

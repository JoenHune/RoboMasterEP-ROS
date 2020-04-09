#pragma once

#include <arpa/inet.h>

class MessageReceiver
{
private:
    /* data */
public:
    MessageReceiver(in_addr_t ip, int port);
    ~MessageReceiver();
};

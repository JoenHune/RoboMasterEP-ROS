#include <thread>
#include <functional>

#include "PushReceiver.h"

using namespace RoboMasterEP;

PushReceiver::PushReceiver(Robot *robot, in_addr_t ip, int port)
    : robot(robot), ip(ip), port(port)
{
    this->connect_via_udp();

    if (this->_udp_socket > 0)
    {
        std::clog << "[Info] Waiting push messages..." << std::endl;

        // allocate receive buffer for some control command which within response
        this->receive_buffer = new char[BUFFER_LENGTH];
        memset(this->receive_buffer, 0, BUFFER_LENGTH);

        this->on = false;
    }
}

PushReceiver::~PushReceiver()
{
    this->stop();

    close(this->_udp_socket);

    if (this->receive_buffer)
    {
        delete this->receive_buffer;
        receive_buffer = nullptr;
    }

    this->robot = nullptr;
}

bool PushReceiver::connect_via_udp()
{
    // create a file descriptor of udp socket
    this->_udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (this->_udp_socket < 0)
    {
        std::cerr << "[Error] Failed to create an udp socket to acquire robot's IP address" << std::endl;
        return false;
    }

    // setup udp socket's IP address and port
    struct sockaddr_in listen_addr;
    memset(&listen_addr, 0, sizeof(sockaddr_in));

    listen_addr.sin_family = AF_INET;           // using IPv4
    listen_addr.sin_port = htons(this->port);   // port
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);     // ip address

    // bind socket with IP address and port
    if (bind(this->_udp_socket, (struct sockaddr *)&listen_addr, sizeof(sockaddr_in)) < 0)
    {
        std::cerr << "[Error] Failed to bind the socket with the appointed IP address and port" << std::endl;
        return false;
    }

    return true;
}

void PushReceiver::receive()
{
    while (this->on)
    {
        // an udp message is received
        int l = recvfrom(this->_udp_socket, this->receive_buffer, BUFFER_LENGTH, 0, (struct sockaddr *)&(this->receive_addr), (socklen_t *)&(this->socket_length));

        if (l > chassis_push.length())
        {
            this->push = this->receive_buffer;
            this->push = this->push.substr(0, l);

            // if this->push is valid
            if (0 == this->push.compare(0, chassis_push.length(), chassis_push))
            {
                std::clog << "[Response] " << this->push << std::endl;
            }
            else if (0 == this->push.compare(0, gimbal_push.length(), gimbal_push))
            {
                std::clog << "[Response] " << this->push << std::endl;
            }
        }
    }
}

bool PushReceiver::start()
{
    if (this->on) return true;

    this->on = true;
    try
    {
        std::thread th(std::bind(&PushReceiver::receive, this));
        th.detach();
    }
    catch(const std::exception& e)
    {
        std::cerr << "[Error] Cannot create thread for push receiver: " << e.what() << '\n';
        return false;
    }
    return true;
}

bool PushReceiver::stop()
{
    this->on = false;
    return true;
}
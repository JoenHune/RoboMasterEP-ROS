#include "PushReceiver.h"

using namespace RoboMasterEP;

PushReceiver::PushReceiver(Robot *robot, in_addr_t ip, int port)
    : robot(robot), ip(ip), port(port)
{
    // allocate receive buffer for some control command which within response
    this->receive_buffer = new char[BUFFER_LENGTH];
    memset(this->receive_buffer, 0, BUFFER_LENGTH);

    this->connect_via_udp();
    std::clog << "[Info] Waiting push messages..." << std::endl;
}

PushReceiver::~PushReceiver()
{

    close(this->_udp_socket);

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
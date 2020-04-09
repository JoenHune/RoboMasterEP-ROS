#include "Robot.h"

Robot::Robot()
{
    // allocate receive buffer for UDP transmission
    this->receive_buffer = new char[BUFFER_LENGTH];
    memset(this->receive_buffer, 0, BUFFER_LENGTH);

    this->ip = update_ip_via_udp(PORT_UDP_BROADCAST);

    try
    {
        this->controller = new Controller(this->ip, PORT_TCP_CONTROL);
        this->messages = new MessageReceiver(this->ip, PORT_UDP_MESSAGE);
        this->events = new EventHandler(this->ip, PORT_TCP_EVENT);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

Robot::~Robot()
{
    if (controller) delete controller;

    if (messages) delete messages;

    if (events) delete events;

    // release receive buffer
    if (receive_buffer) delete receive_buffer;
}

in_addr_t Robot::update_ip_via_udp(int port)
{
    in_addr_t robot_ip = INADDR_NONE;

    // create a file descriptor of udp socket
    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0)
    {
        std::cerr << "[Error] Failed to create an udp socket to acquire robot's IP address" << std::endl;
        return robot_ip;
    }

    // setup udp socket's IP address and port
    struct sockaddr_in listen_addr;
    memset(&listen_addr, 0, sizeof(sockaddr_in));

    listen_addr.sin_family = AF_INET;                // using IPv4
    listen_addr.sin_port = htons(port);              // port
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 0.0.0.0(any address)

    // bind socket with IP address and port
    if (bind(udp_socket, (struct sockaddr *)&listen_addr, sizeof(sockaddr_in)) < 0)
    {
        std::cerr << "[Error] Failed to bind the socket with the appointed IP address and port";
        return robot_ip;
    }

    // to store address information of received messages
    struct sockaddr_in receive_addr;
    int socket_length = sizeof(sockaddr_in);

    std::string response, _template = "robot ip ";

    std::clog << "[Info] Waiting message from robot..." << std::endl;
    while (1)
    {
        // an udp message is received
        int l = recvfrom(udp_socket, this->receive_buffer, BUFFER_LENGTH, 0, (struct sockaddr *)&receive_addr, (socklen_t *)&(socket_length));

        if (l > _template.length())
        {
            response = this->receive_buffer;
            response = response.substr(0, l);

            // if response is valid
            if (0 == response.compare(0, _template.length(), _template))
            {
                std::clog << "[Response] " << response << std::endl;
                robot_ip = receive_addr.sin_addr.s_addr;
                break;
            }
            else continue;

        std::cout << "here4" << std::endl;
        }
    }

    close(udp_socket);

    return robot_ip;
}
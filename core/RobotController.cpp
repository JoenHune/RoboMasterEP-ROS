#include "RobotController.hpp"

RobotController::RobotController()
{
    // allocate receive buffer for some control command which within response
    this->receive_buffer = new char[BUFFER_LENGTH];
    memset(this->receive_buffer, 0, BUFFER_LENGTH);
    
    // scan robot's IP broadcast
    this->IP = _get_ip(this->PORT_UDP_BROADCAST);

    // succeeded to get robot's IP address
    if (this->IP != INADDR_NONE)
    {
        this->_tcp_socket = this->_connect(this->IP, this->PORT_TCP_CONTROL);

        // succeeded to create a tcp link with robot
        if (this->_tcp_socket > 0)
        {
            // enable robot's 'SDK Mode'
            std::string response = this->send_command(std::string("command"));

            // succeeded to enable robot's 'SDK Mode'
            if (0 == response.compare(std::string("ok")))
            {
                std::clog << "[Info] Success to enter robot's SDK Mode" << std::endl;
            }

            // failed to enable robot's 'SDK Mode'
            else
            {

            }
        }
        // failed to create tcp link with robot
        else
        {
            
        }
    }
    // failed to get robot's IP
    else
    {

    }
    
}

RobotController::~RobotController(void)
{
    // close the control socket
    if (this->_tcp_socket > 0) 
    {
        try
        {
            close(_tcp_socket);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }

    // release receive buffer
    if (receive_buffer)
    {
        delete receive_buffer;
    }
}


in_addr_t RobotController::_get_ip(int port)
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
        }
    }

    close(udp_socket);

    return robot_ip;
}

int RobotController::_connect(in_addr_t IP, int port)
{
    int tcp_socket = socket(PF_INET, SOCK_STREAM, 0);
    if (tcp_socket < 0)
    {
        std::cerr << "[Error] Failed to create a tcp socket to link up with the robot" << std::endl;
        return -1;
    }

    struct sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(sockaddr_in));

    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(port);
    target_addr.sin_addr.s_addr = IP;

    std::clog << "[Info] Connecting..." << std::endl;
    if (connect(tcp_socket, (struct sockaddr *)&target_addr, sizeof(sockaddr_in)) < 0)
    {
        std::cerr << "[Error] Connect error" << std::endl;
        return -1;
    }

    std::clog << "[Info] Connected to target" << std::endl;

    return tcp_socket;
}

std::string RobotController::send_command(std::string command)
{
    std::string response;

    send(this->_tcp_socket, command.c_str(), command.length(), 0);

    // response is received
    int l = recv(this->_tcp_socket, this->receive_buffer, this->BUFFER_LENGTH, 0);
    if (l > 0)
    {
        response = this->receive_buffer;
        response = response.substr(0, l);
        
        std::clog << "[Response] " << response << std::endl;
        return response;
    }
        
    return response;
}

bool RobotController::set_chassis_speed(float vx, float vy, float vz)
{
    std::string command = std::string("chassis speed") 
                        + " x " + std::to_string(vx) 
                        + " y " + std::to_string(vy)
                        + " z " + std::to_string(vz);
    std::clog << "[Command] " << command << std::endl;

    return (0 == std::string("ok").compare(this->send_command(command)));
}

#include "Controller.h"
#include "Robot.h"

using namespace RoboMasterEP;

Controller::Controller(Robot * robot, in_addr_t ip, int port)
    : robot(robot), ip(ip), port(port)
{
    // allocate receive buffer for some control command which within response
    this->receive_buffer = new char[BUFFER_LENGTH];
    memset(this->receive_buffer, 0, BUFFER_LENGTH);

    // succeeded to get robot's IP address
    if (this->ip != INADDR_NONE)
    {
        this->_tcp_socket = this->connect_via_tcp(this->ip, this->port);

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
                std::cerr << "[Error] Failed to enable robot's 'SDK Mode'" << std::endl;
            }
        }
        // failed to create tcp link with robot
        else
        {
            std::cerr << "[Error] Failed to create tcp link with robot" << std::endl;
        }
    }
    // failed to get robot's IP
    else
    {
        std::cerr << "[Error] Robot's IP error" << std::endl;
    }
}

Controller::~Controller(void)
{
    // exit robot's 'SDK Mode'
    std::string response = this->send_command(std::string("quit"));

    // succeeded to exit robot's 'SDK Mode'
    if (0 == response.compare(std::string("ok")))
    {
        std::clog << "[Info] Success to exit robot's SDK Mode" << std::endl;
    }

    // failed to exit robot's 'SDK Mode'
    else
    {
        std::cerr << "[Error] Failed to exit robot's 'SDK Mode'" << std::endl;
    }

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
        receive_buffer = nullptr;
    }

    robot = nullptr;
}

int Controller::connect_via_tcp(in_addr_t ip, int port)
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
    target_addr.sin_addr.s_addr = ip;

    std::clog << "[Info] Connecting..." << std::endl;
    if (connect(tcp_socket, (struct sockaddr *)&target_addr, sizeof(sockaddr_in)) < 0)
    {
        std::cerr << "[Error] Connect error" << std::endl;
        return -1;
    }

    std::clog << "[Info] Connected to target" << std::endl;

    return tcp_socket;
}

std::string Controller::send_command(std::string command)
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

bool Controller::set_chassis_speed(float vx, float vy, float vz)
{
    std::string command = std::string("chassis speed") 
                        + " x " + std::to_string(vx) 
                        + " y " + std::to_string(vy)
                        + " z " + std::to_string(vz);
    std::clog << "[Command] " << command << std::endl;

    return (0 == std::string("ok").compare(this->send_command(command)));
}

bool Controller::switch_chassis_push_info(PushSwitch s, ChassisPushAttr attr=ALL, ChassisPushFrequence freq=FREQ_1Hz)
{
    std::string command = std::string("chassis push");

    std::string _s = (s == ON) ? "on" : "off";

    switch (attr)
    {
    case POSITION:
        command += " position " + _s + " pfreq " + std::to_string(freq);
        break;

    case ATTITUDE:
        command += " attitude " + _s + " afreq " + std::to_string(freq);
        break;

    case STATUS:
        command += " status " + _s + " sfreq " + std::to_string(freq);
        break;
    
    default:
        command += " position " + _s + " attitude " + _s + " status " + _s + " freq " + std::to_string(freq);
        break;
    }

    std::clog << "[Command] " << command << std::endl;

    if (s == ON) this->robot->push_receiver->start();
    else         this->robot->push_receiver->stop();

    return (0 == std::string("ok").compare(this->send_command(command)));
}
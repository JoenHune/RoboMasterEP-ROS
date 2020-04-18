#include "EventHandler.h"

using namespace RoboMasterEP;

EventHandler::EventHandler(Robot *robot, in_addr_t ip, int port)
    : robot(robot), ip(ip), port(port)
{

}

EventHandler::~EventHandler()
{
    
    robot = nullptr;
}

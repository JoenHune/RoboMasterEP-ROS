#include <unistd.h>

#include "Robot.h"
#include "Thread.h"
#include "Types.h"

using namespace RoboMasterEP;
using std::thread;

Thread::Thread(Robot* robot)
    : robot(robot)
{
    running = new bool[NUMBER_OF_TYPES];
    delay   = new int[NUMBER_OF_TYPES];

    for (int i = 0; i < NUMBER_OF_TYPES; i++)
        running[i] = false;

    delay[PUSHRECEIVER]  = 10;     // 10us
    delay[EVENTDETECTOR] = 10000;  // 10ms

    callbacks.clear();
    callbacks.push_back(std::bind(&Thread::push_receiver_callback, this, robot));
    callbacks.push_back(std::bind(&Thread::event_detector_callback, this, robot));
}

Thread::~Thread()
{
    for (int i = 0; i < NUMBER_OF_TYPES; i++)
        stop(i);

    if (running)
    {
        delete running;
        running = NULL;
    }
    
    if (delay)
    {
        delete delay;
        delay = NULL;
    }
    
    robot = NULL;
}

bool Thread::start(int type)
{
    if (!running[type])
    {
        try
        {
            thread th(callbacks[type]);
            th.detach();

            running[type] = true;
        }
        catch (...)
        {
            return false;
        }

        return true;
    }
    else
    {
        return true;
    }
}

bool Thread::stop(int type)
{
    running[type] = false;

    // wait for stop
    usleep(2.0 * delay[type]);
    
    return true;
}

// unsafe
bool Thread::restart(int type)
{
    if (running[type])
        stop(type);

    start(type);

    return true;
}

void Thread::push_receiver_callback(Robot* robot)
{
    while(running[PUSHRECEIVER])
    {
        this->robot->push_receiver->receive();
        usleep(delay[PUSHRECEIVER]);
    }
}

void Thread::event_detector_callback(Robot* robot)
{
    while(running[EVENTDETECTOR])
    {
        usleep(delay[EVENTDETECTOR]); // 1000ms
    }
}

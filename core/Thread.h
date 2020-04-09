#pragma once

#include <stdlib.h>
#include <thread>
#include <vector>
#include <functional>

namespace RoboMasterEP
{

class Robot;

class Thread
{
public:
    Thread(Robot* robot);

    ~Thread();

    bool start(int type);

    bool stop(int type);

    bool restart(int type);

private:

    void push_receiver_callback(Robot* robot);

    void event_detector_callback(Robot* robot);

private:
    Robot* robot;

    bool* running; // flag to discribe whether RoboMasterEP_brain is terminal(to stop threads)

    int* delay;

    std::vector<std::__bind<void (RoboMasterEP::Thread::*)(RoboMasterEP::Robot *), RoboMasterEP::Thread *, RoboMasterEP::Robot *&>> callbacks;

}; // class thread

}; // namespace

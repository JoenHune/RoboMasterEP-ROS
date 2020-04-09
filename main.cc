#include "core/Robot.h"

int main()
{
    RoboMasterEP::Robot robot;

    robot.controller->switch_chassis_push_info(RoboMasterEP::ON, RoboMasterEP::ALL, RoboMasterEP::FREQ_10Hz);

    while (1)
    {
        /* code */
    }

    return 0;
}
#include "core/Robot.h"

int main()
{
    Robot robot;

    robot.controller->set_chassis_speed(0, 0, 0);

    return 0;
}
#include "core/RobotController.hpp"

int main()
{
    RobotController robot;

    // robot.send_command("robotic_gripper close 1");

    robot.set_chassis_speed(1, 0, 0);

    return 0;
}
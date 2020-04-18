#include "modules/Types.h"
#include "RobotNode.h"

using namespace RoboMasterEP;

void RobotNode::joystick_control_callback(const sensor_msgs::JoyConstPtr &msg)
{
    float vx, vy, vz;

    if (fabs(vz) < 1e-2) vz = 0;
    if (fabs(vx) < 1e-2) vx = 0;
    if (fabs(vy) < 1e-2) vy = 0;

    vz = msg->axes[0] * -600;   // 左摇杆横
    vx = msg->axes[4] * 3.5;    // 右摇杆竖
    vy = msg->axes[3] * -3.5;   // 右摇杆横

    if (this->robot)
    {
        robot->controller->set_chassis_speed(vx, vy, vz);
    }
}

void RobotNode::expected_state_control_callback(const robomaster_ep::PositionCommandConstPtr &msg)
{
    float dx, dy, dz, vxy, vz;

    if (this->robot)
    {
        robot->controller->set_chassis_position_relative(dx, dy, dz, vxy, vz);
    }
}

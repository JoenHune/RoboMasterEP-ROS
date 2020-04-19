#include "modules/Types.h"
#include "RobotNode.h"
#include <cmath>

using namespace RoboMasterEP;

float shift_eliminate(const float &origin, float deadzone = 0.1)
{
    return (fabs(origin) < deadzone ? 0 : origin);
}

void RobotNode::joystick_control_callback(const sensor_msgs::JoyConstPtr &msg)
{
    float vx, vy, vz;

    vx =  3.5f * shift_eliminate(msg->axes[this->JS_STICK_X], this->JS_STICK_DEADZONE); // 右摇杆竖
    vy = -3.5f * shift_eliminate(msg->axes[this->JS_STICK_Y], this->JS_STICK_DEADZONE); // 右摇杆横
    vz = -600  * shift_eliminate(msg->axes[this->JS_STICK_Z], this->JS_STICK_DEADZONE); // 左摇杆横

    if (msg->buttons[this->JS_BUTTON_MANUAL])
    {
        this->control_mode = ControlMode::MANUAL;
        this->robot->controller->set_led_effect(LEDComp::BOTTOM_ALL, LEDEffect::SOLID, this->LED_MANUAL);
    }

    if (msg->buttons[this->JS_BUTTON_AUTONOMOUS])
    {
        this->control_mode = ControlMode::AUTONOMOUS;
        this->robot->controller->set_led_effect(LEDComp::BOTTOM_ALL, LEDEffect::SOLID, this->LED_AUTONOMOUS);
    }

    if (this->robot && this->control_mode == ControlMode::MANUAL)
    {
        this->robot->controller->set_chassis_speed(vx, vy, vz);
    }
}

void RobotNode::expected_state_control_callback(const robomaster_ep::PositionCommandConstPtr &msg)
{
    float dx, dy, dz, vxy, vz;

    if (this->robot && this->control_mode == ControlMode::AUTONOMOUS)
    {
        this->robot->controller->set_chassis_position_relative(dx, dy, dz, vxy, vz);
    }
}

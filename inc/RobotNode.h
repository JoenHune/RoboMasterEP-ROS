#include <ros/ros.h>
#include <arpa/inet.h>

#include "Robot.h"

#include "robomaster_ep/PositionCommand.h"
#include <sensor_msgs/Joy.h>

namespace RoboMasterEP
{

class RobotNode
{
public:
    RobotNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    ~RobotNode();

private:
    bool init_robot();
    bool init_subscribers(ros::NodeHandle &nh);
    bool init_publishers(ros::NodeHandle &nh);

    // subscribe callbacks
    void joystick_control_callback(const sensor_msgs::JoyConstPtr &msg);
    void expected_state_control_callback(const robomaster_ep::PositionCommandConstPtr &msg);

public:
    // publish callbacks


private:
    // core
    Robot *robot;

    // subscribers
    ros::Subscriber joystick_subscriber;
    ros::Subscriber expected_state_subscriber;

    // publishers

    // connection data
    // serial
    std::string serial_portname;
    int         baud_rate;
    // lan
    in_addr_t   lan_ip;

    // enums

}; // end of class RobotNode

} // end of namespace RoboMasterEP
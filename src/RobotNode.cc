#include <RobotNode.h>

using namespace RoboMasterEP;

RobotNode::RobotNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
{
    this->robot = nullptr;

	if(!this->init_subscribers(nh))
    {
	    ROS_ERROR("Initialize subscribers failed.");
        throw std::runtime_error("Initialize subscribers failed.");
    }

	if(!this->init_publishers(nh))
    {
	    ROS_ERROR("Initialize publishers failed.");
        throw std::runtime_error("Initialize publishers failed.");
    }

	if (!this->init_robot())
	{
	    ROS_ERROR("Initialize robot failed.");
        throw std::runtime_error("Initialize robot failed.");
	}
	else
	{
        ROS_INFO("Successful to access robot.");
	}
}

RobotNode::~RobotNode()
{
    if (this->robot)
    {
        delete this->robot;
        this->robot = nullptr;
    }
}

bool RobotNode::init_robot()
{
    try
    {
        this->robot = new Robot(this);
    }
    catch (...)
    {
        ROS_ERROR_STREAM("Ooops, an unknown error occurs.\n\n"
                      << "Connect the author to look for more help please. :-)\n");
        return false;
    }

    return true;
}

bool RobotNode::init_subscribers(ros::NodeHandle &nh)
{
    try
    {
        this->joystick_subscriber = nh.subscribe<sensor_msgs::Joy>
        (
            "joy", 
            10, 
            boost::bind(&RobotNode::joystick_control_callback, this, _1),
            ros::VoidConstPtr(),
            ros::TransportHints().tcpNoDelay()
        );
        this->expected_state_subscriber = nh.subscribe<robomaster_ep::PositionCommand>
        (
            "exp",
            10,
            boost::bind(&RobotNode::expected_state_control_callback, this, _1),
            ros::VoidConstPtr(),
            ros::TransportHints().tcpNoDelay()
        );
    }
    catch (...)
    {
        return false;
    }

    return true;
}

bool RobotNode::init_publishers(ros::NodeHandle &nh)
{
    return true;
}
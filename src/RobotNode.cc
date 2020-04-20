#include <RobotNode.h>

using namespace RoboMasterEP;

RobotNode::RobotNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
{
    this->robot = nullptr;

    if (!init_parameters(nh_private))
    {
	    ROS_ERROR("Initialize parameters failed.");
        throw std::runtime_error("Initialize parameters failed.");
    }

	if (!this->init_subscribers(nh))
    {
	    ROS_ERROR("Initialize subscribers failed.");
        throw std::runtime_error("Initialize subscribers failed.");
    }

	if (!this->init_publishers(nh))
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
        // this->robot->controller->set_led_effect(LEDComp::BOTTOM_ALL, LEDEffect::SOLID, this->LED_CONNECTED);
	}
}

RobotNode::~RobotNode()
{
    // this->robot->controller->set_led_effect(LEDComp::BOTTOM_ALL, LEDEffect::BLINK, this->LED_DISCONNECTED);

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
        this->control_mode = ControlMode::MANUAL;
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

bool RobotNode::init_parameters(ros::NodeHandle &nh)
{
    nh.param("JS_STICK_X",          this->JS_STICK_X,               4);
    nh.param("JS_STICK_Y",          this->JS_STICK_Y,               3);
    nh.param("JS_STICK_Z",          this->JS_STICK_Z,               0);
    nh.param("JS_STICK_DEADZONE",   this->JS_STICK_DEADZONE,        0.1f);
    nh.param("JS_BUTTON_MANUAL",    this->JS_BUTTON_MANUAL,         6);
    nh.param("JS_BUTTON_AUTONOMOUS", this->JS_BUTTON_AUTONOMOUS,    7);
    nh.param("LED_CONNECTED",       this->LED_CONNECTED,            0xC2FFDE);
    nh.param("LED_DISCONNECTED",    this->LED_DISCONNECTED,         0xFFA59C);
    nh.param("LED_MANUAL",          this->LED_MANUAL,               0x8FC1FF);
    nh.param("LED_AUTONOMOUS",      this->LED_AUTONOMOUS,           0xFFCAA8);
    return true;
}
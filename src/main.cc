
#include "RobotNode.h"

using RoboMasterEP::RobotNode;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot");

    ros::NodeHandle nh;              // public topics manager
    ros::NodeHandle nh_private("~"); // parameters manager

    RobotNode* robot_node = new RobotNode(nh, nh_private);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    delete robot_node;
    robot_node = NULL;

    return 0;
}
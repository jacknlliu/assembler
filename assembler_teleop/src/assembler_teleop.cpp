#include "ros/ros.h"
#include <ros/console.h>
#include "assembler_teleop/TeleopVirtualRobot.h"

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "assembler_teleop_sim");

	ros::NodeHandle n;

	TeleopVirtualRobot teleop(n, RobotOperateMode::TELEOP_MODE);
	teleop.start();

	ros::shutdown();

	return 0;
}
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "stdio.h"
#include <stdlib.h>
#include <iostream>
#include <sstream>

int timeout = 1; //seconds

ros::Time start;
ros::Time end;

ros::Publisher command_pub;

void checkNetCallback(const std_msgs::Bool::ConstPtr& msg)
{
	std_msgs::String cmd;
	cmd.data = "resume";
	command_pub.publish(cmd);

	start = ros::Time::now();
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "ping_node");
	ros::NodeHandle n;

	ros::Subscriber checkNet_sub = n.subscribe("checkNet", 10, checkNetCallback);
	command_pub = n.advertise<std_msgs::String>("ping_command", 2);
	
	start = ros::Time::now();
	end = ros::Time::now();

	ros::Rate r(1);

	while (ros::ok())
	{
		end = ros::Time::now();
		// ROS_INFO("start: %f\tend: %f\tdiff: %f", start.toSec(), end.toSec(), end.toSec()-start.toSec());
		if ((end.toSec() - start.toSec()) > timeout)
		{
			std_msgs::String cmd;
			cmd.data = "pause";
			command_pub.publish(cmd);
		}

		r.sleep();
		ros::spinOnce();
	}

	return 0;
}
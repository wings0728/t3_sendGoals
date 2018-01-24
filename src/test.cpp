#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "stdio.h"
#include <stdlib.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::Bool>("checkNet", 10);
	ros::Rate r(1);

	while (n.ok())
	{
		std_msgs::Bool ret;
		ret.data = true;
		pub.publish(ret);
		r.sleep();
	}

	return 0;
}
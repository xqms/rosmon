// rosmon - versatile ROS node launcher & monitor
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/init.h>
#include <ros/master.h>
#include <ros/rate.h>
#include <ros/node_handle.h>

#include "launch_config.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rosmon");

	printf("ROS_MASTER_URI: '%s'\n", ros::master::getURI().c_str());

	if(ros::master::check())
	{
		printf("roscore is already running.\n");
	}
	else
	{
		printf("Starting own roscore...\n");
		fprintf(stderr, "Scratch that, I can't do that yet. Exiting...\n");
		return 1;
	}

	if(argc != 2)
	{
		fprintf(stderr, "Usage: rosmon <launchfile>\n");
		return 1;
	}

	rosmon::LaunchConfig config;
	config.parse(argv[1]);
	config.setParameters();

	config.start();

	ros::NodeHandle nh;

	while(ros::ok())
	{
		ros::spinOnce();
		config.communicate();
	}

	printf("Shutting down...\n");
	config.shutdown();

	ros::WallRate rate(10.0);
	while(1)
	{
		config.communicate();

		if(config.allShutdown())
			break;

		rate.sleep();
	}

	return 0;
}

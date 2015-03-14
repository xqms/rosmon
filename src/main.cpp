// rosmon - versatile ROS node launcher & monitor
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/init.h>
#include <ros/master.h>
#include <ros/rate.h>
#include <ros/node_handle.h>

#include "launch_config.h"
#include "ui.h"
#include "ros_interface.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rosmon", ros::init_options::AnonymousName);
	setlocale(LC_ALL, "");

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

	// Setup a sane ROSCONSOLE_FORMAT if the user isn't able to
	setenv("ROSCONSOLE_FORMAT", "[${function}]: ${message}", 0);

	rosmon::LaunchConfig config;
	config.parse(argv[1]);
	config.setParameters();

	config.start();

	// Start the ncurses UI
	rosmon::UI ui(&config);

	// ROS interface
	rosmon::ROSInterface rosInterface(&config);

	ros::NodeHandle nh;

	while(ros::ok())
	{
		ros::spinOnce();
		config.communicate();
		ui.update();
	}

	printf("Shutting down...\n");
	config.shutdown();

	ros::WallRate rate(10.0);
	ros::WallTime start = ros::WallTime::now();
	while(ros::WallTime::now() - start < ros::WallDuration(5.0))
	{
		config.communicate();
		ui.update();

		if(config.allShutdown())
			return 0;
	}

	config.forceExit();

	while(1)
	{
		config.communicate();
		ui.update();

		if(config.allShutdown())
			return 0;
	}

	return 0;
}

// rosmon - versatile ROS node launcher & monitor
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/init.h>
#include <ros/master.h>
#include <ros/rate.h>
#include <ros/node_handle.h>
#include <ros/package.h>

#include "launch_config.h"
#include "ui.h"
#include "ros_interface.h"
#include "package_registry.h"

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

fs::path findFile(const fs::path& base, const std::string name)
{
	for(fs::directory_iterator it(base); it != fs::directory_iterator(); ++it)
	{
		if(fs::is_directory(*it))
		{
			fs::path p = findFile(*it, name);
			if(!p.empty())
				return p;
		}
		else if(it->path().leaf() == name)
			return *it;
	}

	return fs::path();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rosmon", ros::init_options::AnonymousName);
	setlocale(LC_ALL, "");

	std::string launchFileName;
	if(argc == 2)
		launchFileName = argv[1];
	else if(argc == 3)
	{
		std::string package = rosmon::PackageRegistry::getPath(argv[1]);
		if(package.empty())
		{
			fprintf(stderr, "Could not find path of package '%s'\n", package.c_str());
			return 1;
		}

		fs::path path = findFile(package, argv[2]);
		if(path.empty())
		{
			fprintf(stderr, "Could not find launch file '%s' in package '%s'\n", argv[2], argv[1]);
			return 1;
		}

		launchFileName = path.string();
	}
	else
	{
		fprintf(stderr, "Usage: rosmon <launchfile>\n");
		return 1;
	}

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

	// Setup a sane ROSCONSOLE_FORMAT if the user did not already
	setenv("ROSCONSOLE_FORMAT", "[${function}]: ${message}", 0);

	rosmon::LaunchConfig config;
	config.parse(launchFileName);
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

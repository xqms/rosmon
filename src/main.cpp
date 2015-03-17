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
#include "fd_watcher.h"

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
	// FIXME: We can't pass the whole argv to ros::init() because it interprets
	// the launch arguments (arg1:=v1) as remappings...
	int dummyArgc = 1;
	ros::init(dummyArgc, argv, "rosmon", ros::init_options::AnonymousName);

	rosmon::PackageRegistry::init();

	if(argc == 1 || (argc >= 2 && strcmp(argv[1], "--help") == 0))
	{
		fprintf(stderr, "Usage: rosmon (<launchfile> or <package> <launchfile>) arg1:=v1 arg2:=v2...\n");
		return 1;
	}

	int firstArg = 1;
	for(; firstArg < argc; ++firstArg)
	{
		if(strstr(argv[firstArg], ":="))
			break;
	}

	std::string launchFileName;
	if(firstArg == 2)
		launchFileName = argv[1];
	else if(firstArg == 3)
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
		fprintf(stderr, "Usage: rosmon (<launchfile> or <package> <launchfile>) arg1:=v1 arg2:=v2...\n");
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

	ros::NodeHandle nh;

	// Setup a sane ROSCONSOLE_FORMAT if the user did not already
	setenv("ROSCONSOLE_FORMAT", "[${function}]: ${message}", 0);

	rosmon::FDWatcher::Ptr watcher(new rosmon::FDWatcher);

	rosmon::LaunchConfig config(watcher);

	for(int i = firstArg; i < argc; ++i)
	{
		char* arg = strstr(argv[i], ":=");

		if(!arg)
		{
			fprintf(stderr, "You specified a non-argument after an argument\n");
			return 1;
		}

		char* name = argv[i];

		*arg = 0;

		char* value = arg + 2;

		config.setArgument(name, value);
	}

	config.parse(launchFileName);
	config.setParameters();

	config.start();

	// Start the ncurses UI
	rosmon::UI ui(&config);

	// ROS interface
	rosmon::ROSInterface rosInterface(&config);

	ros::WallDuration waitDuration(0.1);

	while(ros::ok())
	{
		ros::spinOnce();
		watcher->wait(waitDuration);
		ui.update();
	}

	ui.log("[rosmon]", "Shutting down...");
	config.shutdown();

	ros::WallTime start = ros::WallTime::now();
	while(ros::WallTime::now() - start < ros::WallDuration(5.0))
	{
		watcher->wait(waitDuration);
		ui.update();

		if(config.allShutdown())
			return 0;
	}

	config.forceExit();

	while(1)
	{
		watcher->wait(waitDuration);
		ui.update();

		if(config.allShutdown())
			return 0;
	}

	return 0;
}

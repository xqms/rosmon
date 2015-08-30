// rosmon - versatile ROS node launcher & monitor
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/init.h>
#include <ros/master.h>
#include <ros/rate.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <ros/console_backend.h>

#include "launch_config.h"
#include "ui.h"
#include "ros_interface.h"
#include "package_registry.h"
#include "fd_watcher.h"
#include "logger.h"

#include <boost/filesystem.hpp>

#include <unistd.h>
#include <getopt.h>
#include <signal.h>

namespace fs = boost::filesystem;

static ros::Publisher g_pub_rosout;

bool g_shouldStop = false;

static fs::path findFile(const fs::path& base, const std::string name)
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

void usage()
{
	fprintf(stderr,
		"Usage:\n"
		"  rosmon [options] some_package test.launch [arg1:=value1 ...]\n"
		"  rosmon [options] path/to/test.launch [arg1:=value1 ...]\n"
		"\n"
		"Options:\n"
		"  --benchmark    Exit after loading the launch file\n"
		"  --help         This help screen\n"
		"  --log=FILE     Write log file to FILE\n"
		"  --name=NAME    Use NAME as ROS node name. By default, an anonymous\n"
		"                 name is chosen.\n"
		"\n"
	);
}

void handleSIGINT(int)
{
	g_shouldStop = true;
}

// Options
static const struct option OPTIONS[] = {
	{"help", no_argument, NULL, 'h'},
	{"name", required_argument, NULL, 'n'},
	{"log",  required_argument, NULL, 'l'},
	{"benchmark", no_argument, NULL, 'b'},
	{NULL, 0, NULL, 0}
};

int main(int argc, char** argv)
{
	std::string name;
	std::string logFile;
	std::string launchFilePath;
	bool benchmark = false;

	// Parse options
	while(1)
	{
		int option_index;
		int c = getopt_long(argc, argv, "h", OPTIONS, &option_index);

		if(c == -1)
			break;

		switch(c)
		{
			case 'h':
				usage();
				return 0;
			case 'n':
				name = optarg;
				break;
			case 'l':
				logFile = optarg;
				break;
			case 'b':
				benchmark = true;
				break;
		}
	}

	// Initialize the ROS node.
	uint32_t init_options = ros::init_options::NoSigintHandler;

	if(name.empty())
	{
		name = "rosmon";
		init_options |= ros::init_options::AnonymousName;
	}

	// prevent ros::init from parsing the launch file arguments as remappings
	int dummyArgc = 1;
	ros::init(dummyArgc, argv, name, init_options);

	// Parse the positional arguments
	if(optind == argc)
	{
		usage();
		return 1;
	}

	// Find first argument (must contain ':=')
	int firstArg = optind + 1;
	for(; firstArg < argc; ++firstArg)
	{
		if(strstr(argv[firstArg], ":="))
			break;
	}

	// From the position of the argument (or the end-of-options), we know
	// if we were called with a) package + filename or b) just a path.

	rosmon::PackageRegistry::init();

	if(firstArg - optind == 1)
	{
		launchFilePath = argv[optind];
	}
	else if(firstArg - optind == 2)
	{
		const char* packageName = argv[optind];
		const char* fileName = argv[optind + 1];

		std::string package = rosmon::PackageRegistry::getPath(packageName);
		if(package.empty())
		{
			fprintf(stderr, "Could not find path of package '%s'\n", packageName);
			return 1;
		}

		fs::path path = findFile(package, fileName);
		if(path.empty())
		{
			fprintf(stderr, "Could not find launch file '%s' in package '%s'\n",
				fileName, packageName
			);
			return 1;
		}

		launchFilePath = path.string();
	}
	else
	{
		usage();
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

	// Disable direct logging to stdout
	ros::console::backend::function_print = nullptr;

	// Open logger
	boost::scoped_ptr<rosmon::Logger> logger;
	if(!logFile.empty())
	{
		logger.reset(new rosmon::Logger(logFile));
		config.logMessageSignal.connect(boost::bind(&rosmon::Logger::log, logger.get(), _1, _2));
	}

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

	config.parse(launchFilePath);
	config.setParameters();

	if(benchmark)
		return 0;

	config.start();

	// Start the ncurses UI
	rosmon::UI ui(&config, watcher);

	// ROS interface
	rosmon::ROSInterface rosInterface(&config);

	ros::WallDuration waitDuration(0.1);

	signal(SIGINT, handleSIGINT);

	while(ros::ok() && config.ok() && !g_shouldStop)
	{
		ros::spinOnce();
		watcher->wait(waitDuration);
		ui.update();
	}

	ui.log("[rosmon]", "Shutting down...");
	config.shutdown();

	ros::WallTime start = ros::WallTime::now();
	while(!config.allShutdown() && ros::WallTime::now() - start < ros::WallDuration(5.0))
	{
		watcher->wait(waitDuration);
		ui.update();
	}

	if(!config.allShutdown())
		config.forceExit();

	rosInterface.shutdown();

	while(!config.allShutdown())
	{
		watcher->wait(waitDuration);
		ui.update();
	}

	bool coredumpsAvailable = false;
	for(auto& node : config.nodes())
	{
		if(node->coredumpAvailable())
		{
			coredumpsAvailable = true;
			break;
		}
	}

	if(coredumpsAvailable)
	{
		ui.log("[rosmon]", "\n");
		ui.log("[rosmon]", "If you want to debug one of the crashed nodes, you can use the following commands");
		for(auto& node : config.nodes())
		{
			if(node->coredumpAvailable())
				ui.log("[rosmon]", " # " + node->debuggerCommand());
		}
	}

	return 0;
}

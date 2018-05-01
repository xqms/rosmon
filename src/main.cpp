// rosmon - versatile ROS node launcher & monitor
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/init.h>
#include <ros/master.h>
#include <ros/rate.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <ros/console_backend.h>
#include <ros/this_node.h>

#include "launch/launch_config.h"
#include "monitor/monitor.h"
#include "ui.h"
#include "ros_interface.h"
#include "package_registry.h"
#include "fd_watcher.h"
#include "logger.h"

#include <boost/filesystem.hpp>

#include <unistd.h>
#include <getopt.h>
#include <csignal>

#include <iostream>

namespace fs = boost::filesystem;

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
		"  rosmon [actions] [options] some_package test.launch [arg1:=value1 ...]\n"
		"  rosmon [actions] [options] path/to/test.launch [arg1:=value1 ...]\n"
		"\n"
		"Actions (default is to launch the launch file):\n"
		"  --benchmark    Exit after loading the launch file\n"
		"  --list-args    List launch file arguments\n"
		"\n"
		"Options:\n"
		"  --disable-ui   Disable fancy terminal UI\n"
		"  --help         This help screen\n"
		"  --log=FILE     Write log file to FILE\n"
		"  --name=NAME    Use NAME as ROS node name. By default, an anonymous\n"
		"                 name is chosen.\n"
		"\n"
		"rosmon also obeys some environment variables:\n"
		"  ROSMON_COLOR_MODE   Can be set to 'truecolor', '256colors', 'ansi'\n"
		"                      to force a specific color mode\n"
		"                      If unset, rosmon tries to detect the best\n"
		"                      available color mode.\n"
	);
}

void handleSIGINT(int)
{
	g_shouldStop = true;
}

void logToStdout(const std::string& channel, const std::string& str)
{
	std::string clean = str;
	unsigned int len = clean.length();
	while(len != 0 && (clean[len-1] == '\n' || clean[len-1] == '\r'))
		len--;

	clean.resize(len);

	printf("%20s: %s\n", channel.c_str(), clean.c_str());
}

// Options
static const struct option OPTIONS[] = {
	{"disable-ui", no_argument, nullptr, 'd'},
	{"benchmark", no_argument, nullptr, 'b'},
	{"help", no_argument, nullptr, 'h'},
	{"list-args", no_argument, nullptr, 'L'},
	{"log",  required_argument, nullptr, 'l'},
	{"name", required_argument, nullptr, 'n'},
	{nullptr, 0, nullptr, 0}
};

enum Action {
	ACTION_LAUNCH,
	ACTION_BENCHMARK,
	ACTION_LIST_ARGS,
};

int main(int argc, char** argv)
{
	std::string name;
	std::string logFile;
	std::string launchFilePath;

	Action action = ACTION_LAUNCH;
	bool enableUI = true;

	// Parse options
	while(true)
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
			case 'L':
				action = ACTION_LIST_ARGS;
				break;
			case 'b':
				action = ACTION_BENCHMARK;
				break;
			case 'd':
				enableUI = false;
				break;
		}
	}

	// Parse the positional arguments
	if(optind == argc)
	{
		usage();
		return 1;
	}

	// Find first launch file argument (must contain ':=')
	int firstArg = optind + 1;
	for(; firstArg < argc; ++firstArg)
	{
		if(strstr(argv[firstArg], ":="))
			break;
	}

	// From the position of the argument (or the end-of-options), we know
	// if we were called with a) package + filename or b) just a path.

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

	// Setup logging
	boost::scoped_ptr<rosmon::Logger> logger;
	{
		// Setup a sane ROSCONSOLE_FORMAT if the user did not already
		setenv("ROSCONSOLE_FORMAT", "[${function}]: ${message}", 0);

		// Disable direct logging to stdout
		ros::console::backend::function_print = nullptr;

		// Open logger
		if(logFile.empty())
		{
			// Log to /tmp by default

			time_t t = time(nullptr);
			tm currentTime;
			localtime_r(&t, &currentTime);

			char buf[256];
			strftime(buf, sizeof(buf), "/tmp/rosmon_%Y_%m_%d_%H_%M_%S.log", &currentTime);

			logFile = buf;
		}

		logger.reset(new rosmon::Logger(logFile));
	}

	rosmon::FDWatcher::Ptr watcher(new rosmon::FDWatcher);

	rosmon::launch::LaunchConfig::Ptr config(new rosmon::launch::LaunchConfig);

	// Parse launch file arguments from command line
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

		config->setArgument(name, value);
	}

	bool onlyArguments = (action == ACTION_LIST_ARGS);

	try
	{
		config->parse(launchFilePath, onlyArguments);
		config->evaluateParameters();
	}
	catch(rosmon::launch::LaunchConfig::ParseException& e)
	{
		fprintf(stderr, "Could not load launch file: %s\n", e.what());
		return 1;
	}

	switch(action)
	{
		case ACTION_BENCHMARK:
			return 0;
		case ACTION_LIST_ARGS:
			for(const auto& arg : config->arguments())
				std::cout << arg.first << std::endl;

			return 0;
		case ACTION_LAUNCH:
			break;
	}

	// Initialize the ROS node.
	{
		uint32_t init_options = ros::init_options::NoSigintHandler;

		// If the user did not specify a node name on the command line, look in
		// the launch file
		if(name.empty())
			name = config->rosmonNodeName();

		// As last fallback, choose anonymous name.
		if(name.empty())
		{
			name = "rosmon";
			init_options |= ros::init_options::AnonymousName;
		}

		// prevent ros::init from parsing the launch file arguments as remappings
		int dummyArgc = 1;
		ros::init(dummyArgc, argv, name, init_options);
	}

	// Check connectivity to ROS master
	{
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
	}

	ros::NodeHandle nh;

	printf("Running as '%s'\n", ros::this_node::getName().c_str());

	rosmon::monitor::Monitor monitor(config, watcher);
	monitor.logMessageSignal.connect(boost::bind(&rosmon::Logger::log, logger.get(), _1, _2));

	printf("\n\n");
	monitor.setParameters();

	if(config->nodes().empty())
	{
		printf("No ROS nodes to be launched. Finished...\n");
		return 0;
	}

	monitor.start();

	// Start the ncurses UI
	boost::scoped_ptr<rosmon::UI> ui;
	if(enableUI)
	{
		ui.reset(new rosmon::UI(&monitor, watcher));
	}
	else
	{
		monitor.logMessageSignal.connect(logToStdout);
	}

	// ROS interface
	rosmon::ROSInterface rosInterface(&monitor);

	ros::WallDuration waitDuration(0.1);

	signal(SIGINT, handleSIGINT);

	// Main loop
	while(ros::ok() && monitor.ok() && !g_shouldStop)
	{
		ros::spinOnce();
		watcher->wait(waitDuration);

		if(ui)
			ui->update();
	}

	if(ui)
		ui->log("[rosmon]", "Shutting down...");
	monitor.shutdown();

	// Wait for graceful shutdown
	ros::WallTime start = ros::WallTime::now();
	while(!monitor.allShutdown() && ros::WallTime::now() - start < ros::WallDuration(5.0))
	{
		watcher->wait(waitDuration);

		if(ui)
			ui->update();
	}

	// If we timed out, force exit (kill the nodes with SIGKILL)
	if(!monitor.allShutdown())
		monitor.forceExit();

	rosInterface.shutdown();

	// Wait until that is finished (should always work)
	while(!monitor.allShutdown())
	{
		watcher->wait(waitDuration);

		if(enableUI)
			ui->update();
	}

	// If coredumps are available, be helpful and display gdb commands
	bool coredumpsAvailable = std::any_of(monitor.nodes().begin(), monitor.nodes().end(),
		[](const rosmon::monitor::NodeMonitor::Ptr& n) { return n->coredumpAvailable(); }
	);

	if(ui && coredumpsAvailable)
	{
		ui->log("[rosmon]", "\n");
		ui->log("[rosmon]", "If you want to debug one of the crashed nodes, you can use the following commands");
		for(auto& node : monitor.nodes())
		{
			if(node->coredumpAvailable())
			{
				std::stringstream ss;
				ss << std::setw(20) << node->name() << ": # " + node->debuggerCommand();
				ui->log("[rosmon]", ss.str());
			}
		}
	}

	return 0;
}

// Monitors execution of a launch file
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "monitor.h"

#include <ros/package.h>
#include <ros/node_handle.h>

#include <fstream>

#include <stdarg.h>
#include <stdio.h>

#include <boost/regex.hpp>
#include <boost/algorithm/string/trim.hpp>

#include <yaml-cpp/yaml.h>

namespace rosmon
{
namespace monitor
{

Monitor::Monitor(const launch::LaunchConfig::ConstPtr& config, const FDWatcher::Ptr& watcher)
 : m_config(config)
 , m_fdWatcher(watcher)
 , m_ok(true)
{
	for(auto& launchNode : config->nodes())
	{
		auto node = std::make_shared<NodeMonitor>(launchNode, m_fdWatcher, m_nh);

		node->logMessageSignal.connect(logMessageSignal);

		if(launchNode->required())
		{
			node->exitedSignal.connect(
				boost::bind(&Monitor::handleRequiredNodeExit, this, _1)
			);
		}

		m_nodes.push_back(node);
	}
}

Monitor::~Monitor()
{
}

void Monitor::setParameters()
{
	for(auto& param : m_config->parameters())
		m_nh.setParam(param.first, param.second);
}

void Monitor::start()
{
	for(auto& node : m_nodes)
	{
		node->start();
	}
}

void Monitor::shutdown()
{
	for(auto& node : m_nodes)
		node->shutdown();
}

void Monitor::forceExit()
{
	log("Killing the following nodes, which are refusing to exit:\n");
	for(auto& node : m_nodes)
	{
		if(node->running())
		{
			log(" - %s\n", node->name().c_str());
			node->forceExit();
		}
	}
}

bool Monitor::allShutdown()
{
	bool allShutdown = true;
	for(auto& node : m_nodes)
	{
		if(node->running())
			allShutdown = false;
	}

	return allShutdown;
}

void Monitor::handleRequiredNodeExit(const std::string& name)
{
	log("Required node '%s' exited, shutting down...", name.c_str());
	m_ok = false;
}

void Monitor::log(const char* fmt, ...)
{
	static char buf[512];

	va_list v;
	va_start(v, fmt);

	vsnprintf(buf, sizeof(buf), fmt, v);

	va_end(v);

	logMessageSignal("[rosmon]", buf);
}

}
}

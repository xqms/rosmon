// Monitors execution of a launch file
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "monitor.h"

#include <ros/package.h>
#include <ros/node_handle.h>
#include <ros/param.h>

#include <cstdarg>
#include <cstdio>
#include <fstream>

#include <boost/regex.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/filesystem.hpp>

#include <yaml-cpp/yaml.h>

#include <fmt/format.h>

#include "linux_process_info.h"

template<typename... Args>
std::runtime_error error(const char* fmt, const Args& ... args)
{
	return std::runtime_error(fmt::format(fmt, args...));
}

namespace rosmon
{
namespace monitor
{

Monitor::Monitor(launch::LaunchConfig::ConstPtr config, FDWatcher::Ptr watcher)
 : m_config(std::move(config))
 , m_fdWatcher(std::move(watcher))
 , m_ok(true)
{
	for(auto& launchNode : m_config->nodes())
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

#if HAVE_STEADYTIMER
	m_statTimer = m_nh.createSteadyTimer(
#else
	m_statTimer = m_nh.createWallTimer(
#endif
		ros::WallDuration(1.0),
		boost::bind(&Monitor::updateStats, this)
	);
}

void Monitor::setParameters()
{
	{
		std::vector<std::string> paramNames;

		for(auto& node : m_config->nodes())
		{
			if(node->clearParams())
			{
				std::string paramNamespace = node->namespaceString() + "/" + node->name() + "/";

				log("Deleting parameters in namespace {}", paramNamespace.c_str());

				if(paramNames.empty())
				{
					if(!ros::param::getParamNames(paramNames))
						throw error("Could not get list of parameters for clear_param");
				}

				for(auto& param : paramNames)
				{
					if(param.substr(0, paramNamespace.length()) == paramNamespace)
					{
						ros::param::del(param);
					}
				}
			}
		}
	}

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
			log(" - {}\n", node->name());
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

double Monitor::shutdownTimeout()
{
	double timeout = 0.0;
	// Find the biggest timeout
	for(auto& node : m_nodes)
		if(node->stopTimeout() > timeout)
			timeout = node->stopTimeout();
	return timeout;
}


void Monitor::handleRequiredNodeExit(const std::string& name)
{
	log("Required node '{}' exited, shutting down...", name);
	m_ok = false;
}

template<typename... Args>
void Monitor::log(const char* fmt, const Args& ... args)
{
	logMessageSignal(
		"[rosmon]",
		fmt::format(fmt, args...)
	);
}

void Monitor::updateStats()
{
	namespace fs = boost::filesystem;

	fs::directory_iterator it("/proc");
	fs::directory_iterator end;

	std::map<int, NodeMonitor::Ptr> nodeMap;
	for(auto& node : m_nodes)
	{
		if(node->pid() != -1)
			nodeMap[node->pid()] = node;

		node->beginStatUpdate();
	}

	for(auto& procInfo : m_processInfos)
		procInfo.second.active = false;

	for(; it != end; ++it)
	{
		fs::path statPath = (*it) / "stat";
		if(!fs::exists(statPath))
			continue;

		process_info::ProcessStat stat;
		if(!process_info::readStatFile(statPath.c_str(), &stat))
			continue;

		// Find corresponding node by the process group ID
		// (= process ID of the group leader process)
		auto it = nodeMap.find(stat.pgrp);
		if(it == nodeMap.end())
			continue;

		auto& node = it->second;

		// We need to store the stats and subtract the last one to get a time
		// delta
		auto infoIt = m_processInfos.find(stat.pid);
		if(infoIt == m_processInfos.end())
		{
			ProcessInfo info;
			info.stat = stat;
			info.active = true;
			m_processInfos[stat.pid] = info;
			continue;
		}

		const auto& oldStat = infoIt->second.stat;

		node->addCPUTime(stat.utime - oldStat.utime, stat.stime - oldStat.stime);
		node->addMemory(stat.mem_rss);

		infoIt->second.active = true;
		infoIt->second.stat = stat;
	}

	for(auto& node : m_nodes)
		node->endStatUpdate(process_info::kernel_hz());

	// Clean up old processes
	for(auto it = m_processInfos.begin(); it != m_processInfos.end();)
	{
		if(!it->second.active)
			it = m_processInfos.erase(it);
		else
			it++;
	}
}

}
}

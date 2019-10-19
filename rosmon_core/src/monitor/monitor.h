// Monitors execution of a launch file
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSMON_MONITOR_MONITOR_H
#define ROSMON_MONITOR_MONITOR_H

#include "../fd_watcher.h"
#include "../launch/launch_config.h"
#include "../log_event.h"

#include "node_monitor.h"
#include "linux_process_info.h"

#include <boost/signals2.hpp>

#include <ros/node_handle.h>

namespace rosmon
{

namespace monitor
{

class Monitor
{
public:
public:
	explicit Monitor(launch::LaunchConfig::ConstPtr config, FDWatcher::Ptr watcher);

	void setParameters();
	void start();
	void shutdown();
	void forceExit();
	bool allShutdown();

	double shutdownTimeout();

	inline bool ok() const
	{ return m_ok; }

	const std::vector<NodeMonitor::Ptr>& nodes() const
	{ return m_nodes; }
	std::vector<NodeMonitor::Ptr>& nodes()
	{ return m_nodes; }

	launch::LaunchConfig::ConstPtr config() const
	{ return m_config; }

	boost::signals2::signal<void(LogEvent)> logMessageSignal;
private:
	struct ProcessInfo
	{
		process_info::ProcessStat stat;
		bool active;
	};

	template<typename... Args>
	void log(const char* fmt, Args&& ... args);

	template<typename... Args>
	void logTyped(LogEvent::Type type, const char* fmt, Args&& ... args);

	void handleRequiredNodeExit(const std::string& name);

#if HAVE_STEADYTIMER
	void updateStats(const ros::SteadyTimerEvent& event);
#else
	void updateStats(const ros::WallTimerEvent& event);
#endif

	launch::LaunchConfig::ConstPtr m_config;

	ros::NodeHandle m_nh;
	FDWatcher::Ptr m_fdWatcher;

	std::vector<NodeMonitor::Ptr> m_nodes;

	bool m_ok;

#if HAVE_STEADYTIMER
	ros::SteadyTimer m_statTimer;
#else
	ros::WallTimer m_statTimer;
#endif

	std::map<int, ProcessInfo> m_processInfos;
};

}

}

#endif

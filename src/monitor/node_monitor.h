// Monitors a single node process
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSMON_MONITOR_NODE_MONITOR_H
#define ROSMON_MONITOR_NODE_MONITOR_H

#include "../launch/node.h"
#include "../fd_watcher.h"

#include <ros/node_handle.h>

#include <boost/signals2.hpp>
#include <boost/circular_buffer.hpp>

namespace rosmon
{

namespace monitor
{

class NodeMonitor
{
public:
	typedef std::shared_ptr<NodeMonitor> Ptr;
	typedef std::shared_ptr<NodeMonitor> ConstPtr;

	enum State
	{
		STATE_IDLE,
		STATE_RUNNING,
		STATE_CRASHED,
		STATE_WAITING
	};

	enum Command
	{
		CMD_RUN,
		CMD_STOP,
		CMD_RESTART
	};

	NodeMonitor(
		const launch::Node::ConstPtr& launchNode,
		const FDWatcher::Ptr& fdWatcher, ros::NodeHandle& nh);
	~NodeMonitor();

	std::vector<std::string> composeCommand() const;

	void start();
	void stop(bool restart = false);
	void restart();

	void shutdown();
	void forceExit();

	bool running() const;
	State state() const;

	void communicate();

	inline int fd()
	{ return m_fd; }

	inline bool coredumpAvailable() const
	{ return !m_debuggerCommand.empty(); }

	inline std::string debuggerCommand() const
	{ return m_debuggerCommand; }

	void launchDebugger();

	inline std::string name() const
	{ return m_launchNode->name(); }

	boost::signals2::signal<void(std::string,std::string)> logMessageSignal;
	boost::signals2::signal<void(std::string)> exitedSignal;
private:
	void log(const char* fmt, ...) __attribute__ (( format (printf, 2, 3) ));
	void checkStop();
	void gatherCoredump(int signal);

	launch::Node::ConstPtr m_launchNode;

	FDWatcher::Ptr m_fdWatcher;

	boost::circular_buffer<char> m_rxBuffer;

	int m_pid;
	int m_fd;
	int m_exitCode;

	ros::WallTimer m_stopCheckTimer;
	ros::WallTimer m_restartTimer;

	Command m_command;

	bool m_restarting;

	std::string m_debuggerCommand;
};

}

}

#endif

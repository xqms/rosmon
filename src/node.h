// Represents a node to be started
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSMON_NODE_H
#define ROSMON_NODE_H

#include <boost/shared_ptr.hpp>

#include <map>

#include <boost/circular_buffer.hpp>
#include <boost/signals2.hpp>

#include <ros/wall_timer.h>

#include "fd_watcher.h"

namespace rosmon
{

class Node
{
public:
	typedef boost::shared_ptr<Node> Ptr;

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

	Node(const FDWatcher::Ptr& fdWatcher, ros::NodeHandle& nh, const std::string& name, const std::string& package, const std::string& type);
	~Node();

	void addRemapping(const std::string& from, const std::string& to);
	void addExtraArguments(const std::string& argString);
	void setNamespace(const std::string& ns);
	void setExtraEnvironment(const std::map<std::string, std::string>& env);

	void setRespawn(bool respawn);
	void setRespawnDelay(const ros::WallDuration& respawnDelay);

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

	inline const std::string& name() const
	{ return m_name; }

	inline bool coredumpAvailable() const
	{ return !m_debuggerCommand.empty(); }

	inline std::string debuggerCommand() const
	{ return m_debuggerCommand; }

	void launchDebugger();

	boost::signals2::signal<void(std::string,std::string)> logMessageSignal;
	boost::signals2::signal<void(std::string)> exitedSignal;
private:
	void log(const char* fmt, ...) __attribute__ (( format (printf, 2, 3) ));
	void checkStop();
	void gatherCoredump(int signal);

	FDWatcher::Ptr m_fdWatcher;

	std::string m_name;
	std::string m_package;
	std::string m_type;

	std::string m_executable;

	std::string m_namespace;

	std::map<std::string, std::string> m_remappings;
	std::vector<std::string> m_extraArgs;

	std::map<std::string, std::string> m_extraEnvironment;

	boost::circular_buffer<char> m_rxBuffer;

	int m_pid;
	int m_fd;
	int m_exitCode;

	ros::WallTimer m_stopCheckTimer;
	ros::WallTimer m_restartTimer;

	Command m_command;

	bool m_restarting;

	bool m_respawn;
	ros::WallDuration m_respawnDelay;

	std::string m_debuggerCommand;
};

}

#endif

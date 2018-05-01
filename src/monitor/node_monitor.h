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

/**
 * @brief Monitors a single node process
 **/
class NodeMonitor
{
public:
	typedef std::shared_ptr<NodeMonitor> Ptr;
	typedef std::shared_ptr<NodeMonitor> ConstPtr;

	//! Process state
	enum State
	{
		STATE_IDLE,    //!< Idle (e.g. exited with code 0)
		STATE_RUNNING, //!< Running
		STATE_CRASHED, //!< Crashed (i.e. exited with code != 0)
		STATE_WAITING  //!< Waiting for automatic restart after crash
	};

	/**
	 * @brief Constructor
	 *
	 * @param launchNode Corresponding launch::Node instance
	 * @param fdWatcher FDWatcher instance to register in
	 * @param nh ros::NodeHandle to use for creating timers
	 **/
	NodeMonitor(
		launch::Node::ConstPtr launchNode,
		FDWatcher::Ptr fdWatcher, ros::NodeHandle& nh);
	~NodeMonitor();

	//! @name Starting & stopping
	//@{

	//! Start the node
	void start();

	//! Stop the node
	void stop(bool restart = false);

	//! Restart the node
	void restart();

	/**
	 * @brief Start shutdown sequence
	 *
	 * If the node is still running, this sends SIGINT.
	 **/
	void shutdown();

	/**
	 * @brief Finish shutdown sequence
	 *
	 * If the node is still running, this sends SIGKILL and prints a warning.
	 **/
	void forceExit();

	//! Is the node running?
	bool running() const;

	//! Get process state
	State state() const;
	//@}

	//! @name Debugging
	//@{

	//! Is a core dump available from a crash under rosmon control?
	inline bool coredumpAvailable() const
	{ return !m_debuggerCommand.empty(); }

	/**
	 * @brief What command should we use to debug the coredump?
	 *
	 * @sa coredumpAvailable()
	 **/
	inline std::string debuggerCommand() const
	{ return m_debuggerCommand; }

	/**
	 * @brief Launch gdb interactively
	 *
	 * This opens gdb in a new terminal window. If X11 is not available, the
	 * gdb command is emitted via logMessageSignal().
	 *
	 * If a coredump is available (see coredumpAvailable()), gdb is launched
	 * against the coredump instead of the running process.
	 **/
	void launchDebugger();
	//@}

	//! @name Statistics
	//@{

	void beginStatUpdate();
	void addCPUTime(uint64_t userTime, uint64_t systemTime);
	void addMemory(uint64_t memoryBytes);
	void endStatUpdate(uint64_t elapsedTime);

	/**
	 * @brief Estimate of the userspace load
	 *
	 * This is the fraction of CPU time spent in userspace code of this node.
	 * Note that this value is relative to one CPU core.
	 **/
	inline double userLoad() const
	{ return m_userLoad; }

	/**
	 * @brief Estimate of the kernelspace load
	 *
	 * This is the fraction of CPU time spent in kernelspace code of this node.
	 * Note that this value is relative to one CPU core.
	 **/
	inline double systemLoad() const
	{ return m_systemLoad; }

	/**
	 * @brief Total system memory used by the node
	 *
	 * This includes child processes. Only resident memory is counted
	 * (i.e. how much physical memory is used).
	 *
	 * @return Memory usage in bytes
	 **/
	inline double memory() const
	{ return m_memory; }

	inline unsigned int restartCount() const
	{ return m_restartCount; }

	//@}

	//! Node name
	inline std::string name() const
	{ return m_launchNode->name(); }

	//! Node PID
	inline int pid() const
	{ return m_pid; }

	/**
	 * @brief Logging signal
	 *
	 * Contains a log message (node name, message) to be printed or saved in a log file.
	 **/
	boost::signals2::signal<void(std::string,std::string)> logMessageSignal;

	//! Signalled whenever the process exits.
	boost::signals2::signal<void(std::string)> exitedSignal;
private:
	enum Command
	{
		CMD_RUN,
		CMD_STOP,
		CMD_RESTART
	};

	std::vector<std::string> composeCommand() const;

	void communicate();

	void log(const char* fmt, ...) __attribute__ (( format (printf, 2, 3) ));
	void checkStop();
	void gatherCoredump(int signal);

	launch::Node::ConstPtr m_launchNode;

	FDWatcher::Ptr m_fdWatcher;

	boost::circular_buffer<char> m_rxBuffer;

	int m_pid = -1;
	int m_fd = -1;
	int m_exitCode;

	ros::WallTimer m_stopCheckTimer;
	ros::WallTimer m_restartTimer;

	Command m_command;

	bool m_restarting;

	std::string m_debuggerCommand;

	unsigned int m_restartCount = 0;

	uint64_t m_userTime = 0;
	uint64_t m_systemTime = 0;
	double m_userLoad = 0.0;
	double m_systemLoad = 0.0;
	uint64_t m_memory = 0;

	std::string m_processWorkingDirectory;
};

}

}

#endif

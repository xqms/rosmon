// Monitors a single node process
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "node_monitor.h"

#include <cerrno>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <sstream>

#include <fcntl.h>
#include <glob.h>
#include <pty.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/resource.h>
#include <sys/utsname.h>
#include <sys/prctl.h>
#include <unistd.h>
#include <wordexp.h>

#include <boost/tokenizer.hpp>
#include <boost/range.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/filesystem.hpp>

#include <fmt/format.h>

#define TASK_COMM_LEN 16 // from linux/sched.h

namespace fs = boost::filesystem;

namespace
{
	template<typename... Args>
	std::runtime_error error(const char* fmt, const Args& ... args)
	{
		return std::runtime_error(fmt::format(fmt, args...));
	}

	// finally() from C++ core guidelines
	template <class F>
	class final_act
	{
	public:
		explicit final_act(F f) noexcept
		: f_(std::move(f)), invoke_(true) {}

		final_act(final_act&& other) noexcept
		: f_(std::move(other.f_)),
		invoke_(other.invoke_)
		{
			other.invoke_ = false;
		}

		final_act(const final_act&) = delete;
		final_act& operator=(const final_act&) = delete;

		~final_act() noexcept
		{
			if (invoke_) f_();
		}

	private:
		F f_;
		bool invoke_;
	};
	template <class F>
	inline final_act<F> finally(const F& f) noexcept
	{
		return final_act<F>(f);
	}

	template <class F>
	inline final_act<F> finally(F&& f) noexcept
	{
		return final_act<F>(std::forward<F>(f));
	}

	static bool g_corePatternAnalyzed = false;
	static bool g_coreIsRelative = true;
	static bool g_coreIsSystemd = false;
}

namespace rosmon
{
namespace monitor
{

NodeMonitor::NodeMonitor(launch::Node::ConstPtr launchNode, FDWatcher::Ptr fdWatcher, ros::NodeHandle& nh)
 : m_launchNode(std::move(launchNode))
 , m_fdWatcher(std::move(fdWatcher))
 , m_rxBuffer(4096)
 , m_stderrBuffer(4096)
 , m_exitCode(0)
 , m_command(CMD_STOP) // we start in stopped state
 , m_restarting(false)
 , m_muted(m_launchNode->isMuted())
{
	m_restartTimer = nh.createWallTimer(ros::WallDuration(1.0), boost::bind(&NodeMonitor::start, this), false, false);
	m_stopCheckTimer = nh.createWallTimer(ros::WallDuration(m_launchNode->stopTimeout()), boost::bind(&NodeMonitor::checkStop, this));

	m_processWorkingDirectory = m_launchNode->workingDirectory();

	if(!g_corePatternAnalyzed)
	{
		char core_pattern[256];
		int core_fd = open("/proc/sys/kernel/core_pattern", O_RDONLY | O_CLOEXEC);
		if(core_fd < 0)
		{
			logTyped(LogEvent::Type::Error, "could not open /proc/sys/kernel/core_pattern: {}", strerror(errno));
			return;
		}

		int bytes = read(core_fd, core_pattern, sizeof(core_pattern)-1);
		close(core_fd);

		if(bytes < 1)
		{
			logTyped(LogEvent::Type::Error, "Could not read /proc/sys/kernel/core_pattern: {}", strerror(errno));
			return;
		}

		g_coreIsRelative = (core_pattern[0] != '/');

		if(std::string(core_pattern).find("systemd-coredump") != std::string::npos)
			g_coreIsSystemd = true;

		g_corePatternAnalyzed = true;
	}
}

NodeMonitor::~NodeMonitor()
{
	if(m_pid != -1)
	{
		kill(m_pid, SIGKILL);
	}
}

std::vector<std::string> NodeMonitor::composeCommand() const
{
	if(m_launchNode->executable().empty())
	{
		throw error("Could not find node '{}' in package '{}'",
			m_launchNode->type(), m_launchNode->package()
		);
	}

	// Start with the launch prefix...
	std::vector<std::string> cmd = m_launchNode->launchPrefix();

	// add executable file
	cmd.push_back(m_launchNode->executable());

	// add extra arguments from 'args'
	auto args = m_launchNode->extraArguments();
	std::copy(args.begin(), args.end(), std::back_inserter(cmd));

	// add parameter for node name
	cmd.push_back("__name:=" + m_launchNode->name());

	// and finally add remappings.
	for(auto map : m_launchNode->remappings())
	{
		cmd.push_back(map.first + ":=" + map.second);
	}

	return cmd;
}

void NodeMonitor::start()
{
	m_command = CMD_RUN;

	m_stopCheckTimer.stop();
	m_restartTimer.stop();
	m_restarting = false;

	if(running())
		return;

	if(m_launchNode->coredumpsEnabled() && g_coreIsRelative && m_processWorkingDirectory.empty())
	{
		char tmpfile[256];
		strncpy(tmpfile, "/tmp/rosmon-node-XXXXXX", sizeof(tmpfile));
		tmpfile[sizeof(tmpfile)-1] = 0;
		m_processWorkingDirectory = mkdtemp(tmpfile);
		m_processWorkingDirectoryCreated = true;
	}

	ROS_INFO("rosmon: starting '%s'", fullName().c_str());

	if(!m_firstStart)
		logMessageSignal({"[rosmon]", fmt::format("Starting node {}", fullName()), LogEvent::Type::Info});

	m_firstStart = false;

	std::vector<std::string> cmd = composeCommand();

	std::vector<char*> args;
	auto argsCleaner = finally([&args](){
		for(char* arg : args)
			free(arg);
	});

	// Open pseudo-terminal
	int master, slave;
	std::tie(master, slave) = createPTY();

	// For stderr, we open a second PTY
	int stderr_master, stderr_slave;
	std::tie(stderr_master, stderr_slave) = createPTY();

	// Compose args
	{
		args.push_back(strdup("rosrun"));
		args.push_back(strdup("rosmon_core"));
		args.push_back(strdup("_shim"));

		args.push_back(strdup("--tty"));
		args.push_back(strdup(fmt::format("{}", slave).c_str()));

		args.push_back(strdup("--stderr"));
		args.push_back(strdup(fmt::format("{}", stderr_slave).c_str()));

		if(!m_launchNode->namespaceString().empty())
		{
			args.push_back(strdup("--namespace"));
			args.push_back(strdup(m_launchNode->namespaceString().c_str()));
		}

		for(auto& pair : m_launchNode->extraEnvironment())
		{
			args.push_back(strdup("--env"));
			args.push_back(strdup(fmt::format("{}={}", pair.first, pair.second).c_str()));
		}

		if(m_launchNode->coredumpsEnabled())
		{
			args.push_back(strdup("--coredump"));

			if(g_coreIsRelative)
			{
				args.push_back(strdup("--coredump-relative"));
				args.push_back(strdup(m_processWorkingDirectory.c_str()));
			}
		}

		args.push_back(strdup("--run"));

		for(auto& c : cmd)
			args.push_back(strdup(c.c_str()));

		args.push_back(nullptr);
	}

	// Fork!
	int pid = fork();
	if(pid < 0)
		throw error("Could not fork(): {}", strerror(errno));

	if(pid == 0)
	{
		close(master);
		close(stderr_master);

		if(execvp("rosrun", args.data()) != 0)
		{
			std::stringstream ss;
			for(const auto& part : cmd)
				ss << part << " ";

			fmt::print(stderr, "Could not execute '{}': {}\n", ss.str(), strerror(errno));
		}

		// We should not end up here...
		std::abort();
	}

	// Parent
	close(slave);
	close(stderr_slave);

	m_fd = master;
	m_stderrFD = stderr_master;
	m_pid = pid;
	m_fdWatcher->registerFD(m_fd, boost::bind(&NodeMonitor::communicate, this));
	m_fdWatcher->registerFD(m_stderrFD, boost::bind(&NodeMonitor::communicateStderr, this));
}

void NodeMonitor::stop(bool restart)
{
	if(restart)
		m_command = CMD_RESTART;
	else
		m_command = CMD_STOP;

	m_stopCheckTimer.stop();
	m_restartTimer.stop();

	if(!running())
		return;

	logMessageSignal({"[rosmon]", fmt::format("Stopping node {}", fullName()), LogEvent::Type::Info});

	// kill(-pid) sends the signal to all processes in the process group
	kill(-m_pid, SIGINT);

	m_stopCheckTimer.start();
}

void NodeMonitor::checkStop()
{
	if(running())
	{
		logTyped(LogEvent::Type::Warning, "required SIGKILL");
		kill(m_pid, SIGKILL);
	}

	m_stopCheckTimer.stop();
}

void NodeMonitor::restart()
{
	m_stopCheckTimer.stop();
	m_restartTimer.stop();

	if(running())
		stop(true);
	else
		start();
}

void NodeMonitor::shutdown()
{
	if(m_pid != -1)
	{
		kill(-m_pid, SIGINT);
	}
}

void NodeMonitor::forceExit()
{
	if(m_pid != -1)
	{
		kill(-m_pid, SIGKILL);
	}
}

bool NodeMonitor::running() const
{
	return m_pid != -1;
}

NodeMonitor::State NodeMonitor::state() const
{
	if(running())
		return STATE_RUNNING;

	if(m_restarting)
		return STATE_WAITING;

	if(m_exitCode == 0)
		return STATE_IDLE;

	return STATE_CRASHED;
}

void NodeMonitor::communicateStderr()
{
	auto handleByte = [&](char c){
		m_stderrBuffer.push_back(c);
		if(c == '\n')
		{
			m_stderrBuffer.push_back(0);
			m_stderrBuffer.linearize();

			auto one = m_stderrBuffer.array_one();

			LogEvent event{fullName(), one.first};
			event.muted = isMuted();
			event.channel = LogEvent::Channel::Stderr;

			logMessageSignal(std::move(event));

			m_stderrBuffer.clear();
		}
	};

	char buf[1024];
	int bytes = read(m_stderrFD, buf, sizeof(buf));

	if(bytes == 0 || (bytes < 0 && errno == EIO))
	{
		// Flush out any remaining stderr
		if(!m_stderrBuffer.empty())
			handleByte('\n');

		m_fdWatcher->removeFD(m_stderrFD);
        close(m_stderrFD);
        m_stderrFD = -1;
		return; // handled in communicate()
	}

	if(bytes < 0)
		throw error("{}: Could not read: {}", fullName(), strerror(errno));

	for(int i = 0; i < bytes; ++i)
	{
		handleByte(buf[i]);
	}
}

void NodeMonitor::communicate()
{
	auto handleByte = [&](char c){
		m_rxBuffer.push_back(c);
		if(c == '\n')
		{
			m_rxBuffer.push_back(0);
			m_rxBuffer.linearize();

			auto one = m_rxBuffer.array_one();

			LogEvent event{fullName(), one.first};
			event.muted = isMuted();
			event.channel = LogEvent::Channel::Stdout;
			event.showStdout = m_launchNode->stdoutDisplayed();

			logMessageSignal(std::move(event));

			m_rxBuffer.clear();
		}
	};

	char buf[1024];
	int bytes = read(m_fd, buf, sizeof(buf));

	if(bytes == 0 || (bytes < 0 && errno == EIO))
	{
		int status;

		while(true)
		{
			if(waitpid(m_pid, &status, 0) > 0)
				break;

			if(errno == EINTR || errno == EAGAIN)
				continue;

			throw error("{}: Could not waitpid(): {}", fullName(), strerror(errno));
		}

		// Flush out any remaining stdout
		if(!m_rxBuffer.empty())
			handleByte('\n');

		if(WIFEXITED(status))
		{
			auto type = (WEXITSTATUS(status) == 0) ? LogEvent::Type::Info : LogEvent::Type::Error;
			logTyped(type, "{} exited with status {}", fullName(), WEXITSTATUS(status));
			ROS_INFO("rosmon: %s exited with status %d", fullName().c_str(), WEXITSTATUS(status));
			m_exitCode = WEXITSTATUS(status);
		}
		else if(WIFSIGNALED(status))
		{
			logTyped(LogEvent::Type::Error, "{} died from signal {}", fullName(), WTERMSIG(status));
			ROS_ERROR("rosmon: %s died from signal %d", fullName().c_str(), WTERMSIG(status));
			m_exitCode = 255;
		}

#ifdef WCOREDUMP
		if(WCOREDUMP(status))
		{
			if(!m_launchNode->launchPrefix().empty())
			{
				logTyped(LogEvent::Type::Info, "{} used launch-prefix, not collecting core dump as it is probably useless.", fullName());
			}
			else
			{
				// We have a chance to find the core dump...
				logTyped(LogEvent::Type::Info, "{} left a core dump", fullName());
				gatherCoredump(WTERMSIG(status));
			}
		}
#endif

		if(m_processWorkingDirectoryCreated)
		{
			// Our removal strategy is two-fold: After a process exits,
			// we immediately try to delete the temporary working directory.
			// If that fails (e.g. because there is a core dump in there),
			// we remember the directory in m_lastWorkingDirectory.

			// and then delete it recursively on the next process exit.
			// That way, we always keep the last core dump around, but prevent
			// infinite pile-up.
			if(!m_lastWorkingDirectory.empty())
			{
				boost::system::error_code error;
				boost::filesystem::remove_all(m_lastWorkingDirectory, error);
				if(error)
				{
					logTyped(LogEvent::Type::Warning,
						"Could not remove old working directory '{}' after process died twice: {}",
						m_lastWorkingDirectory, error.message()
					);
				}

				m_lastWorkingDirectory.clear();
			}

			if(rmdir(m_processWorkingDirectory.c_str()) != 0)
			{
				logTyped(LogEvent::Type::Warning, "Could not remove process working directory '{}' after process exit: {}",
					m_processWorkingDirectory, strerror(errno)
				);

				m_lastWorkingDirectory = m_processWorkingDirectory;
			}

			m_processWorkingDirectory.clear();
		}

		m_pid = -1;
		m_fdWatcher->removeFD(m_fd);
		close(m_fd);
		m_fd = -1;

		if(m_command == CMD_RESTART || (m_command == CMD_RUN && m_launchNode->respawn()))
		{
			if(m_command == CMD_RESTART)
				m_restartTimer.setPeriod(ros::WallDuration(1.0));
			else
				m_restartTimer.setPeriod(m_launchNode->respawnDelay());

			m_restartCount++;
			m_restartTimer.start();
			m_restarting = true;
		}

		exitedSignal(fullName());

		return;
	}

	if(bytes < 0)
		throw error("{}: Could not read: {}", fullName(), strerror(errno));

	for(int i = 0; i < bytes; ++i)
	{
		handleByte(buf[i]);
	}
}

template<typename... Args>
void NodeMonitor::log(const char* format, Args&& ... args)
{
	logMessageSignal({fullName(), fmt::format(format, std::forward<Args>(args)...)});
}

template<typename... Args>
void NodeMonitor::logTyped(LogEvent::Type type, const char* format, Args&& ... args)
{
	logMessageSignal({fullName(), fmt::format(format, std::forward<Args>(args)...), type});
}

static boost::iterator_range<std::string::const_iterator>
corePatternFormatFinder(std::string::const_iterator begin, std::string::const_iterator end)
{
	for(; begin != end && begin+1 != end; ++begin)
	{
		if(*begin == '%')
			return {begin, begin+2};
	}

	return {end, end};
}

void NodeMonitor::gatherCoredump(int signal)
{
	// If systemd-coredump is enabled, our job is easy
	if(g_coreIsSystemd)
	{
		m_debuggerCommand = fmt::format("coredumpctl gdb COREDUMP_PID={}", m_pid);
		return;
	}

	// Otherwise we have to find the core ourselves...
	char core_pattern[256];
	int core_fd = open("/proc/sys/kernel/core_pattern", O_RDONLY | O_CLOEXEC);
	if(core_fd < 0)
	{
		logTyped(LogEvent::Type::Error, "could not open /proc/sys/kernel/core_pattern: {}", strerror(errno));
		return;
	}

	int bytes = read(core_fd, core_pattern, sizeof(core_pattern)-1);
	close(core_fd);

	if(bytes < 1)
	{
		log("Could not read /proc/sys/kernel/core_pattern: {}", strerror(errno));
		return;
	}

	core_pattern[bytes-1] = 0; // Strip off the newline at the end

	if(core_pattern[0] == '|')
	{
		// This may be apport, but apport still writes a "core" file if the
		// limit is set appropriately.
		strncpy(core_pattern, "core", sizeof(core_pattern));
	}

	auto formatter = [&](boost::iterator_range<std::string::const_iterator> match) -> std::string {
		char code = *(match.begin()+1);

		switch(code)
		{
			case '%':
				return "%";
			case 'p':
				return std::to_string(m_pid);
			case 'u':
				return std::to_string(getuid());
			case 'g':
				return std::to_string(getgid());
			case 's':
				return std::to_string(signal);
			case 't':
				return "*"; // No chance
			case 'h':
			{
				utsname uts;
				memset(&uts, 0, sizeof(uts));
				if(uname(&uts) != 0)
					return "*";

				return uts.nodename;
			}
			case 'e':
				return m_launchNode->type().substr(0, TASK_COMM_LEN-1);
			case 'E':
			{
				std::string executable = m_launchNode->executable();
				boost::replace_all(executable, "/", "!");
				return executable;
			}
			case 'c':
			{
				rlimit limit;
				memset(&limit, 0, sizeof(limit));
				getrlimit(RLIMIT_CORE, &limit);

				// core limit is set to the maximum above
				return std::to_string(limit.rlim_max);
			}
			default:
				return "*";
		}
	};

	std::string coreGlob = boost::find_format_all_copy(std::string(core_pattern), corePatternFormatFinder, formatter);

	// If the pattern is not absolute, it is relative to our node's cwd.
	if(coreGlob[0] != '/')
		coreGlob = m_processWorkingDirectory + "/" + coreGlob;

	log("Determined pattern '{}'", coreGlob);

	glob_t results;
	memset(&results, 0, sizeof(results));
	int ret = glob(coreGlob.c_str(), GLOB_NOSORT, nullptr, &results);

	if(ret != 0 || results.gl_pathc == 0)
	{
		logTyped(LogEvent::Type::Warning, "Could not find a matching core file :-(");
		globfree(&results);
		return;
	}

	if(results.gl_pathc > 1)
	{
		logTyped(LogEvent::Type::Info, "Found multiple matching core files :-(");
		globfree(&results);
		return;
	}

	std::string coreFile = results.gl_pathv[0];
	globfree(&results);

	logTyped(LogEvent::Type::Info, "Found core file '{}'", coreFile);

	std::stringstream ss;

	ss << "gdb " << m_launchNode->executable() << " " << coreFile;

	m_debuggerCommand = ss.str();
}

std::pair<int,int> NodeMonitor::createPTY()
{
	int master, slave;

	// NOTE: We are not using forkpty() here, as it is probably not safe in
	//  a multi-threaded process (see
	//  https://www.linuxprogrammingblog.com/threads-and-fork-think-twice-before-using-them)

	if(openpty(&master, &slave, nullptr, nullptr, nullptr) == -1)
		throw error("Could not open pseudo terminal for child process: {}", strerror(errno));

	// On Linux, a new unix98 pty is initialized with the output flag ONLCR set,
	// which converts \n to \r\n
	// Disable this bahavior by clearing the flag
	struct termios termios;
	if(tcgetattr(slave, &termios) == -1)
		throw error("Could not get PTY slave attributes: {}", strerror(errno));

	termios.c_oflag &= ~ONLCR;

	if(tcsetattr(slave, TCSANOW, &termios) == -1)
		throw error("Could not set PTY slave attributes: {}", strerror(errno));

	return {master,slave};
}

void NodeMonitor::launchDebugger()
{
	std::string cmd;

	if(coredumpAvailable())
		cmd = m_debuggerCommand;
	else
	{
		std::stringstream ss;
		ss << "gdb -p " << m_pid;
		cmd = ss.str();
	}

	if(!getenv("DISPLAY"))
	{
		logTyped(LogEvent::Type::Info, "No X11 available, run gdb yourself: {}", cmd);
	}
	else
	{
		char* envTerm = getenv("ROSMON_DEBUGGER_TERMINAL");
		std::string term = "xterm -e";
		if(envTerm)
			term = envTerm;
		else if(getenv("KONSOLE_DBUS_SESSION"))
			term = "konsole -e";
		else if(getenv("VTE_VERSION"))
			term = "gnome-terminal -e";

		logTyped(LogEvent::Type::Info, "Launching debugger: '{}'", cmd);

		// system() is not particularly elegant here, but we trust our cmd.
		if(system((term + " '" + cmd + "' &").c_str()) != 0)
		{
			logTyped(LogEvent::Type::Error, "Could not launch debugger");
		}
	}
}


void NodeMonitor::beginStatUpdate()
{
	m_userTime = 0;
	m_systemTime = 0;
	m_memory = 0;
}

void NodeMonitor::addCPUTime(uint64_t userTime, uint64_t systemTime)
{
	m_userTime += userTime;
	m_systemTime += systemTime;
}

void NodeMonitor::addMemory(uint64_t memoryBytes)
{
	m_memory += memoryBytes;
}

void NodeMonitor::endStatUpdate(double elapsedTimeInTicks)
{
	m_userLoad = m_userTime / elapsedTimeInTicks;
	m_systemLoad = m_systemTime / elapsedTimeInTicks;
}

void NodeMonitor::setMuted(bool muted)
{
	m_muted = muted;
}

}
}

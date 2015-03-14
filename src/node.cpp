// Represents a node to be started
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "node.h"

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <stdarg.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <pty.h>

#include <sstream>

static std::runtime_error error(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	char str[1024];

	vsnprintf(str, sizeof(str), fmt, args);

	va_end(args);

	return std::runtime_error(str);
}

namespace rosmon
{

Node::Node(const std::string& name, const std::string& package, const std::string& type)
 : m_name(name)
 , m_package(package)
 , m_type(type)
 , m_rxBuffer(4096)
 , m_pid(-1)
{
}

Node::~Node()
{
	if(m_pid != -1)
	{
		kill(m_pid, SIGKILL);
	}
}

void Node::addRemapping(const std::string& from, const std::string& to)
{
	m_remappings[from] = to;
}

std::vector<std::string> Node::composeCommand() const
{
	std::vector<std::string> cmd{
		"rosrun",
		m_package,
		m_type,
		"__name:=" + m_name
	};

	for(auto map : m_remappings)
	{
		cmd.push_back(map.first + ":=" + map.second);
	}

	return cmd;
}

void Node::start()
{
	int pid = forkpty(&m_fd, NULL, NULL, NULL);
	if(pid < 0)
		throw error("Could not fork with forkpty(): %s", strerror(errno));

	if(pid == 0)
	{
		std::vector<std::string> cmd = composeCommand();

		char* path = strdup(cmd[0].c_str());

		std::vector<char*> ptrs(cmd.size());

		for(unsigned int i = 0; i < cmd.size(); ++i)
			ptrs[i] = strdup(cmd[i].data());
		ptrs.push_back(nullptr);

		if(execvp(path, ptrs.data()) != 0)
		{
			std::stringstream ss;
			for(auto part : cmd)
				ss << part << " ";

			fprintf(stderr, "Could not execute '%s': %s\n", ss.str().c_str(), strerror(errno));
		}
		exit(1);
	}

	// Parent
	m_pid = pid;
}

void Node::shutdown()
{
	if(m_pid != -1)
		kill(m_pid, SIGINT);
}

bool Node::running() const
{
	return m_pid != -1;
}

void Node::communicate()
{
	char buf[1024];
	int bytes = read(m_fd, buf, sizeof(buf));

	if(bytes == 0 || (bytes < 0 && errno == EIO))
	{
		int status;

		while(1)
		{
			if(waitpid(m_pid, &status, 0) > 0)
				break;
			else
			{
				if(errno == EINTR || errno == EAGAIN)
					continue;

				throw error("%s: Could not waitpid(): %s", m_name.c_str(), strerror(errno));
			}
		}

		if(WIFEXITED(status))
			printf("%20s: Exited with status %d\n", m_name.c_str(), WEXITSTATUS(status));
		else if(WIFSIGNALED(status))
			printf("%20s: Exited with status %d\n", m_name.c_str(), WTERMSIG(status));

		m_pid = -1;
		return;
	}

	if(bytes < 0)
		throw error("%s: Could not read: %s", m_name.c_str(), strerror(errno));

	for(int i = 0; i < bytes; ++i)
	{
		m_rxBuffer.push_back(buf[i]);
		if(buf[i] == '\n')
		{
			printf("%20s: ", m_name.c_str());

			auto one = m_rxBuffer.array_one();
			fwrite(one.first, 1, one.second, stdout);

			auto two = m_rxBuffer.array_two();
			fwrite(two.first, 1, two.second, stdout);

			m_rxBuffer.clear();
		}
	}
}

}

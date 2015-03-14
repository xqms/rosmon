// Represents a node to be started
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSMON_NODE_H
#define ROSMON_NODE_H

#include <boost/shared_ptr.hpp>

#include <map>

#include <boost/circular_buffer.hpp>
#include <boost/signals2.hpp>

namespace rosmon
{

class Node
{
public:
	typedef boost::shared_ptr<Node> Ptr;

	Node(const std::string& name, const std::string& package, const std::string& type);
	~Node();

	void addRemapping(const std::string& from, const std::string& to);
	void addExtraArguments(const std::string& argString);
	void setNamespace(const std::string& ns);

	std::vector<std::string> composeCommand() const;

	void start();
	void shutdown();
	void forceExit();

	bool running() const;

	void communicate();

	inline int fd()
	{ return m_fd; }

	inline const std::string& name() const
	{ return m_name; }

	boost::signals2::signal<void(std::string,std::string)> logMessageSignal;
private:
	void log(const char* fmt, ...) __attribute__ (( format (printf, 2, 3) ));

	std::string m_name;
	std::string m_package;
	std::string m_type;

	std::string m_namespace;

	std::map<std::string, std::string> m_remappings;
	std::vector<std::string> m_extraArgs;

	boost::circular_buffer<char> m_rxBuffer;

	int m_pid;
	int m_fd;
};

}

#endif

// Represents a node to be started
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSMON_NODE_H
#define ROSMON_NODE_H

#include <boost/shared_ptr.hpp>

#include <map>

#include <boost/circular_buffer.hpp>

namespace rosmon
{

class Node
{
public:
	typedef boost::shared_ptr<Node> Ptr;

	Node(const std::string& name, const std::string& package, const std::string& type);
	~Node();

	void addRemapping(const std::string& from, const std::string& to);

	std::vector<std::string> composeCommand() const;

	void start();
	void shutdown();

	bool running() const;

	void communicate();

	inline int fd()
	{ return m_fd; }
private:
	std::string m_name;
	std::string m_package;
	std::string m_type;

	std::map<std::string, std::string> m_remappings;

	boost::circular_buffer<char> m_rxBuffer;

	int m_pid;
	int m_fd;
};

}

#endif

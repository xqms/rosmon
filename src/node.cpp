// Represents a node to be started
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "node.h"

namespace rosmon
{

Node::Node(const std::string& name, const std::string& package, const std::string& type)
 : m_name(name)
 , m_package(package)
 , m_type(type)
{
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

}

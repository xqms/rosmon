// Test utils for nodes
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "node_utils.h"

#include <sstream>

#include <catch_ros/catch.hpp>

std::string printMapping(const std::map<std::string, std::string>& mapping)
{
	std::stringstream ss;
	ss << "{ ";
	for(auto& pair : mapping)
	{
		ss << "'" << pair.first << "':='" << pair.second << "', ";
	}
	ss << "}";

	return ss.str();
}

rosmon::launch::Node::Ptr getNode(const std::vector<rosmon::launch::Node::Ptr>& nodes, const std::string& name, const std::string& namespaceString)
{
	rosmon::launch::Node::Ptr ret;

	INFO("Looking for node '" << name << "' in namespace '" << namespaceString << "'");

	for(auto& node : nodes)
	{
		if(node->namespaceString() != namespaceString)
			continue;

		if(node->name() == name)
		{
			REQUIRE(!ret);
			ret = node;
		}
	}

	REQUIRE(ret);
	return ret;
}

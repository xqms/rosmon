// Test utils for nodes
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NODE_UTILS_H
#define NODE_UTILS_H

#include <catch_ros/catch.hpp>

#include "../../src/launch/node.h"

namespace Catch
{
	template<>
	struct StringMaker<rosmon::launch::Node::Ptr>
	{
		static std::string convert(const rosmon::launch::Node::Ptr& node)
		{
			if(node)
				return "NodePtr(name='" + node->name() + "', namespace='" + node->namespaceString() + "')";
			else
				return "NodePtr()";
		}
	};

	template<>
	struct StringMaker<std::vector<rosmon::launch::Node::Ptr>>
	{
		static std::string convert(const std::vector<rosmon::launch::Node::Ptr>& nodes)
		{
			std::stringstream ss;
			ss << "{ ";
			for(auto& n : nodes)
				ss << StringMaker<rosmon::launch::Node::Ptr>::convert(n) << ", ";
			ss << "}";

			return ss.str();
		}
	};
}

std::string printMapping(const std::map<std::string, std::string>& mapping);

rosmon::launch::Node::Ptr getNode(const std::vector<rosmon::launch::Node::Ptr>& nodes, const std::string& name, const std::string& namespaceString="");

#endif

// Represents a node to be started
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NODE_H
#define NODE_H

#include <boost/shared_ptr.hpp>

namespace rosmon
{

class Node
{
public:
	typedef boost::shared_ptr<Node> Ptr;
};

}

#endif


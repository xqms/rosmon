// Provides a ROS interface for controlling rosmon
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "ros_interface.h"

#include <rosmon/State.h>

namespace rosmon
{

ROSInterface::ROSInterface(LaunchConfig* config)
 : m_config(config)
 , m_nh("~")
{
	m_updateTimer = m_nh.createWallTimer(ros::WallDuration(1.0), boost::bind(&ROSInterface::update, this));

	m_pub_state = m_nh.advertise<rosmon::State>("state", 10, true);
}

ROSInterface::~ROSInterface()
{
}

void ROSInterface::update()
{
	rosmon::State state;
	state.header.stamp = ros::Time::now();

	for(auto node : m_config->nodes())
	{
		rosmon::NodeState nstate;
		nstate.name = node->name();

		switch(node->state())
		{
			case Node::STATE_RUNNING:
				nstate.state = nstate.RUNNING;
				break;
			case Node::STATE_CRASHED:
				nstate.state = nstate.CRASHED;
				break;
			case Node::STATE_IDLE:
				nstate.state = nstate.IDLE;
				break;
			case Node::STATE_WAITING:
				nstate.state = nstate.WAITING;
				break;
			default:
				nstate.state = nstate.IDLE;
				break;
		}

		state.nodes.push_back(nstate);
	}

	m_pub_state.publish(state);
}

}


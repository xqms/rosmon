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
		nstate.state = node->running() ? (int)nstate.RUNNING : (int)nstate.CRASHED;

		state.nodes.push_back(nstate);
	}

	m_pub_state.publish(state);
}

}


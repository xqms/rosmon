// Provides a ROS interface for controlling rosmon
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "ros_interface.h"

#include <rosmon/State.h>

#include <algorithm>

namespace rosmon
{

ROSInterface::ROSInterface(LaunchConfig* config)
 : m_config(config)
 , m_nh("~")
{
	m_updateTimer = m_nh.createWallTimer(ros::WallDuration(1.0), boost::bind(&ROSInterface::update, this));

	m_pub_state = m_nh.advertise<rosmon::State>("state", 10, true);

	m_srv_startStop = m_nh.advertiseService("start_stop", &ROSInterface::handleStartStop, this);
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

bool ROSInterface::handleStartStop(StartStopRequest& req, StartStopResponse& resp)
{
	auto it = std::find_if(
		m_config->nodes().begin(), m_config->nodes().end(),
		[&](const Node::Ptr& n){ return n->name() == req.node; }
	);

	if(it == m_config->nodes().end())
		return false;

	switch(req.action)
	{
		case StartStopRequest::START:
			(*it)->start();
			break;
		case StartStopRequest::STOP:
			(*it)->stop();
			break;
		case StartStopRequest::RESTART:
			(*it)->restart();
			break;
	}

	return true;
}

void ROSInterface::shutdown()
{
	m_updateTimer.stop();

	// Send empty state packet to clear the GUI
	rosmon::State state;
	state.header.stamp = ros::Time::now();
	m_pub_state.publish(state);

	// HACK: Currently there is no way to make sure that we sent a message.
	usleep(200 * 1000);
}

}


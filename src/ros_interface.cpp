// Provides a ROS interface for controlling rosmon
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "ros_interface.h"

#include <rosmon/State.h>

#include <algorithm>

namespace rosmon
{

ROSInterface::ROSInterface(monitor::Monitor* monitor)
 : m_monitor(monitor)
 , m_nh("~")
{
	m_updateTimer = m_nh.createWallTimer(ros::WallDuration(1.0), boost::bind(&ROSInterface::update, this));

	m_pub_state = m_nh.advertise<rosmon::State>("state", 10, true);

	m_srv_startStop = m_nh.advertiseService("start_stop", &ROSInterface::handleStartStop, this);
}

void ROSInterface::update()
{
	rosmon::State state;
	state.header.stamp = ros::Time::now();

	for(auto& node : m_monitor->nodes())
	{
		rosmon::NodeState nstate;
		nstate.name = node->name();

		switch(node->state())
		{
			case monitor::NodeMonitor::STATE_RUNNING:
				nstate.state = nstate.RUNNING;
				break;
			case monitor::NodeMonitor::STATE_CRASHED:
				nstate.state = nstate.CRASHED;
				break;
			case monitor::NodeMonitor::STATE_IDLE:
				nstate.state = nstate.IDLE;
				break;
			case monitor::NodeMonitor::STATE_WAITING:
				nstate.state = nstate.WAITING;
				break;
			default:
				nstate.state = nstate.IDLE;
				break;
		}

		nstate.restart_count = node->restartCount();

		nstate.user_load = node->userLoad();
		nstate.system_load = node->systemLoad();

		nstate.memory = node->memory();

		state.nodes.push_back(nstate);
	}

	m_pub_state.publish(state);
}

bool ROSInterface::handleStartStop(StartStopRequest& req, StartStopResponse&)
{
	auto it = std::find_if(
		m_monitor->nodes().begin(), m_monitor->nodes().end(),
		[&](const monitor::NodeMonitor::ConstPtr& n){ return n->name() == req.node; }
	);

	if(it == m_monitor->nodes().end())
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

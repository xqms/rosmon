// Provides a ROS interface for controlling rosmon
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "ros_interface.h"

#include <rosmon_msgs/State.h>

#include <algorithm>

namespace rosmon
{

ROSInterface::ROSInterface(monitor::Monitor* monitor, bool enableDiagnostics,
                           const std::string& diagnosticsPrefix)
 : m_monitor(monitor)
 , m_nh("~")
 , m_diagnosticsEnabled(enableDiagnostics)
{
	m_updateTimer = m_nh.createWallTimer(ros::WallDuration(1.0), boost::bind(&ROSInterface::update, this));

	m_pub_state = m_nh.advertise<rosmon_msgs::State>("state", 10, true);

	m_srv_startStop = m_nh.advertiseService("start_stop", &ROSInterface::handleStartStop, this);

	if(m_diagnosticsEnabled)
		m_diagnosticsPublisher.reset(new DiagnosticsPublisher(diagnosticsPrefix));
}

void ROSInterface::update()
{
	rosmon_msgs::State state;
	state.header.stamp = ros::Time::now();

	if(m_diagnosticsPublisher)
		m_diagnosticsPublisher->publish(m_monitor->nodes());

	for(auto& node : m_monitor->nodes())
	{
		rosmon_msgs::NodeState nstate;
		nstate.name = node->name();
		nstate.ns = node->namespaceString();

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

		nstate.user_load = static_cast<float>(node->userLoad());
		nstate.system_load = static_cast<float>(node->systemLoad());

		nstate.memory = node->memory();

		state.nodes.push_back(nstate);
	}

	m_pub_state.publish(state);
}

bool ROSInterface::handleStartStop(rosmon_msgs::StartStopRequest& req, rosmon_msgs::StartStopResponse&)
{
  auto start_stop = [&](decltype(m_monitor->nodes().begin())::value_type node) {
    switch (req.action) {
      case rosmon_msgs::StartStopRequest::START:
        node->start();
        break;
      case rosmon_msgs::StartStopRequest::STOP:
        node->stop();
        break;
      case rosmon_msgs::StartStopRequest::RESTART:
        node->restart();
        break;
    }
  };

  if (req.node.empty()) {
    for (auto& n : m_monitor->nodes()) {
      if (req.ns.empty() || n->namespaceString() == req.ns) {
        start_stop(n);
      }
    }
    return true;
  }

  auto it = std::find_if(
		m_monitor->nodes().begin(), m_monitor->nodes().end(),
		[&](const monitor::NodeMonitor::ConstPtr& n){ return (n->name() == req.node) && (n->namespaceString() == req.ns); }
	);

	if(it == m_monitor->nodes().end())
		return false;

	start_stop(*it);

	return true;
}

void ROSInterface::shutdown()
{
	m_updateTimer.stop();

	// Send empty state packet to clear the GUI
	rosmon_msgs::State state;
	state.header.stamp = ros::Time::now();
	m_pub_state.publish(state);

	// HACK: Currently there is no way to make sure that we sent a message.
	usleep(200 * 1000);
}

}

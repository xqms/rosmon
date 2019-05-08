// Publishes diagnostic_msgs/DiagnosticArray
// Authors: Adrien Barral, Max Schwarz

#ifndef ROSMON_DIAGNOSTIC_PUBLISHER_H
#define ROSMON_DIAGNOSTIC_PUBLISHER_H

#include "monitor/node_monitor.h"

#include <rosmon_msgs/State.h>
#include <ros/publisher.h>

namespace rosmon
{

class DiagnosticsPublisher
{
public:
	explicit DiagnosticsPublisher(const std::string& diagnosticsPrefix);
	void publish(const std::vector<rosmon::monitor::NodeMonitor::Ptr>& state);

protected:
	std::string m_diagnosticNamePrefix = "rosmon";
	ros::Publisher m_diagnosticsPublisher;
};

}

#endif

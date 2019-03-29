#pragma once

#include <diagnostic_msgs/DiagnosticArray.h>
#include <rosmon_msgs/State.h>
#include <ros/publisher.h>
#include "../monitor/node_monitor.h"

namespace rosmon
{

namespace diagnostics
{

class RosmonToDiagnostic
{
public:
    RosmonToDiagnostic(const std::string& diagnosticsPrefix);
    void updateDiagnostics(const std::vector<rosmon::monitor::NodeMonitor::Ptr>& state);

protected:
    std::string m_diagnosticNamePrefix = "processes";
    ros::Publisher m_diagnosticsPublisher;
};

}

}

#include "rosmon_to_diagnostic.h"
#include "../launch/bytes_parser.h"
#include <fmt/format.h>
#include <ros/node_handle.h>
#include <ros/this_node.h>

using namespace rosmon::monitor;
using namespace rosmon::diagnostics;

RosmonToDiagnostic::RosmonToDiagnostic(const std::string& diagnosticsPrefix)
    : m_diagnosticNamePrefix(diagnosticsPrefix)
{
    ros::NodeHandle nh;
    if(diagnosticsPrefix.empty())
    {
        m_diagnosticNamePrefix = ros::this_node::getName() + ":";
    }
    m_diagnosticsPublisher =
        nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1, true);
}

void RosmonToDiagnostic::updateDiagnostics(const std::vector<NodeMonitor::Ptr>& state)
{
    diagnostic_msgs::DiagnosticArray currentDiagnosticArray;

    // cleanup the diagnostic array result :
    currentDiagnosticArray.header.stamp = ros::Time::now();
    currentDiagnosticArray.status.reserve(state.size());
    // convert state to diag :
    for(const auto& nodeState : state)
    {
        diagnostic_msgs::DiagnosticStatus nodeStatus;
        nodeStatus.name = m_diagnosticNamePrefix + nodeState->name();
        diagnostic_msgs::KeyValue kv;
        kv.key = "CPU Load";
        kv.value = fmt::format("{:.1f}%",
                               (nodeState->userLoad() + nodeState->systemLoad()) * 100.);
        nodeStatus.values.push_back(kv);

        kv.key = "used memory";
        kv.value = memoryToString(nodeState->memory());
        nodeStatus.values.push_back(kv);

        // Apply the operation level rule :
        // If process is CRASHED => ERROR
        // If process has been automatically restarted => WARN
        // If process memory limit or cpu limit is too high => WARN
        std::string msg;
        nodeStatus.level = diagnostic_msgs::DiagnosticStatus::OK;
        if(nodeState->state() == NodeMonitor::STATE_CRASHED)
        {
            nodeStatus.level = diagnostic_msgs::DiagnosticStatus::ERROR;
            nodeStatus.message = "Process has crashed";
        }
        else
        {
            if(nodeState->restartCount() > 0)
            {
                nodeStatus.level = diagnostic_msgs::DiagnosticStatus::WARN;
                msg = "restart count > 0!";
            }
            if(nodeState->memory() > nodeState->memoryLimit())
            {
                nodeStatus.level = diagnostic_msgs::DiagnosticStatus::WARN;
                msg += "memory usage is high! ";
            }
            if(nodeState->userLoad() + nodeState->systemLoad() > nodeState->cpuLimit())
            {
                nodeStatus.level = diagnostic_msgs::DiagnosticStatus::WARN;
                msg += "CPU load is high! ";
            }
            nodeStatus.message = msg;
        }
        currentDiagnosticArray.status.push_back(nodeStatus);
    }
    m_diagnosticsPublisher.publish(currentDiagnosticArray);
}

std::string RosmonToDiagnostic::memoryToString(uint64_t memory)
{
    if(memory < 1000)
    {
        return fmt::format("{} B", memory);
    }
    else if(memory < 1e6)
    {
        return fmt::format("{:.2f} KiB", memory / 1e3);
    }
    else if(memory < 1e9)
    {
        return fmt::format("{:.2f} MiB", memory / 1e6);
    }
    else if(memory < 1e12)
    {
        return fmt::format("{:.2f} GiB", memory / 1e9);
    }
    return fmt::format("{:.2f} TiB", memory / 1e12);
}

#include "rosmon_to_diagnostic.h"
#include <fmt/format.h>
#include <ros/node_handle.h>

using namespace rosmon_diagnostics;

RosmonToDiagnostic::RosmonToDiagnostic()
{
    ros::NodeHandle nh("~");
    diagnosticNamePrefix = nh.param("diagnostics_prefix", std::string("processes_"));
}

void RosmonToDiagnostic::onNewStateMessage(const rosmon_msgs::State &state)
{
    // cleanup the diagnostic array result :
    currentDiagnosticArray.header.stamp = state.header.stamp;
    currentDiagnosticArray.status.clear();

    // convert state to diag :
    for(auto& nodeState : state.nodes)
    {
        diagnostic_msgs::DiagnosticStatus nodeStatus;
        nodeStatus.name = diagnosticNamePrefix + nodeState.name;
        diagnostic_msgs::KeyValue kv;
        kv.key = "user CPU Load";
        kv.value = fmt::format("{:.1f}%", nodeState.user_load*100.);
        nodeStatus.values.push_back(kv);

        kv.key = "used memory";
        kv.value = RosmonToDiagnostic::memoryToString(nodeState.memory);
        nodeStatus.values.push_back(kv);

        // Apply the operation level rule :
        // If process is CRASHED => ERROR
        // If process has been automatically restarted => WARN
        // If process memory limit or cpu limit is too high => WARN
        std::string msg;
        nodeStatus.level = diagnostic_msgs::DiagnosticStatus::OK;
        if(nodeState.state == rosmon_msgs::NodeState::CRASHED)
        {
            nodeStatus.level = diagnostic_msgs::DiagnosticStatus::ERROR;
            nodeStatus.message = "Process has crashed";
        }else{
            if(nodeState.restart_count > 0)
            {
                nodeStatus.level = diagnostic_msgs::DiagnosticStatus::WARN;
                msg = "restart count > 0 ! ";
            }
            if(nodeState.memory > resourceChecker.getMaxAllowedMemory(nodeState.name))
            {
                nodeStatus.level = diagnostic_msgs::DiagnosticStatus::WARN;
                msg += "memory usage is high ! ";
            }
            if(nodeState.user_load > resourceChecker.getMaxUserAllowedCPU(nodeState.name))
            {
                nodeStatus.level = diagnostic_msgs::DiagnosticStatus::WARN;
                msg += "CPU User load is high ! ";
            }
            nodeStatus.message = msg;
        }
        currentDiagnosticArray.status.push_back(nodeStatus);
    }
}

std::string RosmonToDiagnostic::memoryToString(uint64_t memory)
{
    if(memory < 1000)
    {
        return fmt::format("{} Bytes", memory);
    }
    else if(memory < 1e6)
    {
        return fmt::format("{:.2f} KB", memory / 1e3);
    }
    else if(memory < 1e9)
    {
        return fmt::format("{:.2f} MB", memory / 1e6);
    }
    else if(memory < 1e12)
    {
        return fmt::format("{:.2f} GB", memory / 1e9);
    }
    return fmt::format("{:.2f} TB", memory / 1e12);
}

// Publishes diagnostic_msgs/DiagnosticArray
// Authors: Adrien Barral, Max Schwarz

#include "diagnostics_publisher.h"

#include <diagnostic_msgs/DiagnosticArray.h>

#include <fmt/format.h>

#include <ros/node_handle.h>
#include <ros/this_node.h>

using namespace rosmon::monitor;

namespace
{

std::string memoryToString(uint64_t memory)
{
	if(memory < static_cast<uint64_t>(1<<10))
		return fmt::format("{} B", memory);
	else if(memory < static_cast<uint64_t>(1<<20))
		return fmt::format("{:.2f} KiB", static_cast<double>(memory) / static_cast<uint64_t>(1<<10));
	else if(memory < static_cast<uint64_t>(1<<30))
		return fmt::format("{:.2f} MiB", static_cast<double>(memory) / static_cast<uint64_t>(1<<20));
	else if(memory < static_cast<uint64_t>(1ull<<40))
		return fmt::format("{:.2f} GiB", static_cast<double>(memory) / static_cast<uint64_t>(1ull<<30));
	else
		return fmt::format("{:.2f} TiB", static_cast<double>(memory) / static_cast<uint64_t>(1ull<<40));
}

}

namespace rosmon
{

DiagnosticsPublisher::DiagnosticsPublisher(const std::string& diagnosticsPrefix)
 : m_diagnosticNamePrefix(diagnosticsPrefix)
{
	ros::NodeHandle nh;
	if(diagnosticsPrefix.empty())
		m_diagnosticNamePrefix = ros::this_node::getName() + ":";

	m_diagnosticsPublisher =
		nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1, true);
}

void DiagnosticsPublisher::publish(const std::vector<NodeMonitor::Ptr>& state)
{
	diagnostic_msgs::DiagnosticArray currentDiagnosticArray;

	// cleanup the diagnostic array result
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

		kv.key = "restart count";
		kv.value = std::to_string(nodeState->restartCount());
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
				msg = "restart count > 0! (" + std::to_string(nodeState->restartCount()) + ")";
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

}

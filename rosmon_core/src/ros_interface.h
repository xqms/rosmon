// Provides a ROS interface for controlling rosmon
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include "monitor/monitor.h"

#include <ros/node_handle.h>

#include <rosmon_msgs/StartStop.h>
#include "diagnostics/rosmon_to_diagnostic.h"

namespace rosmon
{

class ROSInterface
{
public:
    ROSInterface(monitor::Monitor* monitor, bool enableDiagnostics=false,
                 const std::string &diagnosticsPrefix=std::string());

	void shutdown();
private:
	void update();
	bool handleStartStop(rosmon_msgs::StartStopRequest& req, rosmon_msgs::StartStopResponse& resp);

	monitor::Monitor* m_monitor;

	ros::NodeHandle m_nh;

	ros::WallTimer m_updateTimer;

	ros::Publisher m_pub_state;

	ros::ServiceServer m_srv_startStop;

    bool m_diagnosticsEnabled;
    std::shared_ptr<diagnostics::RosmonToDiagnostic> m_diagnosticsPublisher;
};

}

#endif


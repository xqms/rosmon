// Provides a ROS interface for controlling rosmon
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include "monitor/monitor.h"

#include <ros/node_handle.h>

#include <rosmon/StartStop.h>

namespace rosmon
{

struct LaunchInfo{
	std::string robot_name;
	std::string launch_group;
	std::string launch_config;
};

class ROSInterface
{
public:
	ROSInterface(monitor::Monitor* monitor, LaunchInfo* launchInfo);

	void shutdown();
private:
	void update();
	bool handleStartStop(StartStopRequest& req, StartStopResponse& resp);

	monitor::Monitor* m_monitor;

	LaunchInfo* m_launchInfo;

	ros::NodeHandle m_nh;

	ros::WallTimer m_updateTimer;

	ros::Publisher m_pub_state;

	ros::ServiceServer m_srv_startStop;
};

}

#endif


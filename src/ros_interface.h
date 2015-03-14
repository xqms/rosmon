// Provides a ROS interface for controlling rosmon
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include "launch_config.h"

#include <ros/node_handle.h>

namespace rosmon
{

class ROSInterface
{
public:
	ROSInterface(LaunchConfig* config);
	~ROSInterface();
private:
	void update();

	LaunchConfig* m_config;

	ros::NodeHandle m_nh;

	ros::WallTimer m_updateTimer;

	ros::Publisher m_pub_state;
};

}

#endif


// console user interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UI_H
#define UI_H

#include "launch_config.h"

#include <ros/wall_timer.h>

namespace rosmon
{

class UI
{
public:
	explicit UI(LaunchConfig* config);
	~UI();

	void update();
	void log(const std::string& channel, const std::string& str);
private:
	void drawStatusLine();
	void checkWindowSize();

	LaunchConfig* m_config;

	int m_columns;
	ros::WallTimer m_sizeTimer;
};

}

#endif


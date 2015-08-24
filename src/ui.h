// console user interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UI_H
#define UI_H

#include "launch_config.h"
#include "fd_watcher.h"

#include <ros/wall_timer.h>

#include <map>

namespace rosmon
{

class UI
{
public:
	explicit UI(LaunchConfig* config, const FDWatcher::Ptr& fdWatcher);
	~UI();

	void update();
	void log(const std::string& channel, const std::string& str);
private:
	void drawStatusLine();
	void checkWindowSize();
	void setupColors();
	void handleInput();

	LaunchConfig* m_config;
	FDWatcher::Ptr m_fdWatcher;

	int m_columns;
	ros::WallTimer m_sizeTimer;

	std::map<std::string, unsigned int> m_nodeColorMap;

	int m_selectedNode;
};

}

#endif


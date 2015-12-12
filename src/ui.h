// console user interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UI_H
#define UI_H

#include "monitor/monitor.h"
#include "fd_watcher.h"
#include "terminal.h"

#include <ros/wall_timer.h>

#include <map>

namespace rosmon
{

class UI
{
public:
	explicit UI(monitor::Monitor* monitor, const FDWatcher::Ptr& fdWatcher);
	~UI();

	void update();
	void log(const std::string& channel, const std::string& str);
private:
	struct ChannelInfo
	{
		ChannelInfo()
		{}

		ChannelInfo(uint32_t color)
		 : labelColor(color)
		{}

		uint32_t labelColor;
		Terminal::Parser parser;
	};

	void drawStatusLine();
	void checkWindowSize();
	void setupColors();
	void handleInput();

	monitor::Monitor* m_monitor;
	FDWatcher::Ptr m_fdWatcher;

	Terminal m_term;

	int m_columns;
	ros::WallTimer m_sizeTimer;

	std::map<std::string, ChannelInfo> m_nodeColorMap;

	int m_selectedNode;

	std::string m_strSetColor;
};

}

#endif


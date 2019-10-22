// console user interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UI_H
#define UI_H

#include "monitor/monitor.h"
#include "fd_watcher.h"
#include "terminal.h"
#include "log_event.h"

#include <ros/wall_timer.h>

#include <map>
#include <unordered_set>

namespace rosmon
{

class UI
{
public:
	explicit UI(monitor::Monitor* monitor, const FDWatcher::Ptr& fdWatcher);
	~UI();

	void update();
	void log(const LogEvent& event);
private:
	struct ChannelInfo
	{
		ChannelInfo()
		{}

		ChannelInfo(Terminal* term)
		 : parser{term}
		{}

		ChannelInfo(Terminal* term, uint32_t color)
		 : parser{term}
		 , labelColor(color)
		{}

		Terminal::Parser parser;
		uint32_t labelColor;
	};

	void drawStatusLine();
	void checkWindowSize();
	void setupColors();

	void readInput();
	void checkTerminal();

	void handleKey(int key);

	inline bool anyMuted()
	{ return !m_mutedSet.empty(); }

	inline bool isMuted(const std::string &s) 
	{ return m_mutedSet.find(s) != m_mutedSet.end(); }

	inline void mute(const std::string &s)
	{ m_mutedSet.insert(s); }

	inline void unmute(const std::string &s)
	{ m_mutedSet.erase(s); }

	inline void muteAll()
	{ for(auto& node : m_monitor->nodes()) m_mutedSet.insert(node->name()); }

	inline void unmuteAll()
	{ m_mutedSet.clear(); }

	monitor::Monitor* m_monitor;
	FDWatcher::Ptr m_fdWatcher;

	Terminal m_term;

	int m_columns;
	ros::WallTimer m_sizeTimer;
	ros::WallTimer m_terminalCheckTimer;

	std::unordered_set<std::string> m_mutedSet;

	std::map<std::string, ChannelInfo> m_nodeColorMap;

	int m_selectedNode;

	std::string m_strSetColor;

	bool m_searchActive = false;
	std::string m_searchString;
	unsigned int m_searchSelectedIndex;
	std::vector<unsigned int> m_searchNodes;
	unsigned int m_searchDisplayColumns = 0;

	Terminal::Color m_color_bar;

	Terminal::Style m_style_barLine;
	Terminal::Style m_style_bar;
	Terminal::Style m_style_barKey;
	Terminal::Style m_style_barHighlight;

	Terminal::Style m_style_nodeKey;
	Terminal::Style m_style_nodeKeyMuted;

	Terminal::Style m_style_nodeIdle;
	Terminal::Style m_style_nodeRunning;
	Terminal::Style m_style_nodeCrashed;
	Terminal::Style m_style_nodeWaiting;

	Terminal::Style m_style_nodeIdleFaded;
	Terminal::Style m_style_nodeRunningFaded;
	Terminal::Style m_style_nodeCrashedFaded;
	Terminal::Style m_style_nodeWaitingFaded;

	unsigned int m_nodeLabelWidth = 20;
};

}

#endif


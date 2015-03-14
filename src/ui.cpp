// console user interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "ui.h"

#include <cstdlib>
#include <ros/node_handle.h>

#include <sys/ioctl.h>
#include <sys/types.h>

void cleanup()
{
	printf("\n\n\n");
}

namespace rosmon
{

UI::UI(LaunchConfig* config)
 : m_config(config)
 , m_columns(80)
{
	std::atexit(cleanup);
	m_config->logMessageSignal.connect(boost::bind(&UI::log, this, _1, _2));

	m_sizeTimer = ros::NodeHandle().createWallTimer(ros::WallDuration(2.0), boost::bind(&UI::checkWindowSize, this));
	m_sizeTimer.start();

	checkWindowSize();
}

UI::~UI()
{
}

void UI::drawStatusLine()
{
	const int NODE_WIDTH = 13;

	for(auto node : m_config->nodes())
	{
		char label[NODE_WIDTH+2];
		int padding = std::max<int>(0, (NODE_WIDTH - node->name().length())/2);

		for(int i = 0; i < padding; ++i)
			label[i] = ' ';
		int c = snprintf(label+padding, NODE_WIDTH+1-padding, "%s", node->name().c_str());
		for(int i = padding + c; i < NODE_WIDTH; ++i)
			label[i] = ' ';
		label[NODE_WIDTH] = 0;

		if(node->running())
			printf("\033[42;30m");
		else
			printf("\033[41;30m");
		printf(" %s ", label);
		printf("\033[0m");
		printf(" ");
	}
	printf("\n");
	printf("RUNNING");
}

void UI::log(const std::string& channel, const std::string& log)
{
	printf("\033[K%20s: ", channel.c_str());

	unsigned int len = log.length();
	while(len != 0 && (log[len-1] == '\n' || log[len-1] == '\r'))
		len--;

	fwrite(log.c_str(), 1, len, stdout);
	printf("\033[K\n\033[K");
	fflush(stdout);
}

void UI::update()
{
	// We currently are at the beginning of the status line.
	printf("\n");
	drawStatusLine();

	// Move back
	printf("\033[2A\r");
	fflush(stdout);
}

void UI::checkWindowSize()
{
	struct winsize w;
	if(ioctl(0, TIOCGWINSZ, &w) == 0)
	{
		m_columns = w.ws_col;
	}
}

}

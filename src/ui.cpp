// console user interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "ui.h"
#include "husl/husl.h"

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
	setupColors();
}

UI::~UI()
{
}

void UI::setupColors()
{
	// Sample colors from the HUSL space
	int n = m_config->nodes().size();

	for(int i = 0; i < n; ++i)
	{
		float hue = i * 360 / n;
		float sat = 100;
		float lum = 20;

		float r, g, b;
		HUSLtoRGB(&r, &g, &b, hue, sat, lum);

		r *= 255.0;
		g *= 255.0;
		b *= 255.0;

		unsigned int color =
			std::min(255, std::max<int>(0, r))
			| (std::min(255, std::max<int>(0, g)) << 8)
			| (std::min(255, std::max<int>(0, b)) << 16);

		m_nodeColorMap[m_config->nodes()[i]->name()] = color;
	}
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

static void replaceAll(std::string& subject, const std::string& search,
                          const std::string& replace)
{
	size_t pos = 0;
	while ((pos = subject.find(search, pos)) != std::string::npos)
	{
		subject.replace(pos, search.length(), replace);
		pos += replace.length();
	}
}

// static void replaceAll(std::string& str, const std::string& from, const std::string& to) {
// 	if(from.empty())
// 		return;
// 	std::string wsRet;
// 	wsRet.reserve(str.length());
// 	size_t start_pos = 0, pos;
// 	while((pos = str.find(from, start_pos)) != std::string::npos) {
// 		wsRet += str.substr(start_pos, pos - start_pos);
// 		wsRet += to;
// 		pos += from.length();
// 		start_pos = pos;
// 	}
// 	wsRet += str.substr(start_pos);
// 	str.swap(wsRet); // faster than str = wsRet;
// }


void UI::log(const std::string& channel, const std::string& log)
{
	std::string clean = log;

	auto it = m_nodeColorMap.find(channel);
	if(it != m_nodeColorMap.end())
	{
		char buf[256];

		unsigned int color = it->second;
		snprintf(buf, sizeof(buf), "\033[48;2;%d;%d;%dm", color & 0xFF, (color >> 8) & 0xFF, (color >> 16) & 0xFF);
		fputs(buf, stdout);

// 		replaceAll(clean, "\033[0m", "\033[0m" + std::string(buf));
	}

	printf("\033[K%20s:\033[0m ", channel.c_str());

	unsigned int len = clean.length();
	while(len != 0 && (clean[len-1] == '\n' || clean[len-1] == '\r'))
		len--;

	fwrite(clean.c_str(), 1, len, stdout);
	printf("\033[K\n\033[0m\033[K");
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

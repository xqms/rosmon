// console user interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "ui.h"
#include "husl/husl.h"

#include <cstdlib>
#include <ros/node_handle.h>

#include <sys/ioctl.h>
#include <sys/types.h>

#include <termios.h>

static unsigned int g_statusLines = 2;

void cleanup()
{
	for(unsigned int i = 0; i < g_statusLines+1; ++i)
		printf("\n");

	// Switch cursor back on
	printf("\033[?25h");

	// Switch character echo on
	termios ios;
	if(tcgetattr(STDIN_FILENO, &ios) == 0)
	{
		ios.c_lflag |= ECHO;
		ios.c_lflag |= ICANON;
		tcsetattr(STDIN_FILENO, TCSANOW, &ios);
	}
}

namespace rosmon
{

UI::UI(LaunchConfig* config, const FDWatcher::Ptr& fdWatcher)
 : m_config(config)
 , m_fdWatcher(fdWatcher)
 , m_columns(80)
 , m_selectedNode(-1)
{
	std::atexit(cleanup);
	m_config->logMessageSignal.connect(boost::bind(&UI::log, this, _1, _2));

	m_sizeTimer = ros::NodeHandle().createWallTimer(ros::WallDuration(2.0), boost::bind(&UI::checkWindowSize, this));
	m_sizeTimer.start();

	checkWindowSize();
	setupColors();

	// Switch cursor off
	printf("\033[?25l");

	// Switch character echo off
	termios ios;
	if(tcgetattr(STDIN_FILENO, &ios) == 0)
	{
		ios.c_lflag &= ~ECHO;
		ios.c_lflag &= ~ICANON;
		tcsetattr(STDIN_FILENO, TCSANOW, &ios);
	}

	fdWatcher->registerFD(STDIN_FILENO, boost::bind(&UI::handleInput, this));
}

UI::~UI()
{
	m_fdWatcher->removeFD(STDIN_FILENO);
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

	unsigned int lines = 2;

	if(m_selectedNode != -1)
	{
		auto& node = m_config->nodes()[m_selectedNode];

		printf("Actions: s: start, k: stop");
		if(node->coredumpAvailable())
			printf(", d: debug");
	}
	printf("\n");

	int col = 0;

	char key = 'a';
	int i = 0;

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

		// Print key with grey background
		printf("\033[48;2;200;200;200;30m%c", key);

		switch(node->state())
		{
			case Node::STATE_RUNNING:
				printf("\033[42;30m");
				break;
			case Node::STATE_IDLE:
				printf("\033[0m");
				break;
			case Node::STATE_CRASHED:
				printf("\033[41;30m");
				break;
			case Node::STATE_WAITING:
				printf("\033[43;30m");
				break;
		}

		if(i == m_selectedNode)
			printf("[%s]", label);
		else
			printf(" %s ", label);
		printf("\033[0m");

		// Primitive wrapping control
		const int BLOCK_WIDTH = NODE_WIDTH + 3;
		col += BLOCK_WIDTH;

		if(col + 1 + BLOCK_WIDTH < m_columns)
		{
			printf(" ");
			col += 1;
		}
		else if(col == m_columns)
		{
			col = 0;
			lines++;
		}
		else if(col + 1 + BLOCK_WIDTH > m_columns)
		{
			col = 0;
			lines++;
			printf("\n\033[K");
		}

		if(key == 'z')
			key = 'A';
		else if(key == 'Z')
			key = '0';
		else if(key == '9')
			key = ' ';
		else if(key != ' ')
			++key;

		++i;
	}

	for(unsigned int i = lines; i < g_statusLines; ++i)
		printf("\n\033[K");

	g_statusLines = std::max(lines, g_statusLines);
}

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
	printf("\n\033[K");
	drawStatusLine();

	// Move back
	printf("\033[K\033[%uA\r", g_statusLines);
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

void UI::handleInput()
{
	char c;
	if(read(STDIN_FILENO, &c, 1) != 1)
		return;

	if(m_selectedNode == -1)
	{
		int nodeIndex = -1;

		if(c >= 'a' && c <= 'z')
			nodeIndex = c - 'a';
		else if(c >= 'A' && c <= 'Z')
			nodeIndex = 26 + c - 'A';
		else if(c >= '0' && c <= '9')
			nodeIndex = 26 + 26 + c - '0';

		if(nodeIndex < 0 || (size_t)nodeIndex > m_config->nodes().size())
			return;

		m_selectedNode = nodeIndex;
	}
	else
	{
		auto& node = m_config->nodes()[m_selectedNode];

		switch(c)
		{
			case 's':
				node->start();
				break;
			case 'k':
				node->stop();
				break;
			case 'd':
				node->launchDebugger();
				break;
		}

		m_selectedNode = -1;
	}
}

}

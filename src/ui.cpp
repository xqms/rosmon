// console user interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "ui.h"
#include "husl/husl.h"

#include <cstdlib>
#include <ros/node_handle.h>

static unsigned int g_statusLines = 2;
static std::string g_windowTitle;

void cleanup()
{
	for(unsigned int i = 0; i < g_statusLines+1; ++i)
		printf("\n");

	rosmon::Terminal term;

	// Switch cursor back on
	term.setCursorVisible();

	// Switch character echo on
	term.setEcho(true);

	// Restore window title (at least try)
	if(!g_windowTitle.empty())
		term.clearWindowTitle(g_windowTitle + "[-]");
}

namespace rosmon
{

UI::UI(monitor::Monitor* monitor, const FDWatcher::Ptr& fdWatcher)
 : m_monitor(monitor)
 , m_fdWatcher(fdWatcher)
 , m_columns(80)
 , m_selectedNode(-1)
{
	std::atexit(cleanup);
	m_monitor->logMessageSignal.connect(boost::bind(&UI::log, this, _1, _2));

	m_sizeTimer = ros::NodeHandle().createWallTimer(ros::WallDuration(2.0), boost::bind(&UI::checkWindowSize, this));
	m_sizeTimer.start();

	checkWindowSize();
	setupColors();

	// Switch cursor off
	m_term.setCursorInvisible();

	// Switch character echo off
	m_term.setEcho(false);

	// Configure window title
	std::string title = m_monitor->config()->windowTitle();
	if(!title.empty())
	{
		m_term.setWindowTitle(title);
		g_windowTitle = title;
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
	int n = m_monitor->nodes().size();

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

		m_nodeColorMap[m_monitor->nodes()[i]->name()] = ChannelInfo(color);
	}
}

void UI::drawStatusLine()
{
	const int NODE_WIDTH = 13;

	unsigned int lines = 2;

	// Print menu if a node is selected
	if(m_selectedNode != -1)
		printf("Actions: s: start, k: stop, d: debug");

	printf("\n");

	int col = 0;

	char key = 'a';
	int i = 0;

	for(auto& node : m_monitor->nodes())
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
		m_term.setSimpleForeground(Terminal::Black);

		if(m_term.has256Colors())
			m_term.setBackgroundColor(0xC8C8C8);
		else
			m_term.setSimpleBackground(Terminal::White);
		printf("%c", key);

		switch(node->state())
		{
			case monitor::NodeMonitor::STATE_RUNNING:
				m_term.setSimplePair(Terminal::Black, Terminal::Green);
				break;
			case monitor::NodeMonitor::STATE_IDLE:
				m_term.setStandardColors();
				break;
			case monitor::NodeMonitor::STATE_CRASHED:
				m_term.setSimplePair(Terminal::Black, Terminal::Red);
				break;
			case monitor::NodeMonitor::STATE_WAITING:
				m_term.setSimplePair(Terminal::Black, Terminal::Yellow);
				break;
		}

		if(i == m_selectedNode)
			printf("[%s]", label);
		else
			printf(" %s ", label);
		m_term.setStandardColors();

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

void UI::log(const std::string& channel, const std::string& str)
{
	std::string clean = str;

	auto it = m_nodeColorMap.find(channel);

	if(m_term.has256Colors())
	{
		if(it != m_nodeColorMap.end())
		{
			m_term.setBackgroundColor(it->second.labelColor);
			m_term.setSimpleForeground(Terminal::White);
		}
	}
	else
	{
		m_term.setSimplePair(Terminal::Black, Terminal::White);
	}

	printf("%20s:", channel.c_str());
	m_term.setStandardColors();
	m_term.clearToEndOfLine();
	putchar(' ');

	unsigned int len = clean.length();
	while(len != 0 && (clean[len-1] == '\n' || clean[len-1] == '\r'))
		len--;

	if(it != m_nodeColorMap.end())
	{
		it->second.parser.apply(&m_term);
		it->second.parser.parse(clean);
	}

	fwrite(clean.c_str(), 1, len, stdout);
	m_term.clearToEndOfLine();
	putchar('\n');
	m_term.setStandardColors();
	m_term.clearToEndOfLine();
	fflush(stdout);
}

void UI::update()
{
	if(!m_term.interactive())
		return;

	// We currently are at the beginning of the status line.
	putchar('\n');
	m_term.clearToEndOfLine();
	drawStatusLine();

	// Move back
	m_term.clearToEndOfLine();
	m_term.moveCursorUp(g_statusLines);
	m_term.moveCursorToStartOfLine();
	fflush(stdout);
}

void UI::checkWindowSize()
{
	int rows, columns;
	if(m_term.getSize(&columns, &rows))
		m_columns = columns;
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

		if(nodeIndex < 0 || (size_t)nodeIndex >= m_monitor->nodes().size())
			return;

		m_selectedNode = nodeIndex;
	}
	else
	{
		auto& node = m_monitor->nodes()[m_selectedNode];

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

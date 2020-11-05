// console user interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "ui.h"
#include "husl/husl.h"

#include <cstdlib>
#include <ros/node_handle.h>

#include <fmt/format.h>

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
	m_monitor->logMessageSignal.connect(boost::bind(&UI::log, this, _1));

	m_sizeTimer = ros::NodeHandle().createWallTimer(ros::WallDuration(2.0), boost::bind(&UI::checkWindowSize, this));
	m_sizeTimer.start();

	m_terminalCheckTimer = ros::NodeHandle().createWallTimer(ros::WallDuration(0.1), boost::bind(&UI::checkTerminal, this));
	m_terminalCheckTimer.start();

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

	// Setup colors & styles
	m_color_bar = m_term.color(0x404000, Terminal::Cyan);
	auto white = m_term.color(0xffffff, Terminal::White);
	auto barFg = m_term.color(0xffffff, Terminal::Black);

	m_style_barLine = Terminal::Style{m_color_bar, m_term.color(Terminal::Black)};
	m_style_bar = Terminal::Style{barFg, m_color_bar};
	m_style_barKey = Terminal::Style{barFg, m_term.color(0x606000, Terminal::Blue)};
	m_style_barHighlight = Terminal::Style{m_term.color(0x00FFFF, Terminal::Red), m_color_bar};

	m_style_nodeKey = Terminal::Style{m_term.color(Terminal::Black), m_term.color(0xC8C8C8, Terminal::White)};
	m_style_nodeKeyMuted = Terminal::Style{white, m_term.color(0x0000A5, Terminal::Red)};
	m_style_nodeIdle = Terminal::Style{white, m_term.color(Terminal::Black)};
	m_style_nodeRunning = Terminal::Style{m_term.color(Terminal::Black), m_term.color(Terminal::Green)};
	m_style_nodeCrashed = Terminal::Style{m_term.color(Terminal::Black), m_term.color(Terminal::Red)};
	m_style_nodeWaiting = Terminal::Style{m_term.color(Terminal::Black), m_term.color(Terminal::Yellow)};

	m_style_nodeIdleFaded = Terminal::Style{white, m_term.color(Terminal::Black)};
	m_style_nodeRunningFaded = Terminal::Style{m_term.color(Terminal::Black), m_term.color(0x008000, Terminal::Green)};
	m_style_nodeCrashedFaded = Terminal::Style{m_term.color(Terminal::Black), m_term.color(0x000040, Terminal::Red)};
	m_style_nodeWaitingFaded = Terminal::Style{m_term.color(Terminal::Black), m_term.color(0x004040, Terminal::Yellow)};

	fdWatcher->registerFD(STDIN_FILENO, boost::bind(&UI::readInput, this));
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

		r *= 255.0f;
		g *= 255.0f;
		b *= 255.0f;

		unsigned int color =
			std::min(255, std::max(0, static_cast<int>(r)))
			| (std::min(255, std::max(0, static_cast<int>(g))) << 8)
			| (std::min(255, std::max(0, static_cast<int>(b))) << 16);

		m_nodeColorMap[m_monitor->nodes()[i]->fullName()] = ChannelInfo{&m_term, color};
	}
}

// Could be done more elegantly in C++14 with a variadic lambda
namespace
{
	class ColumnPrinter
	{
	public:
		constexpr unsigned int column() const
		{ return m_column; }

		template<typename ... Args>
		void operator()(Args&& ... args)
		{
			std::string str = fmt::format(std::forward<Args>(args)...);
			m_column += str.size();
			fputs(str.c_str(), stdout);
		}
	private:
		unsigned int m_column = 0;
	};
}

std::string UI::nodeDisplayName(monitor::NodeMonitor& node, std::size_t maxWidth)
{
	std::string fullName;
	if(node.namespaceString().empty())
		fullName = node.name();
	else
		fullName = node.namespaceString() + "/" + node.name();

	// Strip initial / to save space
	if(!fullName.empty() && fullName[0] == '/')
		fullName = fullName.substr(1);

	return fullName.substr(0, maxWidth);
}

void UI::drawStatusLine()
{
	const int NODE_WIDTH = 13;

	unsigned int lines = 0;

	// Draw line using UTF-8 box characters
	{
		m_term.setStandardColors();
		m_term.clearToEndOfLine();
		m_color_bar.foreground();
		for(int i = 0; i < m_columns; ++i)
			fmt::print("▂");
		putchar('\n');

		lines++;
	}

	// Print menu / status line
	{
		m_term.setStandardColors();
		m_term.clearToEndOfLine();

		ColumnPrinter print;

		auto printKey = [&](const std::string& key, const std::string& label) {
			m_style_barKey.use();
			print(" {}:", key);
			m_style_bar.use();
			print(" {} ", label);
		};

		if(m_searchActive)
		{
			m_style_barHighlight.use();
			print("Searching for: {}", m_searchString);
			m_style_bar.use();
		}
		else if(m_selectedNode != -1)
		{
			m_style_barHighlight.use();
			auto& selectedNode = m_monitor->nodes()[m_selectedNode];

			std::string state;
			switch(selectedNode->state())
			{
				case monitor::NodeMonitor::STATE_RUNNING: state = "is running"; break;
				case monitor::NodeMonitor::STATE_IDLE:    state = "is idle";    break;
				case monitor::NodeMonitor::STATE_CRASHED: state = "has crashed"; break;
				case monitor::NodeMonitor::STATE_WAITING: state = "is waiting"; break;
				default: state = "<UNKNOWN>"; break;
			}

			print("Node '{}' {}. Actions: ", selectedNode->fullName(), state);
			printKey("s", "start");
			printKey("k", "stop");
			printKey("d", "debug");

			if(selectedNode->isMuted())
				printKey("u", "unmute");
			else
				printKey("m", "mute");
		}
		else
		{
			printKey("A-Z", "Node actions");
			printKey("F6", "Start all");
			printKey("F7", "Stop all");
			printKey("F8", "Toggle WARN+ only");
			printKey("F9", "Mute all");
			printKey("F10", "Unmute all");
			printKey("/", "Node search");

			if(stderrOnly())
			{
				print("      ");
				m_term.setSimpleForeground(Terminal::Black);
				m_term.setSimpleBackground(Terminal::Magenta);
				print("! WARN+ output only !");
				m_style_bar.use();
			}

			if(anyMuted())
			{
				print("      ");
				m_term.setSimpleForeground(Terminal::Black);
				m_term.setSimpleBackground(Terminal::Yellow);
				print("! Caution: Nodes muted !");
				m_style_bar.use();
			}
		}

		for(int i = print.column(); i < m_columns; ++i)
			putchar(' ');

		putchar('\n');

		lines++;
	}

	int col = 0;

	m_term.setStandardColors();
	m_term.clearToEndOfLine();
	if(m_searchActive)
	{
		const auto& nodes = m_monitor->nodes();
		unsigned int i = 0;

		// We can use the space of the [ ] and key characters
		constexpr auto SEARCH_NODE_WIDTH = NODE_WIDTH+3;

		std::size_t nodeWidth = SEARCH_NODE_WIDTH;
		for(auto& nodeIdx : m_searchNodes)
			nodeWidth = std::max(nodeWidth, nodeDisplayName(*nodes[nodeIdx]).length());

		// If it doesn't fit on one line, constrain to SEARCH_NODE_WIDTH
		if(m_searchNodes.size() * (nodeWidth+1) >= static_cast<std::size_t>(m_columns-1))
			nodeWidth = SEARCH_NODE_WIDTH;

		const int BLOCK_WIDTH = nodeWidth;
		for(auto& nodeIdx : m_searchNodes)
		{
			const auto& node = m_monitor->nodes()[nodeIdx];

			if(i == m_searchSelectedIndex)
				m_term.setSimplePair(Terminal::Black, Terminal::Cyan);
			else
				m_term.setStandardColors();

			std::string label = nodeDisplayName(*node, nodeWidth);
			fmt::print("{:^{}}", label, nodeWidth);
			m_term.setStandardColors();

			// Primitive wrapping control
			col += BLOCK_WIDTH;

			if(col + 1 + BLOCK_WIDTH <= m_columns)
			{
				printf(" ");
				col += 1;
			}
			else if(col + 1 + BLOCK_WIDTH > m_columns)
			{
				col = 0;
				lines++;
				putchar('\n');
				m_term.clearToEndOfLine();
			}

			++i;
		}

		m_searchDisplayColumns = (m_columns+1) / (BLOCK_WIDTH+1);
	}
	else
	{
		char key = 'a';
		int i = 0;

		for(auto& node : m_monitor->nodes())
		{
			if(m_selectedNode == -1)
			{
				// Print key with grey background
				if(node->isMuted())
					m_style_nodeKeyMuted.use();
				else
					m_style_nodeKey.use();

				fmt::print("{:c}", key);
			}
			else
			{
				m_term.setStandardColors();
				fmt::print(" ");
			}

			if(m_selectedNode == -1 || m_selectedNode == i)
			{
				switch(node->state())
				{
					case monitor::NodeMonitor::STATE_RUNNING:
						m_style_nodeRunning.use();
						break;
					case monitor::NodeMonitor::STATE_IDLE:
						m_style_nodeIdle.use();
						break;
					case monitor::NodeMonitor::STATE_CRASHED:
						m_style_nodeCrashed.use();
						break;
					case monitor::NodeMonitor::STATE_WAITING:
						m_style_nodeWaiting.use();
						break;
				}
			}
			else
			{
				switch(node->state())
				{
					case monitor::NodeMonitor::STATE_RUNNING:
						m_style_nodeRunningFaded.use();
						break;
					case monitor::NodeMonitor::STATE_IDLE:
						m_style_nodeIdleFaded.use();
						break;
					case monitor::NodeMonitor::STATE_CRASHED:
						m_style_nodeCrashedFaded.use();
						break;
					case monitor::NodeMonitor::STATE_WAITING:
						m_style_nodeWaitingFaded.use();
						break;
				}
			}

			std::string label = nodeDisplayName(*node, NODE_WIDTH);
			if(i == m_selectedNode)
				fmt::print("[{:^{}}]", label, NODE_WIDTH);
			else
				fmt::print(" {:^{}} ", label, NODE_WIDTH);
			m_term.setStandardColors();

			// Primitive wrapping control
			const int BLOCK_WIDTH = NODE_WIDTH + 3;
			col += BLOCK_WIDTH;

			if(col + 1 + BLOCK_WIDTH <= m_columns)
			{
				printf(" ");
				col += 1;
			}
			else if(col + 1 + BLOCK_WIDTH > m_columns)
			{
				col = 0;
				lines++;
				putchar('\n');
				m_term.clearToEndOfLine();
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
	}

	// Erase rest of the lines
	for(unsigned int i = lines; i < g_statusLines; ++i)
		printf("\n\033[K");

	g_statusLines = std::max(lines, g_statusLines);
}

void UI::log(const LogEvent& event)
{
	// Is this node muted? Muted events go into the log, but are not shown in
	// the UI.
	if(event.muted)
		return;

	// Are we supposed to show stdout?
	if(event.channel == LogEvent::Channel::Stdout && (!event.showStdout || stderrOnly()))
		return;

	const std::string& clean = event.message;

	auto it = m_nodeColorMap.find(event.source);

	// Is this a node message?
	if(it != m_nodeColorMap.end())
	{
		m_term.setLineWrap(false);

		auto actualLabelWidth = std::max<unsigned int>(m_nodeLabelWidth, event.source.size());
		auto lines = it->second.parser.wrap(clean, m_columns - actualLabelWidth - 2);

		for(unsigned int line = 0; line < lines.size(); ++line)
		{
			// Draw label
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

			if(line == 0)
				fmt::print("{:>{}}:", event.source, m_nodeLabelWidth);
			else
			{
				for(unsigned int i = 0; i < actualLabelWidth-1; ++i)
					putchar(' ');
				fmt::print("~ ");
			}
			m_term.setStandardColors();
			m_term.clearToEndOfLine();
			putchar(' ');

			fputs(lines[line].c_str(), stdout);
			putchar('\n');
		}

		m_term.setLineWrap(true);
	}
	else
	{
		fmt::print("{:>{}}:", event.source, m_nodeLabelWidth);
		m_term.setStandardColors();
		m_term.clearToEndOfLine();
		putchar(' ');

		unsigned int len = clean.length();
		while(len != 0 && (clean[len-1] == '\n' || clean[len-1] == '\r'))
			len--;

		switch(event.type)
		{
			case LogEvent::Type::Raw:
			case LogEvent::Type::Info:
				break;
			case LogEvent::Type::Warning:
				m_term.setSimpleForeground(Terminal::Yellow);
				break;
			case LogEvent::Type::Error:
				m_term.setSimpleForeground(Terminal::Red);
				break;
		}

		fwrite(clean.c_str(), 1, len, stdout);
		m_term.clearToEndOfLine();
		putchar('\n');
	}

	m_term.setStandardColors();
	m_term.clearToEndOfLine();
	fflush(stdout);

	scheduleUpdate();
}

void UI::update()
{
	if(!m_term.interactive())
		return;

	if(!m_refresh_required)
		return;

	m_refresh_required = false;

	// Disable automatic linewrap. This prevents ugliness on terminal resize.
	m_term.setLineWrap(false);

	// We currently are at the beginning of the status line.
	drawStatusLine();

	// Move back
	m_term.clearToEndOfLine();
	m_term.moveCursorUp(g_statusLines);
	m_term.moveCursorToStartOfLine();

	// Enable automatic linewrap again
	m_term.setLineWrap(true);

	fflush(stdout);
}

void UI::checkWindowSize()
{
	int rows, columns;
	if(m_term.getSize(&columns, &rows))
		m_columns = columns;

	std::size_t w = 20;
	for(const auto& node : m_monitor->nodes())
		w = std::max(w, node->fullName().size());

	m_nodeLabelWidth = std::min<unsigned int>(w, m_columns/4);
}

void UI::readInput()
{
	int c = m_term.readKey();
	if(c < 0)
		return;

	handleKey(c);
}

void UI::checkTerminal()
{
	int c = m_term.readLeftover();
	if(c < 0)
		return;

	handleKey(c);
}

void UI::handleKey(int c)
{
	// Instead of trying to figure out when exactly we need a redraw, just
	// redraw on every keystroke.
	scheduleUpdate();

	// Are we in search mode?
	if(m_searchActive)
	{
		if(c == '\n')
		{
			if(m_searchSelectedIndex >= 0 && m_searchSelectedIndex < m_searchNodes.size())
				m_selectedNode = m_searchNodes[m_searchSelectedIndex];
			else
				m_selectedNode = -1;

			m_searchActive = false;
			return;
		}

		if(c == '\E')
		{
			m_selectedNode = -1;
			m_searchActive = false;
			return;
		}

		if(c == '\t')
		{
			m_searchSelectedIndex++;
			if(m_searchSelectedIndex >= m_searchNodes.size())
				m_searchSelectedIndex = 0;

			return;
		}

		if(m_searchSelectedIndex >= 0 && m_searchSelectedIndex < m_searchNodes.size())
		{
			int col = m_searchSelectedIndex % m_searchDisplayColumns;
			int row = m_searchSelectedIndex / m_searchDisplayColumns;

			if(c == Terminal::SK_Right)
			{
				if(col < static_cast<int>(m_searchDisplayColumns)-1 && m_searchSelectedIndex < m_searchNodes.size()-1)
					m_searchSelectedIndex++;
				return;
			}

			if(c == Terminal::SK_Left)
			{
				if(col > 0)
					m_searchSelectedIndex--;
				return;
			}

			if(c == Terminal::SK_Up)
			{
				if(row > 0)
					m_searchSelectedIndex -= m_searchDisplayColumns;
				return;
			}

			if(c == Terminal::SK_Down)
			{
				int numRows = (m_searchNodes.size() + m_searchDisplayColumns - 1) / m_searchDisplayColumns;
				if(row < numRows - 1)
					m_searchSelectedIndex = std::min<int>(m_searchNodes.size(), m_searchSelectedIndex + m_searchDisplayColumns);
				return;
			}
		}

		if(c == Terminal::SK_Backspace)
		{
			if(!m_searchString.empty())
				m_searchString.pop_back();
		}
		else if(std::isgraph(c))
			m_searchString.push_back(c);

		m_searchSelectedIndex = 0;

		// Recompute matched nodes
		m_searchNodes.clear();
		const auto& nodes = m_monitor->nodes();
		for(unsigned int i = 0; i < nodes.size(); ++i)
		{
			const auto& node = nodes[i];
			auto idx = nodeDisplayName(*node).find(m_searchString);
			if(idx != std::string::npos)
				m_searchNodes.push_back(i);
		}

		return;
	}

	if(m_selectedNode == -1)
	{
		int nodeIndex = -1;

		if(c == Terminal::SK_F6)
		{
			startAll();
			return;
		}

		if(c == Terminal::SK_F7)
		{
			stopAll();
			return;
		}

		// Check for Mute all keys first
		if(c == Terminal::SK_F9)
		{
			muteAll();
			return;
		}

		if(c == Terminal::SK_F10)
		{
			unmuteAll();
			return;
		}

		// Check for Stderr Only Toggle
		if(c == Terminal::SK_F8)
		{
			toggleStderrOnly();
			return;
		}

		// Search
		if(c == '/')
		{
			m_searchString = {};
			m_searchSelectedIndex = 0;
			m_searchNodes.resize(m_monitor->nodes().size());
			std::iota(m_searchNodes.begin(), m_searchNodes.end(), 0);
			m_searchActive = true;
			return;
		}

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
			case 'm':
				node->setMuted(true);
				break;
			case 'u':
				node->setMuted(false);
				break;
		}

		m_selectedNode = -1;
	}
}

bool UI::anyMuted() const
{
	return std::any_of(m_monitor->nodes().begin(), m_monitor->nodes().end(), [](const monitor::NodeMonitor::Ptr& n){
		return n->isMuted();
	});
}

void UI::startAll()
{
	for(auto& n : m_monitor->nodes())
		n->start();
}

void UI::stopAll()
{
	for(auto& n : m_monitor->nodes())
		n->stop();
}

void UI::muteAll()
{
	for(auto& n : m_monitor->nodes())
		n->setMuted(true);
}

void UI::unmuteAll()
{
	for(auto& n : m_monitor->nodes())
		n->setMuted(false);
}

void UI::scheduleUpdate()
{
	m_refresh_required = true;
}

bool UI::stderrOnly()
{
	return m_stderr_only;
}

void UI::toggleStderrOnly()
{
	m_stderr_only = !m_stderr_only;
}

}

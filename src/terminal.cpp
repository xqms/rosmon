// Encapsulates terminal control (colors, cursor, ...)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "terminal.h"

#include <termios.h>

#include <term.h>
#include <curses.h>

#include <stdio.h>

#include <boost/regex.hpp>

#include <sys/ioctl.h>
#include <sys/types.h>

#include <boost/tokenizer.hpp>

namespace rosmon
{

Terminal::Parser::Parser()
 : m_state(STATE_ESCAPE)
 , m_fgColor(-1)
 , m_bgColor(-1)
{
}

void Terminal::Parser::parseSetAttributes(const std::string& str)
{
	using Tokenizer = boost::tokenizer<boost::char_separator<char>>;

	boost::char_separator<char> sep(";");
	Tokenizer tok(str, sep);

	for(Tokenizer::iterator it = tok.begin(); it != tok.end(); ++it)
	{
		errno = 0;
		char* endptr = const_cast<char*>(it->c_str());
		int code = strtoul(it->c_str(), &endptr, 10);

		if(errno != 0 || *endptr != 0)
		{
			// Error in specification, break out of here
			m_fgColor = -1;
			m_bgColor = -1;
			return;
		}

		if(code == 0)
		{
			m_fgColor = -1;
			m_bgColor = -1;
		}
		else if(code >= 30 && code <= 37)
			m_fgColor = code - 30;
		else if(code >= 40 && code <= 47)
			m_bgColor = code - 40;
	}
}

void Terminal::Parser::parse(char c)
{
	switch(m_state)
	{
		case STATE_ESCAPE:
			if(c == '\033')
				m_state = STATE_TYPE;
			break;
		case STATE_TYPE:
			if(c == '[')
			{
				m_state = STATE_CSI;
				m_buf.clear();
			}
			else
				m_state = STATE_ESCAPE;
			break;
		case STATE_CSI:
			if(c == 'm')
			{
				parseSetAttributes(m_buf);
				m_state = STATE_ESCAPE;
			}
			else
			{
				m_buf.push_back(c);
				if(m_buf.length() >= 16)
					m_state = STATE_ESCAPE;
			}
			break;
	}
}

void Terminal::Parser::parse(const std::string& str)
{
	for(char c : str)
		parse(c);
}

void Terminal::Parser::apply(Terminal* term)
{
	if(m_fgColor >= 0 && m_bgColor >= 0)
		term->setSimplePair((SimpleColor)m_fgColor, (SimpleColor)m_bgColor);
	else
	{
		term->setStandardColors();
		if(m_fgColor >= 0)
			term->setSimpleForeground((SimpleColor)m_fgColor);
		else if(m_bgColor >= 0)
			term->setSimpleBackground((SimpleColor)m_fgColor);
	}
}

Terminal::Terminal()
{
	int ret;
	if(setupterm(NULL, STDOUT_FILENO, &ret) != OK)
	{
		printf("Could not setup the terminal. Disabling all colors...\n");
		// FIXME: do so
	}

	// Detect 256 color terminals
	int num_colors = tigetnum("colors");
	m_256colors = num_colors >= 256;

	// Detect truecolor-capable terminals
	m_truecolor = false;

	if(getenv("KONSOLE_DBUS_SESSION"))
	{
		char buf[256];
		FILE* out = popen("konsole -v", "r");
		boost::regex exp(R"(Konsole: (?<major>\d+)\.(?<minor>\d+)\.(?<patch>\d+))");

		while(fgets(buf, sizeof(buf), out))
		{
			int n = strlen(buf);
			if(buf[n-1] == '\n')
				buf[n-1] = 0;

			boost::cmatch match;
			if(boost::regex_match(buf, match, exp))
			{
				int major = atoi(match.str("major").c_str());

				if(major >= 2)
				{
					m_truecolor = true;
				}
			}
		}

		fclose(out);
	}

	char* vte_version = getenv("VTE_VERSION");
	if(vte_version && atoi(vte_version) >= 3600)
	{
		m_truecolor = true;
	}

	{
		const char* bgColor = tigetstr("setab");
		if(bgColor && bgColor != (char*)-1)
			m_bgColorStr = bgColor;
		else
			printf("Your terminal does not support ANSI background!\n");
	}
	{
		const char* fgColor = tigetstr("setaf");
		if(fgColor && fgColor != (char*)-1)
			m_fgColorStr = fgColor;
		else
			printf("Your terminal does not support ANSI foreground!\n");
	}

	m_opStr = tigetstr("op");
	m_elStr = tigetstr("el");
	m_upStr = tigetstr("cuu");
}

Terminal::~Terminal()
{
}

bool Terminal::has256Colors() const
{
	return m_256colors;
}

void Terminal::setCursorInvisible()
{
	putp(tigetstr("civis"));
}

void Terminal::setCursorVisible()
{
	putp(tigetstr("cnorm"));
}

static int ansiColor(uint32_t rgb)
{
	int r = (rgb & 0xFF);
	int g = (rgb >> 8) & 0xFF;
	int b = (rgb >> 16) & 0xFF;

	r = r * 6 / 256;
	g = g * 6 / 256;
	b = b * 6 / 256;

	return 16 + 36 * r + 6 * g + b;
}

void Terminal::setBackgroundColor(uint32_t color)
{
	if(m_truecolor)
	{
		char buf[256];
		snprintf(buf, sizeof(buf), "\033[48;2;%d;%d;%dm", color & 0xFF, (color >> 8) & 0xFF, (color >> 16) & 0xFF);
		fputs(buf, stdout);
	}
	else
	{
		char* out = tiparm(m_bgColorStr.c_str(), ansiColor(color));
		putp(out);
	}
}

void Terminal::setForegroundColor(uint32_t color)
{
	if(m_truecolor)
	{
		char buf[256];
		snprintf(buf, sizeof(buf), "\033[38;2;%d;%d;%dm", color & 0xFF, (color >> 8) & 0xFF, (color >> 16) & 0xFF);
		fputs(buf, stdout);
	}
	else
	{
		char* out = tiparm(m_fgColorStr.c_str(), ansiColor(color));
		putp(out);
	}
}

void Terminal::setEcho(bool on)
{
	// Switch character echo on
	termios ios;
	if(tcgetattr(STDIN_FILENO, &ios) == 0)
	{
		if(on)
		{
			ios.c_lflag |= ECHO;
			ios.c_lflag |= ICANON;
		}
		else
		{
			ios.c_lflag &= ~ECHO;
			ios.c_lflag &= ~ICANON;
		}

		tcsetattr(STDIN_FILENO, TCSANOW, &ios);
	}
}

void Terminal::resetToShell()
{
	reset_shell_mode();
}

void Terminal::setSimpleForeground(SimpleColor color)
{
	char* out = tiparm(m_fgColorStr.c_str(), color);
	putp(out);
}

void Terminal::setSimpleBackground(SimpleColor color)
{
	char* out = tiparm(m_bgColorStr.c_str(), color);
	putp(out);
}

void Terminal::setSimplePair(SimpleColor fg, SimpleColor bg)
{
	setSimpleForeground(fg);
	setSimpleBackground(bg);
}

void Terminal::setStandardColors()
{
	putp(m_opStr.c_str());
}

void Terminal::clearToEndOfLine()
{
	putp(m_elStr.c_str());
}

void Terminal::moveCursorUp(int numLines)
{
	putp(tparm(m_upStr.c_str(), numLines));
}

void Terminal::moveCursorToStartOfLine()
{
	putchar('\r');
}

bool Terminal::getSize(int* outColumns, int* outRows)
{
	struct winsize w;
	if(ioctl(STDIN_FILENO, TIOCGWINSZ, &w) == 0)
	{
		*outColumns = w.ws_col;
		*outRows = w.ws_row;
		return true;
	}
	else
		return false;
}

}

// Encapsulates terminal control (colors, cursor, ...)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TERMINAL_H
#define TERMINAL_H

#include <stdint.h>
#include <string>

namespace rosmon
{

class Terminal
{
public:
	Terminal();
	~Terminal();

	void setForegroundColor(uint32_t color);
	void setBackgroundColor(uint32_t color);

	void setCursorInvisible();
	void setCursorVisible();

	void setEcho(bool on);

	void resetToShell();

	enum SimpleColor
	{
		Black,
		Red,
		Green,
		Yellow,
		Blue,
		Magenta,
		Cyan,
		White
	};

	void setSimpleBackground(SimpleColor color);
	void setSimpleForeground(SimpleColor color);
	void setSimplePair(SimpleColor fg, SimpleColor bg);

	void setStandardColors();

	void clearToEndOfLine();

	void moveCursorUp(int numLines);
	void moveCursorToStartOfLine();

	bool getSize(int* columns, int* rows);

private:
	bool m_truecolor;

	std::string m_bgColorStr;
	std::string m_fgColorStr;
	std::string m_opStr;
	std::string m_elStr;
	std::string m_upStr;
};

}

#endif

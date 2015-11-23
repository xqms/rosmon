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

	class Parser
	{
	public:
		Parser();

		void parse(char c);
		void parse(const std::string& str);

		void apply(Terminal* term);
	private:
		void parseSetAttributes(const std::string& attrs);

		enum State
		{
			STATE_ESCAPE,
			STATE_TYPE,
			STATE_CSI
		};

		State m_state;
		std::string m_buf;

		int m_fgColor;
		int m_bgColor;
	};

	Terminal();
	~Terminal();

	void setForegroundColor(uint32_t color);
	void setBackgroundColor(uint32_t color);

	void setCursorInvisible();
	void setCursorVisible();

	void setEcho(bool on);

	void resetToShell();

	void setSimpleBackground(SimpleColor color);
	void setSimpleForeground(SimpleColor color);
	void setSimplePair(SimpleColor fg, SimpleColor bg);

	void setStandardColors();

	void clearToEndOfLine();

	void moveCursorUp(int numLines);
	void moveCursorToStartOfLine();

	bool getSize(int* columns, int* rows);

	bool has256Colors() const;

private:
	bool m_256colors;
	bool m_truecolor;

	std::string m_bgColorStr;
	std::string m_fgColorStr;
	std::string m_opStr;
	std::string m_elStr;
	std::string m_upStr;
};

}

#endif

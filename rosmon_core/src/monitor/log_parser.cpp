// Splits the log stream into individual messages
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "log_parser.h"

#include <vector>

namespace rosmon
{
namespace monitor
{

class LogParser::Private
{
public:
	enum class State
	{
		ColorEscape1,
		ColorEscape2,
		ColorEscape3,

		RawMsgContent,

		TypedMsgContent,

		EndEscape1,
		EndEscape2,
		EndEscape3,
		EndEscape4
	};

	static LogEvent::Type typeFromColorCode(int code)
	{
		switch(code)
		{
			case 0: return LogEvent::Type::Info;
			case 31: return LogEvent::Type::Error;
			case 32: return LogEvent::Type::Debug;
			case 33: return LogEvent::Type::Warning;
		}

		return LogEvent::Type::Info;
	}

	void process(char c)
	{
		m_buffer.push_back(c);

		switch(m_state)
		{
			case State::ColorEscape1:
				if(c == '\e')
					m_state = State::ColorEscape2;
				else
					m_state = State::RawMsgContent;

				break;

			case State::ColorEscape2:
				if(c == '[')
				{
					m_state = State::ColorEscape3;
					m_colorCode = 0;
				}
				else
					m_state = State::RawMsgContent;

				break;

			case State::ColorEscape3:
				if(std::isdigit(c))
					m_colorCode = m_colorCode * 10 + (c - '0');
				else if(c == 'm')
				{
					m_msgBegin = m_buffer.size();
					m_state = State::TypedMsgContent;
				}
				else
					m_state = State::RawMsgContent;

				break;

			case State::TypedMsgContent:
				if(c == '\e')
				{
					m_msgEnd = m_buffer.size() - 1;
					m_state = State::EndEscape1;
				}

				break;

			case State::EndEscape1:
				if(c == '[')
					m_state = State::EndEscape2;
				else
					m_state = State::TypedMsgContent;

				break;

			case State::EndEscape2:
				if(c == '0')
					m_state = State::EndEscape3;
				else
					m_state = State::TypedMsgContent;

				break;

			case State::EndEscape3:
				if(c == 'm')
					m_state = State::EndEscape4;
				else
					m_state = State::TypedMsgContent;

				break;

			case State::EndEscape4:
				if(c == '\n')
				{
					std::string msg{m_buffer.data() + m_msgBegin, m_msgEnd - m_msgBegin};

					if(m_cb)
						m_cb(Event{msg, typeFromColorCode(m_colorCode)});

					m_state = State::ColorEscape1;
					m_buffer.clear();
				}

				break;

			case State::RawMsgContent:
				if(c == '\n')
				{
					std::string msg{m_buffer.data(), m_buffer.size()-1};

					if(m_cb)
						m_cb(Event{msg, LogEvent::Type::Raw});

					m_state = State::ColorEscape1;
					m_buffer.clear();
				}

				break;
		}
	}

	State m_state = State::ColorEscape1;

	int m_colorCode = 0;

	std::vector<char> m_buffer;

	std::size_t m_msgBegin = 0;
	std::size_t m_msgEnd = 0;

	std::function<void(LogParser::Event&&)> m_cb;
};

LogParser::LogParser()
 : m_d{new Private}
{
}

LogParser::~LogParser()
{
}

void LogParser::process(const char* input, std::size_t size)
{
	for(std::size_t i = 0; i < size; ++i)
		m_d->process(input[i]);
}

void LogParser::setCallback(const std::function<void(Event&&)>& cb)
{
	m_d->m_cb = cb;
}

}
}

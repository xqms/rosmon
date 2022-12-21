// Splits the log stream into individual messages
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSMON_MONITOR_LOG_PARSER_H
#define ROSMON_MONITOR_LOG_PARSER_H

#include <chrono>
#include <memory>
#include <functional>

#include "../log_event.h"

namespace rosmon
{
namespace monitor
{

class LogParser
{
public:
	LogParser();
	~LogParser();

	struct Event
	{
		std::string message;
		LogEvent::Type severity;
	};

	void setCallback(const std::function<void(Event&&)>& cb);

	void process(const char* input, std::size_t size, const std::chrono::steady_clock::time_point& time = std::chrono::steady_clock::now());

	inline void processString(const std::string& str, const std::chrono::steady_clock::time_point& time = std::chrono::steady_clock::now())
	{ process(str.c_str(), str.length(), time); }

	void checkPending(const std::chrono::steady_clock::time_point& time = std::chrono::steady_clock::now());

	void flush();

private:
	class Private;
	std::unique_ptr<Private> m_d;
};

}
}


#endif

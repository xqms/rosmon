// Log event with metadata
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSMON_LOG_EVENT_H
#define ROSMON_LOG_EVENT_H

#include <string>

namespace rosmon
{

struct LogEvent
{
public:
	enum class Type
	{
		/**
		 * Raw messages from monitored nodes. These are self-coded using
		 * ANSI escape codes. */
		Raw,

		Info,
		Warning,
		Error
	};

	enum class Channel
	{
		NotApplicable,
		Stdout,
		Stderr
	};

	LogEvent(std::string source, std::string message, Type type = Type::Raw)
	 : source{std::move(source)}, message{std::move(message)}, type{type}
	{}

	std::string source;
	std::string message;
	Type type;
	bool muted = false;
	Channel channel = Channel::NotApplicable;
	bool showStdout = true;
};

}

#endif

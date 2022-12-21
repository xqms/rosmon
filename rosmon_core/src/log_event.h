// Log event with metadata
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSMON_LOG_EVENT_H
#define ROSMON_LOG_EVENT_H

#include <string>
#include <sstream>

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

		Debug,
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

	std::string coloredString() const
	{
		auto colorize = [](const char* prefix, const std::string& str){
			std::stringstream ss;
			ss << prefix << str << "\e[0m";
			return ss.str();
		};

		switch(type)
		{
			case Type::Raw: return message;
			case Type::Debug: return colorize("\e[32m", message);
			case Type::Info: return colorize("\e[0m", message);
			case Type::Warning: return colorize("\e[33m", message);
			case Type::Error: return colorize("\e[31m", message);
		}

		return message;
	}

	std::string source;
	std::string message;
	Type type;
	bool muted = false;
	Channel channel = Channel::NotApplicable;
	bool showStdout = true;
};

}

#endif

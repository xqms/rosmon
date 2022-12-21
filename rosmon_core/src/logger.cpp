// Logs all output to a log file for the run
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "logger.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <stdexcept>

#include <sys/time.h>

#include <fmt/format.h>

#include <syslog.h>

namespace rosmon
{

namespace
{
	int prioFromEventType(LogEvent::Type type)
	{
		switch(type)
		{
			case LogEvent::Type::Error: return 3;
			case LogEvent::Type::Warning: return 4;
			case LogEvent::Type::Info: return 6;
			case LogEvent::Type::Raw: return 6;
			case LogEvent::Type::Debug: return 7;
		}

		return 6;
	}
}

FileLogger::FileLogger(const std::string& path, bool flush)
 : m_flush{flush}
{
	m_flush = flush;
	m_file = fopen(path.c_str(), "ae");
	if(!m_file)
	{
		throw std::runtime_error(fmt::format(
			"Could not open log file: {}", strerror(errno)
		));
	}
}

FileLogger::~FileLogger()
{
	if(m_file)
		fclose(m_file);
}

void FileLogger::log(const LogEvent& event)
{
	struct timeval tv;
	memset(&tv, 0, sizeof(tv));
	gettimeofday(&tv, nullptr);

	struct tm btime;
	memset(&btime, 0, sizeof(tv));
	localtime_r(&tv.tv_sec, &btime);

	char timeString[100];
	strftime(timeString, sizeof(timeString), "%a %F %T", &btime);

	unsigned int len = event.message.length();
	while(len != 0 && (event.message[len-1] == '\n' || event.message[len-1] == '\r'))
		len--;

	fmt::print(m_file, "{}.{:03d}: {:>20}: ",
		timeString, tv.tv_usec / 1000,
		event.source.c_str()
	);
	fwrite(event.message.c_str(), 1, len, m_file);
	fputc('\n', m_file);

	if(m_flush)
		fflush(m_file);
}


SyslogLogger::SyslogLogger(const std::string& launchFileName)
{
	m_tag = fmt::format("rosmon@{}", launchFileName);
	openlog(m_tag.c_str(), 0, LOG_USER);
}

void SyslogLogger::log(const LogEvent& event)
{
	syslog(prioFromEventType(event.type), "%20s: %s", event.source.c_str(), event.message.c_str());
}

}

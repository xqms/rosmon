// Logs all output to a log file for the run
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "logger.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <sstream>
#include <stdexcept>

#include <sys/time.h>

namespace rosmon
{

Logger::Logger(const std::string& path)
 : m_file(nullptr)
{
	m_file = fopen(path.c_str(), "we");
	if(!m_file)
	{
		std::stringstream ss;
		ss << "Could not open log file: " << strerror(errno);
		throw std::runtime_error(ss.str());
	}
}

Logger::~Logger()
{
	if(m_file)
		fclose(m_file);
}

void Logger::log(const std::string& source, const std::string& msg)
{
	struct timeval tv;
	memset(&tv, 0, sizeof(tv));
	gettimeofday(&tv, nullptr);

	struct tm btime;
	memset(&btime, 0, sizeof(tv));
	localtime_r(&tv.tv_sec, &btime);

	char timeString[100];
	strftime(timeString, sizeof(timeString), "%a %F %T", &btime);

	unsigned int len = msg.length();
	while(len != 0 && (msg[len-1] == '\n' || msg[len-1] == '\r'))
		len--;

	fprintf(m_file, "%s.%03ld: %20s: ",
		timeString, tv.tv_usec / 1000,
		source.c_str()
	);
	fwrite(msg.c_str(), 1, len, m_file);
	fputc('\n', m_file);
}

}

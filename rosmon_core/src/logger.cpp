// Logs all output to a log file for the run
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "logger.h"
#include "fmt/format.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <stdexcept>
#include <iterator>

#include <sys/time.h>

#include <fmt/format.h>

#include <syslog.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

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


SystemdLogger::SystemdLogger(const std::string& launchFileName)
 : m_launchFileName{launchFileName}
{
	m_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
	if(m_fd < 0)
		throw std::runtime_error{fmt::format("Could not create socket: {}", strerror(errno))};

	sockaddr_un addr{};
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, "/run/systemd/journal/socket", sizeof(addr.sun_path)-1);

	if(connect(m_fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0)
		throw NotAvailable{fmt::format("Systemd Journal not available: {}", strerror(errno))};
}

SystemdLogger::~SystemdLogger()
{
	if(m_fd >= 0)
		close(m_fd);
}

void SystemdLogger::log(const LogEvent& event)
{
	fmt::memory_buffer buffer;

	fmt::format_to(std::back_inserter(buffer),
		"PRIORITY={}\n"
		"ROSMON_LAUNCH_FILE={}\n"
		"ROSMON_NODE={}\n"
		"SYSLOG_IDENTIFIER=rosmon@{}\n",
		prioFromEventType(event.type),
		m_launchFileName,
		event.source,
		m_launchFileName
	);

	std::string msg = fmt::format("{}: {}", event.source, event.message);

	if(msg.find('\n') == std::string::npos)
		fmt::format_to(std::back_inserter(buffer), "MESSAGE={}\n", msg);
	else
	{
		buffer.append(std::string{"MESSAGE\n"});
		std::array<uint8_t, 8> size;
		uint64_t sizeInt = msg.length();
		for(int i = 0; i < 8; ++i)
		{
			size[i] = sizeInt & 0xFF;
			sizeInt >>= 8;
		}

		buffer.append(size);
		buffer.append(msg);
		buffer.append(std::string{"\n"});
	}

	if(write(m_fd, buffer.data(), buffer.size()) < 0)
		fprintf(stderr, "Could not write to systemd log: %s\n", strerror(errno));
}

}

// Watches a set of file descriptors for changes
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "fd_watcher.h"

#include <cstdarg>
#include <vector>

#include <sys/types.h>
#include <sys/select.h>


static std::runtime_error error(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	char str[1024];

	vsnprintf(str, sizeof(str), fmt, args);

	va_end(args);

	return std::runtime_error(str);
}

namespace rosmon
{

FDWatcher::FDWatcher()
{
}

void FDWatcher::registerFD(int fd, const boost::function<void (int)>& cb)
{
	m_fds[fd] = cb;
}

void FDWatcher::removeFD(int fd)
{
	m_fds.erase(fd);
}

void FDWatcher::wait(const ros::WallDuration& duration)
{
	timeval timeout;
	timeout.tv_sec = duration.toNSec() / 1000LL / 1000LL / 1000LL;
	timeout.tv_usec = (duration.toNSec() / 1000LL) % (1000LL * 1000LL * 1000LL);

	fd_set fds;
	FD_ZERO(&fds);

	int maxfd = 0;
	for(auto pair : m_fds)
	{
		FD_SET(pair.first, &fds);
		maxfd = std::max(pair.first, maxfd);
	}

	int ret = select(maxfd+1, &fds, nullptr, nullptr, &timeout);
	if(ret < 0)
	{
		if(errno == EINTR || errno == EAGAIN)
			return;

		throw error("Could not select(): %s", strerror(errno));
	}

	if(ret != 0)
	{
		// Store the callbacks to be notified in a temporary list, as calling
		// the callback might call removeFD(), which will confuse us...
		std::vector<std::pair<int, boost::function<void(int)>>> toBeNotified;

		for(auto pair : m_fds)
		{
			if(FD_ISSET(pair.first, &fds))
				toBeNotified.emplace_back(pair);
		}

		// Actually call the callbacks
		for(auto pair : toBeNotified)
			pair.second(pair.first);
	}
}

}

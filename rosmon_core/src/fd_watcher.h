// Watches a set of file descriptors for changes
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef FD_WATCHER_H
#define FD_WATCHER_H

#include <ros/time.h>

#include <map>

#include <boost/function.hpp>

namespace rosmon
{

class FDWatcher
{
public:
	typedef boost::shared_ptr<FDWatcher> Ptr;

	FDWatcher();

	void registerFD(int fd, const boost::function<void(int)>& cb);
	void removeFD(int fd);

	void wait(const ros::WallDuration& duration);
private:
	std::map<int, boost::function<void(int)>> m_fds;
};

}

#endif

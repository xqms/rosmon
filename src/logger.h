// Logs node all output to a log file for the run
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LOGGER_H
#define LOGGER_H

#include <string>

namespace rosmon
{

class Logger
{
public:
	explicit Logger(const std::string& path);
	~Logger();

	void log(const std::string& source, const std::string& msg);
private:
	FILE* m_file;
};

}

#endif

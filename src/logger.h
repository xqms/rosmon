// Logs all output to a log file for the run
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LOGGER_H
#define LOGGER_H

#include <string>

namespace rosmon
{

/**
 * @brief Write log messages into a log file
 **/
class Logger
{
public:
	/**
	 * @brief Constructor
	 *
	 * @param path Path to the output file
	 **/
	explicit Logger(const std::string& path);
	~Logger();

	//! Log message
	void log(const std::string& source, const std::string& msg);
private:
	FILE* m_file;
};

}

#endif

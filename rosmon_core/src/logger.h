// Logs all output to a log file for the run
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LOGGER_H
#define LOGGER_H

#include <string>

#include "log_event.h"

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
	explicit Logger(const std::string& path, bool flush = false);
	~Logger();

	//! Log message
	void log(const LogEvent& event);
private:
	FILE* m_file = nullptr;
	bool m_flush = false;
};

}

#endif

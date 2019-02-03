// Provides process information on linux systems
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LINUX_PROCESS_INFO_H
#define LINUX_PROCESS_INFO_H

#include <cstddef>

namespace rosmon
{
namespace monitor
{
namespace process_info
{

/**
 * @brief A time value counted in kernel jiffies
 *
 * @sa kernel_hz()
 **/
typedef unsigned long jiffies_t;

//! Number of kernel jiffies per second
jiffies_t kernel_hz();

//! Kernel page size
std::size_t page_size();

/**
 * Process state extracted from /proc/<pid>/stat
 **/
struct ProcessStat
{
	unsigned long pid;
	unsigned long pgrp; //!< Process group ID
	jiffies_t utime;    //!< Total time spent in userspace
	jiffies_t stime;    //!< Total time spent in kernel space
	std::size_t mem_rss; //!< Resident memory size in bytes
};

/**
 * Read process state from /proc/<pid>/stat
 *
 * @param filename Filename of the stat file (e.g. "/proc/1234/stat")
 * @param stat Output struct
 * @return true on success
 **/
bool readStatFile(const char* filename, ProcessStat* stat);

}
}
}

#endif

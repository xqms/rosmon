// Provides process information on linux systems
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "linux_process_info.h"

#include <unistd.h>
#include <cstdio>
#include <cstring>

namespace rosmon
{
namespace monitor
{
namespace process_info
{

static jiffies_t g_kernel_hz = -1;
static std::size_t g_page_size = -1;

jiffies_t kernel_hz()
{
	if(g_kernel_hz == (jiffies_t)-1)
	{
		g_kernel_hz = sysconf(_SC_CLK_TCK);
		if(g_kernel_hz == (jiffies_t)-1)
		{
			fprintf(stderr, "Warning: Could not obtain value of USER_HZ."
				"CPU load measurements might be inaccurate.\n"
			);
			g_kernel_hz = 100;
		}
	}

	return g_kernel_hz;
}

std::size_t page_size()
{
	if(g_page_size == (std::size_t)-1)
	{
		g_page_size = sysconf(_SC_PAGESIZE);
		if(g_page_size == (std::size_t)-1)
		{
			fprintf(stderr, "Warning: Could not obtain page size.");
			g_page_size = 4096;
		}
	}

	return g_page_size;
}

bool readStatFile(const char* filename, ProcessStat* stat)
{
	FILE* f = fopen(filename, "r");
	if(!f)
		return false;

	char buf[1024];
	int ret = fread(buf, 1, sizeof(buf)-1, f);
	fclose(f);

	if(ret <= 0)
		return false;

	buf[ret] = 0;

	unsigned long pid = 0;
	if(sscanf(buf, "%lu", &pid) != 1)
		return false;

	// from procps: skip "(filename)"
	char* start = strrchr(buf, ')');

	if(start - buf > (int)strlen(buf) - 4)
		return false;

	// Skip ")"
	start += 2;

	unsigned long pgrp = 0;
	long long unsigned int user_jiffies = 0;
	long long unsigned int kernel_jiffies = 0;
	long long unsigned int rss_pages = 0;

	// Parse interesting fields
	ret = sscanf(start,
		"%*c " // state
		"%*u " // ppid
		"%lu " // pgrp
		"%*u " // sid
		"%*u " // tty_nr
		"%*u " // tty_pgrp
		"%*u " // flags
		"%*u " // number of minor faults
		"%*u " // number of minor faults with child's
		"%*u " // number of major faults
		"%*u " // number of major faults with child's
		"%llu " // user mode jiffies
		"%llu " // kernel mode jiffies
		"%*u " // user mode jiffies with child's
		"%*u " // kernel mode jiffies with child's
		"%*u " // priority level
		"%*u " // nice level
		"%*u " // num_threads
		"%*u " // it_real_value (obsolete, always 0)
		"%*u " // time the process started after system boot
		"%*u " // virtual memory size
		"%llu " // resident memory size
		, // many more fields follow
		&pgrp,
		&user_jiffies,
		&kernel_jiffies,
		&rss_pages
	);

	if(ret != 4)
		return false;

	stat->pid = pid;
	stat->pgrp = pgrp;
	stat->utime = user_jiffies;
	stat->stime = kernel_jiffies;
	stat->mem_rss = rss_pages * page_size();

	return true;
}


}
}
}

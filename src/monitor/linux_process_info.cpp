// Provides process information on linux systems
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "linux_process_info.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>

namespace rosmon
{
namespace monitor
{
namespace process_info
{

static jiffies_t g_kernel_hz = -1;

jiffies_t kernel_hz()
{
	if(g_kernel_hz == -1)
	{
		g_kernel_hz = sysconf(_SC_CLK_TCK);
		if(g_kernel_hz == -1)
		{
			fprintf(stderr, "Warning: Could not obtain value of USER_HZ."
				"CPU load measurements might be inaccurate.\n"
			);
			g_kernel_hz = 100;
		}
	}

	return g_kernel_hz;
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

	if(start - buf > strlen(buf) - 4)
		return false;

	// Skip ")"
	start += 2;

	unsigned long pgrp = 0;
	long long unsigned int user_jiffies = 0;
	long long unsigned int kernel_jiffies = 0;

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
		, // many more fields follow
		&pgrp,
		&user_jiffies,
		&kernel_jiffies
	);

	if(ret != 3)
		return false;

	stat->pid = pid;
	stat->pgrp = pgrp;
	stat->utime = user_jiffies;
	stat->stime = kernel_jiffies;

	return true;
}


}
}
}

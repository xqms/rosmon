// Sets up a node process environment and executes the target node
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <stdexcept>
#include <vector>
#include <sstream>

#include <unistd.h>
#include <getopt.h>
#include <string.h>

#include <utmp.h>

#include <sys/time.h>
#include <sys/resource.h>
#include <sys/prctl.h>

static const struct option OPTIONS[] = {
	{"help", no_argument, nullptr, 'h'},
	{"namespace", required_argument, nullptr, 'n'},
	{"env", required_argument, nullptr, 'e'},
	{"coredump", no_argument, nullptr, 'c'},
	{"coredump-relative", required_argument, nullptr, 'C'},
	{"tty", required_argument, nullptr, 't'},
	{"run", required_argument, nullptr, 'r'},

	{nullptr, 0, nullptr, 0}
};

void usage()
{
	fprintf(stderr, R"EOS(
This is an internal tool for rosmon. You should not need to call it yourself.

Usage:
  _shim [options] --run <executable> [args...]

Options:
  --namespace=NS           Put the node in namespace NS
  --env=A=B                Set environment variable A to value B (can be repeated)
  --coredump               Enable coredump collection
  --coredump-relative=DIR  Coredumps should go to DIR
  --run <executable>       All arguments after this one are passed on
)EOS");
}

int main(int argc, char** argv)
{
	bool coredumpsEnabled = false;
	char* coredumpsRelative = nullptr;

	char* nodeExecutable = nullptr;
	int nodeOptionsBegin = -1;

	int tty = -1;

	while(true)
	{
		int option_index;
		int c = getopt_long(argc, argv, "h", OPTIONS, &option_index);

		if(c == -1)
			break;

		switch(c)
		{
			case 'h':
				usage();
				return 0;
			case '?':
				usage();
				return 1;
			case 'n':
				setenv("ROS_NAMESPACE", optarg, 1);
				break;
			case 'e':
			{
				char* value = strchr(optarg, '=');
				if(!value)
					throw std::invalid_argument("Need '=' in --env spec");

				*value = 0;

				setenv(optarg, value + 1, 1);
				break;
			}
			case 'c':
				coredumpsEnabled = true;
				break;
			case 'C':
				coredumpsRelative = optarg;
				break;
			case 't':
				tty = atoi(optarg);
				break;
			case 'r':
				nodeExecutable = optarg;
				nodeOptionsBegin = optind;
				break;
		}

		if(nodeExecutable)
			break;
	}

	if(!nodeExecutable)
		throw std::invalid_argument("Need --run option");

	if(tty < 0)
		throw std::invalid_argument("Need --tty option");

	if(login_tty(tty) != 0)
	{
		perror("Could not call login_tty()");
		std::abort();
	}

	// Try to enable core dumps
	if(coredumpsEnabled)
	{
		rlimit limit;
		if(getrlimit(RLIMIT_CORE, &limit) == 0)
		{
			// only modify the limit if coredumps are disabled entirely
			if(limit.rlim_cur == 0)
			{
				limit.rlim_cur = limit.rlim_max;
				setrlimit(RLIMIT_CORE, &limit);
			}
		}

		// If needed for coredump collection with a relative core_pattern,
		// cd to a temporary directory.
		if(coredumpsRelative)
		{
			if(chdir(coredumpsRelative) != 0)
			{
				perror("Could not change to newly created process working directory");
			}
		}
	}
	else
	{
		// Disable coredumps
		rlimit limit;
		if(getrlimit(RLIMIT_CORE, &limit) == 0)
		{
			limit.rlim_cur = 0;
			setrlimit(RLIMIT_CORE, &limit);
		}
	}

	// Allow gdb to attach
	prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY);

	// Build up argument vector
	std::vector<char*> args;

	args.push_back(nodeExecutable);

	for(int i = nodeOptionsBegin; i < argc; ++i)
		args.push_back(argv[i]);

	args.push_back(nullptr);

	// Go!
	if(execvp(nodeExecutable, args.data()) != 0)
	{
		std::stringstream ss;
		for(const auto& part : args)
			ss << part << " ";

		fprintf(stderr, "Could not execute %s: %s\n", ss.str().c_str(), strerror(errno));
	}

	// We should not arrive here
	std::abort();
}

// Simple executable which just aborts (used for testing the core dump feature)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <cstdlib>

int main()
{
	std::abort();
	return 0;
}

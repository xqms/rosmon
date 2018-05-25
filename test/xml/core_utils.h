// General test utilities
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef CORE_UTILS_H
#define CORE_UTILS_H

#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

static void requireParsingException(const std::string& input)
{
	CAPTURE(input);
	REQUIRE_THROWS_AS(
		rosmon::launch::LaunchConfig().parseString(input),
		rosmon::launch::ParseException
	);
}

#endif

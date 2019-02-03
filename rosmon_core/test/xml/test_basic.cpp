// Unit tests for basic loading
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

#include "core_utils.h"
#include "param_utils.h"

using namespace rosmon::launch;

TEST_CASE("basic", "[basic]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
		</launch>
	)EOF");
}

TEST_CASE("basic: invalid XML", "[basic]")
{
	requireParsingException(R"EOF(
		<launch><abc>
		</launch>
	)EOF");
}

TEST_CASE("basic: top-level attributes", "[basic]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch rosmon-name="my_name" rosmon-window-title="my_title">
		</launch>
	)EOF");

	CHECK(config.rosmonNodeName() == "my_name");
	CHECK(config.windowTitle() == "my_title");
}

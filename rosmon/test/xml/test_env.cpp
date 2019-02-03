// Unit tests for env tags
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

#include "param_utils.h"

using namespace rosmon::launch;

TEST_CASE("env basic", "[env]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<env name="test" value="hello world" />

			<node name="test_node" pkg="rosmon" type="abort" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto nodes = config.nodes();
	CAPTURE(nodes);

	REQUIRE(nodes.size() == 1);

	auto node = nodes.at(0);

	auto env = node->extraEnvironment();
	CAPTURE(env);

	CHECK(env.at("test") == "hello world");
}

// Test <remap> tags
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

#include "node_utils.h"

using namespace rosmon::launch;

TEST_CASE("remap", "[remap]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch rosmon-name="rosmon_uut">
				<remap from="topic_a" to="topic_b" />

				<node name="test" pkg="rosmon_core" type="abort">
						<remap from="~local_a" to="/global_a" />
						<remap from="local_b" to="/global_b" />
				</node>

				<remap from="topic_c" to="topic_d" />
		</launch>
	)EOF");

	auto node = getNode(config.nodes(), "test");

	CAPTURE(printMapping(node->remappings()));

	auto maps = node->remappings();

	CHECK(maps.at("topic_a") == "topic_b");
	CHECK(maps.at("~local_a") == "/global_a");
	CHECK(maps.at("local_b") == "/global_b");

	CHECK(maps.find("topic_c") == maps.end());
}

TEST_CASE("remap scoped", "[remap]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch rosmon-name="rosmon_uut">
				<group ns="namespace">
					<remap from="topic_a" to="topic_b" />
				</group>

				<node name="test" pkg="rosmon_core" type="abort">
				</node>
		</launch>
	)EOF");

	auto node = getNode(config.nodes(), "test");

	CAPTURE(printMapping(node->remappings()));

	auto maps = node->remappings();

	CHECK(maps.find("topic_a") == maps.end());
}

// Unit tests for arg tags
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

#include "param_utils.h"

using namespace rosmon::launch;

TEST_CASE("arg basic", "[arg]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<arg name="arg1" value="hello world" />
			<arg name="arg2" default="hello world" />

			<param name="arg1" value="$(arg arg1)" />
			<param name="arg2" value="$(arg arg2)" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto params = config.parameters();

	CHECK(getTypedParam<std::string>(params, "/arg1") == "hello world");
	CHECK(getTypedParam<std::string>(params, "/arg2") == "hello world");
}

TEST_CASE("arg from external", "[arg]")
{
	LaunchConfig config;

	config.setArgument("arg1", "test");
	config.setArgument("arg2", "test");

	config.parseString(R"EOF(
		<launch>
			<arg name="arg1" />
			<arg name="arg2" default="hello world" />

			<param name="arg1" value="$(arg arg1)" />
			<param name="arg2" value="$(arg arg2)" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto params = config.parameters();

	CHECK(getTypedParam<std::string>(params, "/arg1") == "test");
	CHECK(getTypedParam<std::string>(params, "/arg2") == "test");
}

TEST_CASE("arg unset", "[arg]")
{
	REQUIRE_THROWS_AS(
		LaunchConfig().parseString(R"EOF(
			<launch>
				<param name="test" value="$(arg arg1)" />
			</launch>
		)EOF"),
		ParseException
	);

	REQUIRE_THROWS_AS(
		LaunchConfig().parseString(R"EOF(
			<launch>
				<arg name="arg1" />
				<param name="test" value="$(arg arg1)" />
			</launch>
		)EOF"),
		ParseException
	);
}

// Unit tests for include tags
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

#include "param_utils.h"

using namespace rosmon::launch;

TEST_CASE("include basic", "[include]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<arg name="test_argument" value="hello" />

			<include file="$(find rosmon_core)/test/basic_sub.launch">
				<arg name="test_argument" value="$(arg test_argument)" />
			</include>
		</launch>
	)EOF");

	config.evaluateParameters();

	auto params = config.parameters();

	CHECK(getTypedParam<std::string>(params, "/test_argument") == "hello");
}

TEST_CASE("include default", "[include]")
{
	// roslaunch allows this - to me it seems quite confusing, since the value
	// of the arg tag cannot be overriden, despite using "default".

	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<arg name="test_argument" value="hello" />

			<include file="$(find rosmon_core)/test/basic_sub.launch">
				<arg name="test_argument" default="$(arg test_argument)" />
			</include>
		</launch>
	)EOF");

	config.evaluateParameters();

	auto params = config.parameters();

	CHECK(getTypedParam<std::string>(params, "/test_argument") == "hello");
}

TEST_CASE("include pass_all", "[include]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<arg name="test_argument" value="hello" />

			<include file="$(find rosmon_core)/test/basic_sub.launch" pass_all_args="true" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto params = config.parameters();

	CHECK(getTypedParam<std::string>(params, "/test_argument") == "hello");
}

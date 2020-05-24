// Unit tests for include tags
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

#include "node_utils.h"
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

	std::stringstream warnings;

	LaunchConfig config;
	config.setWarningOutput(&warnings);
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

	CAPTURE(warnings.str());
	CHECK(warnings.str().find("default") != std::string::npos);
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

TEST_CASE("include scoped attributes", "[include]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<include file="$(find rosmon_core)/test/basic_sub.launch"
				enable-coredumps="false" rosmon-memory-limit="100" rosmon-stop-timeout="10.0" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto nodes = config.nodes();
	CAPTURE(nodes);

	auto n = getNode(nodes, "test_node");
	CHECK(!n->coredumpsEnabled());
	CHECK(n->memoryLimitByte() == 100);
	CHECK(n->stopTimeout() == Approx(10.0));
}

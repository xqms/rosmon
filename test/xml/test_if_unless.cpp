// Unit tests for if/unless attributes
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

#include "param_utils.h"

using namespace rosmon::launch;

TEST_CASE("if/unless basic", "[if_unless]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<param if="false" name="test_if" value="hello" />
			<param if="true" name="test_if" value="world" />

			<param if="1" name="test_if_number" value="hello" />
			<param if="0" name="test_if_number" value="world" />

			<param unless="false" name="test_unless" value="hello" />
			<param unless="true" name="test_unless" value="world" />
		</launch>
	)EOF");

	config.evaluateParameters();

	CAPTURE(config.parameters());

	auto& params = config.parameters();
	checkTypedParam<std::string>(params, "/test_if", XmlRpc::XmlRpcValue::TypeString, "world");
	checkTypedParam<std::string>(params, "/test_if_number", XmlRpc::XmlRpcValue::TypeString, "hello");
	checkTypedParam<std::string>(params, "/test_unless", XmlRpc::XmlRpcValue::TypeString, "hello");
}

TEST_CASE("if/unless invalid", "[if_unless]")
{
	using Catch::Matchers::Contains;

	REQUIRE_THROWS_WITH(
		LaunchConfig().parseString(R"EOF(<launch><param if="true" unless="true" name="test" value="test" /></launch>)EOF"),
		Contains("if") && Contains("unless")
	);

	REQUIRE_THROWS_AS(
		LaunchConfig().parseString(R"EOF(<launch><param if="unknown_value" name="test" value="test" /></launch>)EOF"),
		ParseException
	);
}

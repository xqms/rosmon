// Unit tests for rosparam tag
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

#include "core_utils.h"
#include "param_utils.h"

using namespace rosmon::launch;

TEST_CASE("rosparam basic", "[rosparam]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
<rosparam>
test_ns:
  param1: true
  param2: hello
  param3: 3
  param4: 10.0
  param5: "3"
</rosparam>
		</launch>
	)EOF");

	config.evaluateParameters();

	CAPTURE(config.parameters());

	auto& params = config.parameters();
	checkTypedParam<bool>(params, "/test_ns/param1", XmlRpc::XmlRpcValue::TypeBoolean, true);
	checkTypedParam<std::string>(params, "/test_ns/param2", XmlRpc::XmlRpcValue::TypeString, "hello");
	checkTypedParam<int>(params, "/test_ns/param3", XmlRpc::XmlRpcValue::TypeInt, 3);
	checkTypedParam<double>(params, "/test_ns/param4", XmlRpc::XmlRpcValue::TypeDouble, 10.0);
	checkTypedParam<std::string>(params, "/test_ns/param5", XmlRpc::XmlRpcValue::TypeString, "3");
}

TEST_CASE("rosparam empty", "[rosparam]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
<rosparam>
</rosparam>

			<rosparam command="load" file="$(find rosmon_core)/test/empty.yaml" />
		</launch>
	)EOF");
}

TEST_CASE("rosparam invalid YAML", "[rosparam]")
{
	requireParsingException(R"EOF(
		<launch>
<rosparam>
hello: {{ invalid }} test
</rosparam>
		</launch>
	)EOF");
}

TEST_CASE("rosparam naming", "[rosparam]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<rosparam param="a_list">[1, 2, 3, 4]</rosparam>

			<arg name="whitelist" default="[3, 2]"/>
			<rosparam param="whitelist" subst_value="True">$(arg whitelist)</rosparam>

<rosparam ns="namespace">
a: false
</rosparam>
		</launch>
	)EOF");

	config.evaluateParameters();

	CAPTURE(config.parameters());

	{
		INFO("looking for: a_list");

		auto it = config.parameters().find("/a_list");
		REQUIRE(it != config.parameters().end());

		auto value = it->second;

		REQUIRE(value.getType() == XmlRpc::XmlRpcValue::TypeArray);
		REQUIRE(value.size() == 4);
	}

	{
		INFO("looking for: whitelist");

		auto it = config.parameters().find("/whitelist");
		REQUIRE(it != config.parameters().end());

		auto value = it->second;

		REQUIRE(value.getType() == XmlRpc::XmlRpcValue::TypeArray);
		REQUIRE(value.size() == 2);
	}

	checkTypedParam<bool>(config.parameters(), "/namespace/a", XmlRpc::XmlRpcValue::TypeBoolean, false);
}

TEST_CASE("rosparam explicit types", "[rosparam]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
<rosparam>
test_ns:
  param1: !!str 1
  param2: !!float 1
  param3: !!binary "aGVsbG8K"
</rosparam>
		</launch>
	)EOF");

	config.evaluateParameters();

	CAPTURE(config.parameters());

	auto& params = config.parameters();
	checkTypedParam<std::string>(params, "/test_ns/param1", XmlRpc::XmlRpcValue::TypeString, "1");
	checkTypedParam<double>(params, "/test_ns/param2", XmlRpc::XmlRpcValue::TypeDouble, 1.0);

	auto binParam = getTypedParam<XmlRpc::XmlRpcValue::BinaryData>(params, "/test_ns/param3", XmlRpc::XmlRpcValue::TypeBase64);
	XmlRpc::XmlRpcValue::BinaryData expected{'h', 'e', 'l', 'l', 'o', '\n'};
	CHECK(binParam == expected);
}

TEST_CASE("rosparam angle extensions", "[rosparam]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
<rosparam>
test_ns:
  param1: rad(2*pi)
  param2: deg(180)
  param3: !degrees 181.0
  param4: !radians 3.14169
</rosparam>
		</launch>
	)EOF");

	config.evaluateParameters();

	CAPTURE(config.parameters());

	auto& params = config.parameters();

	double param1 = getTypedParam<double>(params, "/test_ns/param1", XmlRpc::XmlRpcValue::TypeDouble);
	CHECK(param1 == Approx(2.0 * M_PI));

	double param2 = getTypedParam<double>(params, "/test_ns/param2", XmlRpc::XmlRpcValue::TypeDouble);
	CHECK(param2 == Approx(180.0 * M_PI / 180.0));

	double param3 = getTypedParam<double>(params, "/test_ns/param3", XmlRpc::XmlRpcValue::TypeDouble);
	CHECK(param3 == Approx(181.0 * M_PI / 180.0));

	double param4 = getTypedParam<double>(params, "/test_ns/param4", XmlRpc::XmlRpcValue::TypeDouble);
	CHECK(param4 == Approx(3.14169));
}

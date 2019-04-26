// Unit tests for <param> tags
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

#include "param_utils.h"

using namespace rosmon::launch;

TEST_CASE("global_param", "[param]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<param name="global_param" value="hello_world" />
		</launch>
	)EOF");

	config.evaluateParameters();

	CAPTURE(config.parameters());

	auto it = config.parameters().find("/global_param");

	REQUIRE(it != config.parameters().end());

	XmlRpc::XmlRpcValue value = it->second;
	REQUIRE(static_cast<std::string>(value) == "hello_world");
}

TEST_CASE("param_types", "[param]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<param name="int_param_auto" value="0" />
			<param name="int_param_forced" value="0" type="int" />

			<param name="double_param_auto" value="0.0" />
			<param name="double_param_forced" value="0" type="double" />

			<param name="str_param_auto" value="hello" />
			<param name="str_param_forced" value="0" type="str" />

			<param name="bool_param_auto" value="true" />
			<param name="bool_param_forced" value="true" type="boolean" />

			<param name="bool_param_auto_nonlowercase" value="True" />
			<param name="bool_param_forced_nonlowercase" value="True" type="boolean" />

			<param name="yaml_param" type="yaml" value="test_param: true" />
			<param name="yaml_param_scalar" type="yaml" value="true" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto& params = config.parameters();
	checkTypedParam<int>(params, "/int_param_auto", XmlRpc::XmlRpcValue::TypeInt, 0);
	checkTypedParam<int>(params, "/int_param_forced", XmlRpc::XmlRpcValue::TypeInt, 0);

	checkTypedParam<double>(params, "/double_param_auto", XmlRpc::XmlRpcValue::TypeDouble, 0.0);
	checkTypedParam<double>(params, "/double_param_forced", XmlRpc::XmlRpcValue::TypeDouble, 0.0);

	checkTypedParam<std::string>(params, "/str_param_auto", XmlRpc::XmlRpcValue::TypeString, "hello");
	checkTypedParam<std::string>(params, "/str_param_forced", XmlRpc::XmlRpcValue::TypeString, "0");

	checkTypedParam<bool>(params, "/bool_param_auto", XmlRpc::XmlRpcValue::TypeBoolean, true);
	checkTypedParam<bool>(params, "/bool_param_forced", XmlRpc::XmlRpcValue::TypeBoolean, true);

	checkTypedParam<bool>(params, "/bool_param_auto_nonlowercase", XmlRpc::XmlRpcValue::TypeBoolean, true);
	checkTypedParam<bool>(params, "/bool_param_forced_nonlowercase", XmlRpc::XmlRpcValue::TypeBoolean, true);

	checkTypedParam<bool>(params, "/yaml_param/test_param", XmlRpc::XmlRpcValue::TypeBoolean, true);
	checkTypedParam<bool>(params, "/yaml_param_scalar", XmlRpc::XmlRpcValue::TypeBoolean, true);
}

TEST_CASE("param command", "[param]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<param name="test" command="echo -n hello_world" />

			<param name="multiline" command="/bin/echo -ne hello\\nworld" />

			<param name="yaml_param" type="yaml" command="echo test_param: true" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto& params = config.parameters();

	checkTypedParam<std::string>(params, "/test", XmlRpc::XmlRpcValue::TypeString, "hello_world");
	checkTypedParam<std::string>(params, "/multiline", XmlRpc::XmlRpcValue::TypeString, "hello\nworld");

	checkTypedParam<bool>(params, "/yaml_param/test_param", XmlRpc::XmlRpcValue::TypeBoolean, true);
}

TEST_CASE("param failing command", "[param]")
{
	LaunchConfig config;
	config.parseString(R"EOF(<launch><param name="test" command="false" /></launch>)EOF");
	REQUIRE_THROWS_AS(
		config.evaluateParameters(),
		ParseException
	);
}

TEST_CASE("param textfile", "[param]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<param name="test" textfile="$(find rosmon_core)/test/textfile.txt" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto& params = config.parameters();

	checkTypedParam<std::string>(params, "/test", XmlRpc::XmlRpcValue::TypeString, "hello_world");
}

TEST_CASE("param textfile does not exist", "[param]")
{
	using Catch::Matchers::Contains;

	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<param name="test" textfile="$(find rosmon_core)/test/textfile_does_not_exist.txt" />
		</launch>
	)EOF");

	REQUIRE_THROWS_WITH(
		config.evaluateParameters(),
		Contains("file")
	);
}

TEST_CASE("param binfile", "[param]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<param name="test" binfile="$(find rosmon_core)/test/textfile.txt" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto& params = config.parameters();

	CAPTURE(params);

	auto it = params.find("/test");
	REQUIRE(it != params.end());

	XmlRpc::XmlRpcValue value = it->second;

	REQUIRE(value.getType() == XmlRpc::XmlRpcValue::TypeBase64);

	std::string expectedData = "hello_world";
	auto& data = static_cast<XmlRpc::XmlRpcValue::BinaryData&>(value);

	REQUIRE(expectedData.size() == data.size());
	for(std::size_t i = 0; i < expectedData.size(); ++i)
		REQUIRE(data[i] == expectedData[i]);
}

TEST_CASE("scoped params", "[param]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<param name="global/param" value="abc" />
			<param name="/global/param2" value="def" />

			<group ns="namespace">
				<param name="test" value="val1" />
				<param name="/test2" value="val2" />
			</group>

			<node name="test_node" pkg="rosmon_core" type="abort">
				<param name="private" value="val3" />
				<param name="~private2" value="val4" />
				<param name="/leading_slash" value="val5" />
			</node>
		</launch>
	)EOF");

	config.evaluateParameters();

	auto& params = config.parameters();

	checkTypedParam<std::string>(params, "/global/param", XmlRpc::XmlRpcValue::TypeString, "abc");
	checkTypedParam<std::string>(params, "/global/param2", XmlRpc::XmlRpcValue::TypeString, "def");

	checkTypedParam<std::string>(params, "/namespace/test", XmlRpc::XmlRpcValue::TypeString, "val1");
	checkTypedParam<std::string>(params, "/test2", XmlRpc::XmlRpcValue::TypeString, "val2");

	checkTypedParam<std::string>(params, "/test_node/private", XmlRpc::XmlRpcValue::TypeString, "val3");
	checkTypedParam<std::string>(params, "/test_node/private2", XmlRpc::XmlRpcValue::TypeString, "val4");

	checkTypedParam<std::string>(params, "/test_node/leading_slash", XmlRpc::XmlRpcValue::TypeString, "val5");
}

TEST_CASE("scoped params with double slash (#49)", "[param]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<group ns="/">
				<param name="param1" value="hello" />
			</group>

			<node name="test_node" pkg="rosmon_core" type="abort" ns="/racecar">
				<param name="private_param" value="hello again" />
			</node>
		</launch>
	)EOF");

	config.evaluateParameters();

	auto& params = config.parameters();

	checkTypedParam<std::string>(params, "/param1", XmlRpc::XmlRpcValue::TypeString, "hello");
	checkTypedParam<std::string>(params, "/racecar/test_node/private_param", XmlRpc::XmlRpcValue::TypeString, "hello again");
}

TEST_CASE("wrong param types", "[param]")
{
	REQUIRE_THROWS_AS(
		LaunchConfig().parseString(R"EOF(<launch><param name="test" value="abc" type="int" /></launch>)EOF"),
		ParseException
	);

	REQUIRE_THROWS_AS(
		LaunchConfig().parseString(R"EOF(<launch><param name="test" value="0.5" type="int" /></launch>)EOF"),
		ParseException
	);

	REQUIRE_THROWS_AS(
		LaunchConfig().parseString(R"EOF(<launch><param name="test" value="0.5" type="bool" /></launch>)EOF"),
		ParseException
	);

	REQUIRE_THROWS_AS(
		LaunchConfig().parseString(R"EOF(<launch><param name="test" value="invalid: {{ yaml}} here" type="yaml" /></launch>)EOF"),
		ParseException
	);

	{
		LaunchConfig config;

		config.parseString(R"EOF(<launch><param name="test" command="echo -ne invalid: {{ yaml}} here" type="yaml" /></launch>)EOF");

		REQUIRE_THROWS_AS(
			config.evaluateParameters(),
			ParseException
		);
	}

	REQUIRE_THROWS_AS(
		LaunchConfig().parseString(R"EOF(<launch><param name="test" value="0.5" type="unknown_type" /></launch>)EOF"),
		ParseException
	);
}

TEST_CASE("invalid param input combinations", "[param]")
{
	REQUIRE_THROWS_AS(
		LaunchConfig().parseString(R"EOF(<launch><param name="test" value="abc" command="echo -ne test" /></launch>)EOF"),
		ParseException
	);

	REQUIRE_THROWS_AS(
		LaunchConfig().parseString(R"EOF(<launch><param name="test" textfile="$(find rosmon_core)/test/textfile.txt" command="echo -ne test" /></launch>)EOF"),
		ParseException
	);

	REQUIRE_THROWS_AS(
		LaunchConfig().parseString(R"EOF(<launch><param name="test" /></launch>)EOF"),
		ParseException
	);
}

TEST_CASE("invalid param names", "[param]")
{
	using Catch::Matchers::Contains;

	REQUIRE_THROWS_WITH(
		LaunchConfig().parseString(R"EOF(<launch><param name="$%*" value="abc" /></launch>)EOF"),
		Contains("$%*")
	);

	REQUIRE_THROWS_WITH(
		LaunchConfig().parseString(R"EOF(<launch><param value="abc" /></launch>)EOF"),
		Contains("name")
	);
}


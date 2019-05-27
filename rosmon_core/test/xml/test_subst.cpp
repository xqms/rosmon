// Unit tests for substitution args
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <boost/filesystem.hpp>

#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

#include "core_utils.h"
#include "param_utils.h"

#include <ros/package.h>

using namespace rosmon::launch;

namespace fs = boost::filesystem;

TEST_CASE("env", "[subst]")
{
	SECTION("basic")
	{
		LaunchConfig config;
		config.parseString(R"EOF(
			<launch>
				<param name="env_test" value="$(env PATH)" />
			</launch>
		)EOF");

		config.evaluateParameters();

		CAPTURE(config.parameters());

		checkTypedParam<std::string>(config.parameters(), "/env_test", XmlRpc::XmlRpcValue::TypeString, getenv("PATH"));
	}

	SECTION("failure")
	{
		requireParsingException(R"EOF(
			<launch>
				<param name="env_test" value="$(env ROSMON_UNLIKELY_TO_BE_SET)" />
			</launch>
		)EOF");
	}
}

TEST_CASE("optenv", "[subst]")
{
	SECTION("present")
	{
		LaunchConfig config;
		config.parseString(R"EOF(
			<launch>
				<param name="env_test" value="$(optenv PATH no_such_thing)" />
			</launch>
		)EOF");

		config.evaluateParameters();

		CAPTURE(config.parameters());

		checkTypedParam<std::string>(config.parameters(), "/env_test", XmlRpc::XmlRpcValue::TypeString, getenv("PATH"));
	}

	SECTION("not present")
	{
		LaunchConfig config;
		config.parseString(R"EOF(
			<launch>
				<param name="env_test" value="$(optenv ROSMON_UNLIKELY_TO_BE_SET no_such_thing)" />
			</launch>
		)EOF");

		config.evaluateParameters();

		CAPTURE(config.parameters());

		checkTypedParam<std::string>(config.parameters(), "/env_test", XmlRpc::XmlRpcValue::TypeString, "no_such_thing");
	}

	SECTION("not present - long")
	{
		LaunchConfig config;
		config.parseString(R"EOF(
			<launch>
				<param name="env_test" value="$(optenv ROSMON_UNLIKELY_TO_BE_SET no such thing)" />
			</launch>
		)EOF");

		config.evaluateParameters();

		CAPTURE(config.parameters());

		checkTypedParam<std::string>(config.parameters(), "/env_test", XmlRpc::XmlRpcValue::TypeString, "no such thing");
	}
}

TEST_CASE("find", "[subst]")
{
	SECTION("regular use")
	{
		LaunchConfig config;
		config.parseString(R"EOF(
			<launch>
				<param name="path_to_rosmon" value="$(find rosmon_core)" />
				<param name="path_to_launch_file" value="$(find rosmon_core)/test/basic.launch" />
				<param name="path_to_rosmon_executable" value="$(find rosmon_core)/rosmon" />
			</launch>
		)EOF");

		config.evaluateParameters();

		CAPTURE(config.parameters());

		checkTypedParam<std::string>(config.parameters(), "/path_to_rosmon", XmlRpc::XmlRpcValue::TypeString, ros::package::getPath("rosmon_core"));
		checkTypedParam<std::string>(config.parameters(), "/path_to_launch_file", XmlRpc::XmlRpcValue::TypeString, ros::package::getPath("rosmon_core") + "/test/basic.launch");

		{
			INFO("Looking for /path_to_rosmon_executable");

			auto it = config.parameters().find("/path_to_rosmon_executable");
			REQUIRE(it != config.parameters().end());

			auto value = it->second;

			REQUIRE(value.getType() == XmlRpc::XmlRpcValue::TypeString);
			auto string = static_cast<std::string>(value);

			CHECK((fs::status(string).permissions() & fs::owner_exe));
		}
	}

	SECTION("unknown package")
	{
		requireParsingException(R"EOF(
			<launch>
				<param name="test" value="$(find rosmon_this_package_is_unlikely_to_be_there)" />
			</launch>
		)EOF");
	}
}

TEST_CASE("anon", "[subst]")
{
	SECTION("regular use")
	{
		LaunchConfig config;
		config.parseString(R"EOF(
			<launch>
				<param name="test_1" value="$(anon rviz-1)" />
				<param name="test_2" value="$(anon rviz-1)" />
				<param name="test_3" value="$(anon rviz-2)" />
			</launch>
		)EOF");

		config.evaluateParameters();

		CAPTURE(config.parameters());

		auto test1 = getTypedParam<std::string>(config.parameters(), "/test_1");
		auto test2 = getTypedParam<std::string>(config.parameters(), "/test_2");
		auto test3 = getTypedParam<std::string>(config.parameters(), "/test_3");

		CHECK(test1 == test2);
		CHECK(test1 != test3);
	}

	SECTION("clash example")
	{
		// from http://wiki.ros.org/roslaunch/XML
		requireParsingException(R"EOF(
			<launch>
				<node name="$(anon foo)" pkg="rospy_tutorials" type="talker.py" />
				<node name="$(anon foo)" pkg="rospy_tutorials" type="talker.py" />
			</launch>
		)EOF");
	}
}

TEST_CASE("arg", "[subst]")
{
	SECTION("basic use")
	{
		LaunchConfig config;
		config.parseString(R"EOF(
			<launch>
				<arg name="test_arg" default="hello" />
				<param name="test" value="$(arg test_arg)" />
			</launch>
		)EOF");

		config.evaluateParameters();

		CHECK(getTypedParam<std::string>(config.parameters(), "/test") == "hello");
	}

	SECTION("unknown arg")
	{
		requireParsingException(R"EOF(
			<launch>
				<arg name="test_arg" default="hello" />
				<param name="test" value="$(arg test_arg_unknown)" />
			</launch>
		)EOF");
	}

	// more complicated tests may be in test_arg.cpp
}

TEST_CASE("eval", "[subst]")
{
	SECTION("example 1")
	{
		LaunchConfig config;
		config.parseString(R"EOF(
			<launch>
				<arg name="radius" default="2.0" />
				<param name="circumference" value="$(eval 2.* pi * arg('radius'))"/>
			</launch>
		)EOF");

		config.evaluateParameters();

		auto value = getTypedParam<double>(config.parameters(), "/circumference");
		CHECK(value == Approx(2.0 * M_PI * 2.0));
	}

	SECTION("example 2")
	{
		LaunchConfig config;
		config.parseString(R"EOF(
			<launch>
				<arg name="foo" default="test" />
				<param name="test" value="$(eval arg('foo') + env('PATH') + 'bar' + find('rosmon_core'))"/>
			</launch>
		)EOF");

		config.evaluateParameters();

		auto value = getTypedParam<std::string>(config.parameters(), "/test");
		CHECK(value == std::string("test") + getenv("PATH") + "bar" + ros::package::getPath("rosmon_core"));
	}

	SECTION("example 3")
	{
		LaunchConfig config;
		config.parseString(R"EOF(
			<launch>
				<arg name="foo" default="test" />
				<param name="test" value="$(eval foo + env('PATH') + 'bar' + find('rosmon_core'))"/>
			</launch>
		)EOF");

		config.evaluateParameters();

		auto value = getTypedParam<std::string>(config.parameters(), "/test");
		CHECK(value == std::string("test") + getenv("PATH") + "bar" + ros::package::getPath("rosmon_core"));
	}

	SECTION("example 4")
	{
		LaunchConfig config;
		config.parseString(R"EOF(
			<launch>
				<arg name="foo" default="test" />
				<param name="test" value="$(eval 2 * 20)"/>
			</launch>
		)EOF");

		config.evaluateParameters();

		auto value = getTypedParam<int>(config.parameters(), "/test");
		CHECK(value == 2*20);
	}

	SECTION("python errors")
	{
		using Catch::Matchers::Contains;

		std::string input = R"EOF(
			<launch>
				<arg name="foo" default="test" />
				<param name="test" value="$(eval )))"/>
			</launch>
		)EOF";
		CAPTURE(input);

		REQUIRE_THROWS_WITH(
			LaunchConfig().parseString(input),
			Contains("SyntaxError")
		);

		requireParsingException(R"EOF(
			<launch>
				<arg name="foo" default="test" />
				<param name="test" value="$(eval acos)"/>
			</launch>
		)EOF");
	}

	SECTION("lowercase booleans")
	{
		LaunchConfig config;
		config.parseString(R"EOF(
			<launch>
				<arg name="input" default="false" />
				<param name="output" value="$(eval input == true)"/>
			</launch>
		)EOF");

		config.evaluateParameters();

		auto value = getTypedParam<bool>(config.parameters(), "/output");
		CHECK(value == false);
	}
}

TEST_CASE("dirname", "[subst]")
{
	SECTION("basic usage")
	{
		LaunchConfig config;
		config.parseString(R"EOF(
			<launch>
				<param name="test" value="$(dirname)" />
			</launch>
		)EOF");

		config.evaluateParameters();

		auto value = getTypedParam<std::string>(config.parameters(), "/test");

		CHECK(fs::path(value) == fs::current_path());
	}
}

TEST_CASE("subst invalid", "[subst]")
{
	SECTION("unknown subst")
	{
		requireParsingException(R"EOF(
			<launch>
				<param name="test" value="$(unknown_subst parameter)" />
			</launch>
		)EOF");
	}
}

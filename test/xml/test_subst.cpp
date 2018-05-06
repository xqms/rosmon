// Unit tests for substitution args
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <boost/filesystem.hpp>

#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

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
		REQUIRE_THROWS_AS(
			LaunchConfig().parseString(R"EOF(
				<launch>
					<param name="env_test" value="$(env ROSMON_UNLIKELY_TO_BE_SET)" />
				</launch>
			)EOF"),
			LaunchConfig::ParseException
		);
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
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<param name="path_to_rosmon" value="$(find rosmon)" />
			<param name="path_to_launch_file" value="$(find rosmon)/test/basic.launch" />
			<param name="path_to_rosmon_executable" value="$(find rosmon)/rosmon" />
		</launch>
	)EOF");

	config.evaluateParameters();

	CAPTURE(config.parameters());

	checkTypedParam<std::string>(config.parameters(), "/path_to_rosmon", XmlRpc::XmlRpcValue::TypeString, ros::package::getPath("rosmon"));
	checkTypedParam<std::string>(config.parameters(), "/path_to_launch_file", XmlRpc::XmlRpcValue::TypeString, ros::package::getPath("rosmon") + "/test/basic.launch");

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
		REQUIRE_THROWS_AS(
			LaunchConfig().parseString(R"EOF(
				<launch>
					<node name="$(anon foo)" pkg="rospy_tutorials" type="talker.py" />
					<node name="$(anon foo)" pkg="rospy_tutorials" type="talker.py" />
				</launch>
			)EOF"),
			LaunchConfig::ParseException
		);
	}
}

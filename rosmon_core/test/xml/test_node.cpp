// Unit tests for node tags
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

#include <boost/filesystem.hpp>

#include "core_utils.h"
#include "node_utils.h"
#include "param_utils.h"

using namespace rosmon::launch;

TEST_CASE("node basic", "[node]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<node name="test_node" pkg="rosmon_core" type="abort" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto nodes = config.nodes();
	CAPTURE(nodes);

	REQUIRE(nodes.size() == 1);

	auto node = getNode(nodes, "test_node");

	CHECK(node->name() == "test_node");
	CHECK(node->package() == "rosmon_core");
	CHECK(node->type() == "abort");

	{
		namespace fs = boost::filesystem;

		fs::path executable = node->executable();
		CAPTURE(executable.string());
		CHECK((fs::status(executable).permissions() & fs::owner_exe));
	}
}

TEST_CASE("node invalid", "[node]")
{
	requireParsingException(R"EOF(
		<launch>
			<node name="test_node" />
		</launch>
	)EOF");
}

TEST_CASE("node args", "[node]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<node name="test_node" pkg="rosmon_core" type="abort" args="arg1 arg2 'long arg'" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto nodes = config.nodes();
	CAPTURE(nodes);

	REQUIRE(nodes.size() == 1);

	auto node = getNode(nodes, "test_node");

	auto args = node->extraArguments();

	REQUIRE(args.size() == 3);
	CHECK(args[0] == "arg1");
	CHECK(args[1] == "arg2");
	CHECK(args[2] == "long arg");
}

TEST_CASE("node respawn", "[node]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<node name="test_node" pkg="rosmon_core" type="abort" respawn="true" respawn_delay="10" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto nodes = config.nodes();
	CAPTURE(nodes);

	REQUIRE(nodes.size() == 1);

	auto node = getNode(nodes, "test_node");

	CHECK(node->respawn());
	CHECK(node->respawnDelay().toSec() == Approx(10.0));
}

TEST_CASE("node required", "[node]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<node name="test_node" pkg="rosmon_core" type="abort" required="true" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto nodes = config.nodes();
	CAPTURE(nodes);

	REQUIRE(nodes.size() == 1);

	auto node = getNode(nodes, "test_node");

	CHECK(node->required());
}

TEST_CASE("node ns", "[node]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<node name="test_node" pkg="rosmon_core" type="abort" ns="namespace" />

			<group ns="ns1">
				<node name="test_node" pkg="rosmon_core" type="abort" />

				<node name="test_node" pkg="rosmon_core" type="abort" ns="namespace" />
			</group>
		</launch>
	)EOF");

	config.evaluateParameters();

	auto nodes = config.nodes();
	CAPTURE(nodes);

	REQUIRE(nodes.size() == 3);

	{
		auto node = getNode(nodes, "test_node", "/namespace");
		CHECK(node->namespaceString() == "/namespace");
	}
	{
		auto node = getNode(nodes, "test_node", "/ns1");
		CHECK(node->namespaceString() == "/ns1");
	}
	{
		auto node = getNode(nodes, "test_node", "/ns1/namespace");
		CHECK(node->namespaceString() == "/ns1/namespace");
	}
}

TEST_CASE("node clear_params", "[node]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<node name="test_node_on" pkg="rosmon_core" type="abort" clear_params="true" />
			<node name="test_node_off" pkg="rosmon_core" type="abort" clear_params="false" />
			<node name="test_node_def" pkg="rosmon_core" type="abort" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto nodes = config.nodes();
	CAPTURE(nodes);

	CHECK(getNode(nodes, "test_node_on")->clearParams() == true);
	CHECK(getNode(nodes, "test_node_off")->clearParams() == false);
	CHECK(getNode(nodes, "test_node_def")->clearParams() == false);
}

TEST_CASE("node cwd", "[node]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<node name="test_node" pkg="rosmon_core" type="abort" cwd="/my_cwd/" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto nodes = config.nodes();
	CAPTURE(nodes);

	auto node = getNode(nodes, "test_node");

	CHECK(node->workingDirectory() == "/my_cwd/");
}

TEST_CASE("node launch-prefix", "[node]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<node name="test_node" pkg="rosmon_core" type="abort" launch-prefix="echo my command is:" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto nodes = config.nodes();
	CAPTURE(nodes);

	auto node = getNode(nodes, "test_node");

	auto prefix = node->launchPrefix();
	CAPTURE(prefix);

	REQUIRE(prefix.size() == 4);
	CHECK(prefix[0] == "echo");
	CHECK(prefix[1] == "my");
	CHECK(prefix[2] == "command");
	CHECK(prefix[3] == "is:");
}

TEST_CASE("node remap", "[remap]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<node name="test_node" pkg="rosmon_core" type="abort">
				<remap from="private1" to="/global_target" />
				<remap from="private2" to="local_target" />
			</node>
		</launch>
	)EOF");

	config.evaluateParameters();

	auto nodes = config.nodes();
	CAPTURE(nodes);

	auto node = getNode(nodes, "test_node");

	auto remappings = node->remappings();
	CAPTURE(printMapping(remappings));

	CHECK(remappings.size() == 2);
	CHECK(remappings.at("private1") == "/global_target");
	CHECK(remappings.at("private2") == "local_target");
}

TEST_CASE("node output attr", "[output]")
{
	SECTION("ignore")
	{
		LaunchConfig config;
		config.setOutputAttrMode(LaunchConfig::OutputAttr::Ignore);
		config.parseString(R"EOF(
			<launch>
				<node name="test_node" pkg="rosmon_core" type="abort" output="log">
				</node>
			</launch>
		)EOF");

		auto node = getNode(config.nodes(), "test_node");
		CHECK(!node->stdoutDisplayed());
	}

	SECTION("obey")
	{
		LaunchConfig config;
		config.setOutputAttrMode(LaunchConfig::OutputAttr::Obey);
		config.parseString(R"EOF(
			<launch>
				<node name="test_node" pkg="rosmon_core" type="abort">
				</node>
			</launch>
		)EOF");

		auto node = getNode(config.nodes(), "test_node");
		CHECK(!node->stdoutDisplayed());
	}

	SECTION("obey log")
	{
		LaunchConfig config;
		config.setOutputAttrMode(LaunchConfig::OutputAttr::Obey);
		config.parseString(R"EOF(
			<launch>
				<node name="test_node" pkg="rosmon_core" type="abort" output="log">
				</node>
			</launch>
		)EOF");

		auto node = getNode(config.nodes(), "test_node");
		CHECK(!node->stdoutDisplayed());
	}

	SECTION("obey screen")
	{
		LaunchConfig config;
		config.setOutputAttrMode(LaunchConfig::OutputAttr::Obey);
		config.parseString(R"EOF(
			<launch>
				<node name="test_node" pkg="rosmon_core" type="abort" output="screen">
				</node>
			</launch>
		)EOF");

		auto node = getNode(config.nodes(), "test_node");
		CHECK(node->stdoutDisplayed());
	}
}

// rosmon extensions

TEST_CASE("node enable-coredumps", "[node]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<node name="test_node_on" pkg="rosmon_core" type="abort" enable-coredumps="true" />
			<node name="test_node_off" pkg="rosmon_core" type="abort" enable-coredumps="false" />
		</launch>
	)EOF");

	config.evaluateParameters();

	auto nodes = config.nodes();
	CAPTURE(nodes);

	CHECK(getNode(nodes, "test_node_on")->coredumpsEnabled() == true);
	CHECK(getNode(nodes, "test_node_off")->coredumpsEnabled() == false);
}

TEST_CASE("node memory/cpu limit", "[node]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
			<node name="test_node_default" pkg="rosmon_core" type="abort" />
			<node name="test_node_custom" pkg="rosmon_core" type="abort" rosmon-memory-limit="200" rosmon-cpu-limit="0.2" />

			<group rosmon-memory-limit="100" enable-coredumps="false">
				<node name="test_node_grouped" pkg="rosmon_core" type="abort" />
			</group>
		</launch>
	)EOF");

	config.evaluateParameters();

	auto nodes = config.nodes();
	CAPTURE(nodes);

	auto def = getNode(nodes, "test_node_default");
	CHECK(def->memoryLimitByte() == rosmon::launch::DEFAULT_MEMORY_LIMIT);
	CHECK(def->cpuLimit() == Approx(rosmon::launch::DEFAULT_CPU_LIMIT));

	auto custom = getNode(nodes, "test_node_custom");
	CHECK(custom->memoryLimitByte() == 200);
	CHECK(custom->cpuLimit() == Approx(0.2f));

	auto grouped = getNode(nodes, "test_node_grouped");
	CHECK(grouped->memoryLimitByte() == 100);
	CHECK(!grouped->coredumpsEnabled());
}

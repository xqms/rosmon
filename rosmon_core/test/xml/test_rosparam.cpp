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

TEST_CASE("merge keys", "[rosparam]")
{
	LaunchConfig config;
	config.parseString(R"EOF(
		<launch>
<rosparam>
common: &amp;common
  param1: true
  param2: 5.0
  param3: hello
commonclass: &amp;commonclass
  testclass:
    &lt;&lt;: *common
    testparam: 140
class1:
  &lt;&lt;: *common
  param4: 514
class2:
  param1: false
  &lt;&lt;: *common
class3:
  &lt;&lt;: *common
  param1: false
class4:
  &lt;&lt;: *common
  param1: 5
  param2: test
  param3: 0.1
class5:
  &lt;&lt;: *commonclass
  param4: sub
class6:
  &lt;&lt;: *commonclass
  testclass:
    param2: 10.0
</rosparam>
		</launch>
	)EOF");

	config.evaluateParameters();

	CAPTURE(config.parameters());

	auto& params = config.parameters();

	// Class 1 test simple addition
	std::string class_ns = "/class1";
	checkTypedParam<bool>(params, class_ns + "/param1", XmlRpc::XmlRpcValue::TypeBoolean, true);
	checkTypedParam<double>(params, class_ns + "/param2", XmlRpc::XmlRpcValue::TypeDouble, 5.0);
	checkTypedParam<std::string>(params, class_ns + "/param3", XmlRpc::XmlRpcValue::TypeString, "hello");
	checkTypedParam<int>(params, class_ns + "/param4", XmlRpc::XmlRpcValue::TypeInt, 514);

	// Class 2 test override pre-assigned
	class_ns = "/class2";
	checkTypedParam<bool>(params, class_ns + "/param1", XmlRpc::XmlRpcValue::TypeBoolean, false);
	checkTypedParam<double>(params, class_ns + "/param2", XmlRpc::XmlRpcValue::TypeDouble, 5.0);
	checkTypedParam<std::string>(params, class_ns + "/param3", XmlRpc::XmlRpcValue::TypeString, "hello");

	// Class 3 test override post-assigned
	class_ns = "/class3";
	checkTypedParam<bool>(params, class_ns + "/param1", XmlRpc::XmlRpcValue::TypeBoolean, false);
	checkTypedParam<double>(params, class_ns + "/param2", XmlRpc::XmlRpcValue::TypeDouble, 5.0);
	checkTypedParam<std::string>(params, class_ns + "/param3", XmlRpc::XmlRpcValue::TypeString, "hello");

	// Class 4 test typechange overrides
	class_ns = "/class4";
	checkTypedParam<int>(params, class_ns + "/param1", XmlRpc::XmlRpcValue::TypeInt, 5);
	checkTypedParam<std::string>(params, class_ns + "/param2", XmlRpc::XmlRpcValue::TypeString, "test");
	checkTypedParam<double>(params, class_ns + "/param3", XmlRpc::XmlRpcValue::TypeDouble, 0.1);

	// Class 5 test nested merge
	class_ns = "/class5";
	checkTypedParam<bool>(params, class_ns + "/testclass/param1", XmlRpc::XmlRpcValue::TypeBoolean, true);
	checkTypedParam<double>(params, class_ns + "/testclass/param2", XmlRpc::XmlRpcValue::TypeDouble, 5.0);
	checkTypedParam<std::string>(params, class_ns + "/testclass/param3", XmlRpc::XmlRpcValue::TypeString, "hello");
	checkTypedParam<int>(params, class_ns + "/testclass/testparam", XmlRpc::XmlRpcValue::TypeInt, 140);
	checkTypedParam<std::string>(params, class_ns + "/param4", XmlRpc::XmlRpcValue::TypeString, "sub");

	// Class 6 test override on nested merge
	class_ns = "/class6";
	checkTypedParam<bool>(params, class_ns + "/testclass/param1", XmlRpc::XmlRpcValue::TypeBoolean, true);
	checkTypedParam<double>(params, class_ns + "/testclass/param2", XmlRpc::XmlRpcValue::TypeDouble, 10.0);
	checkTypedParam<std::string>(params, class_ns + "/testclass/param3", XmlRpc::XmlRpcValue::TypeString, "hello");
	checkTypedParam<int>(params, class_ns + "/testclass/testparam", XmlRpc::XmlRpcValue::TypeInt, 140);

}

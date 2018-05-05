
#include <catch_ros/catch.hpp>

#include "../../src/launch/launch_config.h"

using namespace rosmon::launch;

typedef std::map<std::string, XmlRpc::XmlRpcValue> ParameterMap;

namespace Catch
{
	template<>
	struct StringMaker<ParameterMap>
	{
		static std::string convert(const ParameterMap& value)
		{
			std::stringstream ss;
			ss << "{";
			for(auto& param : value)
			{
				ss << "\"" << param.first << "\"=XmlRpcValue, ";
			}
			ss << "}";

			return ss.str();
		}
	};

	template<>
	struct StringMaker<XmlRpc::XmlRpcValue>
	{
		static std::string convert(const XmlRpc::XmlRpcValue& value)
		{
			// We need a copy, since the cast operators don't work on const
			// refs *sigh*
			XmlRpc::XmlRpcValue copy = value;

			std::stringstream ss;
			ss << "XmlRpc";
			switch(value.getType())
			{
				case XmlRpc::XmlRpcValue::TypeInt:
					ss << "<int>(" << static_cast<int>(copy) << ")";
					break;
				case XmlRpc::XmlRpcValue::TypeBoolean:
					ss << "<bool>(" << static_cast<bool>(copy) << ")";
					break;
				case XmlRpc::XmlRpcValue::TypeString:
					ss << "<string>(" << static_cast<std::string>(copy) << ")";
					break;
				case XmlRpc::XmlRpcValue::TypeDouble:
					ss << "<double>(" << static_cast<double>(copy) << ")";
					break;
				default:
					ss << "<unknown>()";
					break;
			}

			return ss.str();
		}
	};
}

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

template<class T>
void checkTypedParam(const ParameterMap& parameters, const std::string& name, XmlRpc::XmlRpcValue::Type expectedType, T expected)
{
	CAPTURE(parameters);
	CAPTURE(name);

	auto it = parameters.find(name);
	REQUIRE(it != parameters.end());

	XmlRpc::XmlRpcValue value = it->second;

	REQUIRE(value.getType() == expectedType);

	CAPTURE(parameters);
	CAPTURE(name);

	REQUIRE(static_cast<T>(value) == expected);
}

TEST_CASE("param_types" "[param]")
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

			<param name="yaml_param" type="yaml" value="test_param: true" />
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

	checkTypedParam<bool>(params, "/yaml_param/test_param", XmlRpc::XmlRpcValue::TypeBoolean, true);
}

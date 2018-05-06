// Utilities for checking params
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef PARAM_UTILS_H
#define PARAM_UTILS_H

#include <catch_ros/catch.hpp>

#include <XmlRpcValue.h>

typedef std::map<std::string, XmlRpc::XmlRpcValue> ParameterMap;

namespace Catch
{
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
					ss << "<string>('" << static_cast<std::string>(copy) << "')";
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

	template<>
	struct StringMaker<ParameterMap>
	{
		static std::string convert(const ParameterMap& value)
		{
			std::stringstream ss;
			ss << "{";
			for(auto& param : value)
			{
				ss << "\"" << param.first << "\"=" << StringMaker<XmlRpc::XmlRpcValue>::convert(param.second) << ", ";
			}
			ss << "}";

			return ss.str();
		}
	};
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
	REQUIRE(static_cast<T>(value) == expected);
}

#endif
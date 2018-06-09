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
	struct StringMaker<XmlRpc::XmlRpcValue::Type>
	{
		static std::string convert(const XmlRpc::XmlRpcValue::Type& value)
		{
			switch(value)
			{
				case XmlRpc::XmlRpcValue::TypeInvalid: return "TypeInvalid";
				case XmlRpc::XmlRpcValue::TypeBoolean: return "TypeBoolean";
				case XmlRpc::XmlRpcValue::TypeInt: return "TypeInt";
				case XmlRpc::XmlRpcValue::TypeDouble: return "TypeDouble";
				case XmlRpc::XmlRpcValue::TypeString: return "TypeString";
				case XmlRpc::XmlRpcValue::TypeDateTime: return "TypeDateTime";
				case XmlRpc::XmlRpcValue::TypeBase64: return "TypeBase64";
				case XmlRpc::XmlRpcValue::TypeArray: return "TypeArray";
				case XmlRpc::XmlRpcValue::TypeStruct: return "TypeStruct";
				default: return "unknown type";
			}
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
					ss << "<string>('" << static_cast<std::string>(copy) << "')";
					break;
				case XmlRpc::XmlRpcValue::TypeDouble:
					ss << "<double>(" << static_cast<double>(copy) << ")";
					break;
				default:
					ss << "<"
						<< StringMaker<XmlRpc::XmlRpcValue::Type>::convert(value.getType())
						<< ">(?)";
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

	T typedValue = value;

	REQUIRE(typedValue == expected);
}

template<class T>
T getTypedParam(const ParameterMap& parameters, const std::string& name)
{
	CAPTURE(parameters);
	CAPTURE(name);

	auto it = parameters.find(name);
	REQUIRE(it != parameters.end());

	XmlRpc::XmlRpcValue value = it->second;

	T typedValue = value;

	return typedValue;
}

template<class T>
T getTypedParam(const ParameterMap& parameters, const std::string& name, XmlRpc::XmlRpcValue::Type expectedType)
{
	CAPTURE(parameters);
	CAPTURE(name);

	auto it = parameters.find(name);
	REQUIRE(it != parameters.end());

	XmlRpc::XmlRpcValue value = it->second;

	REQUIRE(value.getType() == expectedType);

	T typedValue = value;

	return typedValue;
}

#endif

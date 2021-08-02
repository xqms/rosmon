// Methods for converting YAML into XmlRpcValues for the parameter server
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "yaml_params.h"
#include "launch_config.h"
#include "substitution.h"
#include "substitution_python.h"

#include <boost/algorithm/string/predicate.hpp>

namespace rosmon
{
namespace launch
{

namespace
{
	class XmlRpcValueCreator : public XmlRpc::XmlRpcValue
	{
	public:
		static XmlRpcValueCreator createArray(const std::vector<XmlRpcValue>& values)
		{
			XmlRpcValueCreator ret;
			ret._type = TypeArray;
			ret._value.asArray = new ValueArray(values);

			return ret;
		}

		static XmlRpcValueCreator createStruct(const std::map<std::string, XmlRpcValue>& members)
		{
			XmlRpcValueCreator ret;
			ret._type = TypeStruct;
			ret._value.asStruct = new std::map<std::string, XmlRpcValue>(members);
			return ret;
		}
	};
}

XmlRpc::XmlRpcValue yamlToXmlRpc(const ParseContext& ctx, const YAML::Node& n)
{
	if(n.Type() != YAML::NodeType::Scalar)
	{
		switch(n.Type())
		{
			case YAML::NodeType::Map:
			{
				std::map<std::string, XmlRpc::XmlRpcValue> members;

				// Take care of merge keys first
				for(YAML::const_iterator it = n.begin(); it != n.end(); ++it)
				{
					auto key = it->first.as<std::string>();
					if(key == "<<")
					{
						auto merger = yamlToXmlRpc(ctx, it->second);
						if(merger.getType() != XmlRpc::XmlRpcValue::TypeStruct)
							throw std::runtime_error{"Expected a struct here"};

						for(auto& pair : merger)
							members[pair.first] = pair.second;
					}
				}

				// Now everything else
				for(YAML::const_iterator it = n.begin(); it != n.end(); ++it)
				{
					auto key = it->first.as<std::string>();
					if(key == "<<")
						continue;

					members[key] = yamlToXmlRpc(ctx, it->second);
				}

				return XmlRpcValueCreator::createStruct(members);
			}
			case YAML::NodeType::Sequence:
			{
				std::vector<XmlRpc::XmlRpcValue> values;
				for(YAML::const_iterator it = n.begin(); it != n.end(); ++it)
				{
					values.push_back(yamlToXmlRpc(ctx, *it));
				}
				return XmlRpcValueCreator::createArray(values);
			}
			default:
				throw ctx.error("Invalid YAML node type");
		}
	}

	// Scalars are tricky, as XmlRpcValue is strongly typed. So we need to
	// figure out the data type...

	// Check if a YAML tag is present
	if(n.Tag() == "tag:yaml.org,2002:int")
		return XmlRpc::XmlRpcValue(n.as<int>());
	if(n.Tag() == "tag:yaml.org,2002:float")
		return XmlRpc::XmlRpcValue(n.as<double>());
	if(n.Tag() == "tag:yaml.org,2002:bool")
		return XmlRpc::XmlRpcValue(n.as<bool>());
	if(n.Tag() == "tag:yaml.org,2002:str")
		return XmlRpc::XmlRpcValue(n.as<std::string>());
	if(n.Tag() == "tag:yaml.org,2002:binary")
	{
		auto binary = n.as<YAML::Binary>();

		// XmlRpc API is stupid here and expects a mutable void*, so to keep
		// things clean we do a copy.
		std::vector<unsigned char> copy(binary.data(), binary.data()+binary.size());

		return XmlRpc::XmlRpcValue(copy.data(), copy.size());
	}

	// rosparam supports !!degrees and !!radians...
	if(n.Tag() == "!degrees")
	{
		try
		{
			return XmlRpc::XmlRpcValue(evaluateROSParamPython(n.as<std::string>()) * M_PI / 180.0);
		}
		catch(SubstitutionException& e)
		{
			throw ctx.error("{}", e.what());
		}
	}
	if(n.Tag() == "!radians")
	{
		try
		{
			return XmlRpc::XmlRpcValue(evaluateROSParamPython(n.as<std::string>()));
		}
		catch(SubstitutionException& e)
		{
			throw ctx.error("{}", e.what());
		}
	}

	// If we have a "non-specific" tag '!', this means that the YAML scalar
	// is non-plain, i.e. of type seq, map, or str. Since seq and map are
	// handled above, we assume str in this case.
	if(n.Tag() == "!")
		return XmlRpc::XmlRpcValue(n.as<std::string>());

	// Otherwise, we simply have to try things one by one...
	try { return XmlRpc::XmlRpcValue(n.as<bool>()); }
	catch(YAML::Exception&) {}

	try { return XmlRpc::XmlRpcValue(n.as<int>()); }
	catch(YAML::Exception&) {}

	try { return XmlRpc::XmlRpcValue(n.as<double>()); }
	catch(YAML::Exception&) {}

	try
	{
		std::string str = n.as<std::string>();

		// rosparam supports deg(...) and rad(...) expressions, so we have to
		// parse them here.

		if(boost::starts_with(str, "deg(") && boost::ends_with(str, ")"))
		{
			try
			{
				return XmlRpc::XmlRpcValue(
					evaluateROSParamPython(str.substr(4, str.size() - 5)) * M_PI / 180.0
				);
			}
			catch(SubstitutionException& e)
			{
				throw ctx.error("{}", e.what());
			}
		}

		if(boost::starts_with(str, "rad(") && boost::ends_with(str, ")"))
		{
			try
			{
				return XmlRpc::XmlRpcValue(evaluateROSParamPython(str.substr(4, str.size() - 5)));
			}
			catch(SubstitutionException& e)
			{
				throw ctx.error("{}", e.what());
			}
		}

		return XmlRpc::XmlRpcValue(str);
	}
	catch(YAML::Exception&) {}

	throw ctx.error("Could not convert YAML value");
}

}
}

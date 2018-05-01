// Aggregates all information needed to start and monitor nodes
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSMON_LAUNCH_LAUNCH_CONFIG_H
#define ROSMON_LAUNCH_LAUNCH_CONFIG_H

#include "node.h"

#include <map>
#include <vector>
#include <stdexcept>
#include <future>
#include <random>

#include <XmlRpc.h>
#include <tinyxml.h>
#include <yaml-cpp/yaml.h>

namespace rosmon
{
namespace launch
{

class LaunchConfig;

extern const char* UNSET_MARKER;

class ParseContext
{
public:
	ParseContext(LaunchConfig* config)
		: m_config(config)
		, m_prefix("/")
	{}

	const std::string& prefix() const
	{ return m_prefix; }

	const std::string& filename() const
	{ return m_filename; }

	void setFilename(const std::string& filename)
	{ m_filename = filename; }

	ParseContext enterScope(const std::string& prefix)
	{
		ParseContext ret = *this;
		ret.m_prefix = ret.m_prefix + prefix;
		if(prefix[prefix.size()-1] != '/')
			ret.m_prefix.push_back('/');

		return ret;
	}

	std::string evaluate(const std::string& tpl);

	bool parseBool(const std::string& input, int line);

	void clearArguments()
	{
		m_args.clear();
	}

	inline const std::map<std::string, std::string>& arguments() const
	{ return m_args; }

	void setArg(const std::string& name, const std::string& argument, bool override);

	void setEnvironment(const std::string& name, const std::string& value);

	inline const std::map<std::string, std::string> environment() const
	{ return m_environment; }

	bool shouldSkip(TiXmlElement* element);

	inline LaunchConfig* config()
	{ return m_config; }
private:
	LaunchConfig* m_config;

	std::string m_prefix;
	std::string m_filename;
	std::map<std::string, std::string> m_args;
	std::map<std::string, std::string> m_environment;
};

class LaunchConfig
{
public:
	typedef std::shared_ptr<LaunchConfig> Ptr;
	typedef std::shared_ptr<const LaunchConfig> ConstPtr;

	class ParseException : public std::exception
	{
	public:
		explicit ParseException(const std::string& msg)
		 : m_msg(msg)
		{}

		virtual ~ParseException() throw()
		{}

		virtual const char* what() const noexcept
		{ return m_msg.c_str(); }
	private:
		std::string m_msg;
	};

	LaunchConfig();

	void setArgument(const std::string& name, const std::string& value);

	void parse(const std::string& filename, bool onlyArguments = false);

	void evaluateParameters();

	inline const std::map<std::string, XmlRpc::XmlRpcValue>& parameters() const
	{ return m_params; }

	inline const std::vector<Node::Ptr>& nodes() const
	{ return m_nodes; }

	inline const std::map<std::string, std::string>& arguments() const
	{ return m_rootContext.arguments(); }

	std::string anonName(const std::string& base);

	std::string rosmonNodeName() const
	{ return m_rosmonNodeName; }

	std::string windowTitle() const
	{ return m_windowTitle; }
private:
	void parse(TiXmlElement* element, ParseContext* context, bool onlyArguments = false);
	void parseNode(TiXmlElement* element, ParseContext context);
	void parseParam(TiXmlElement* element, ParseContext context);
	void parseROSParam(TiXmlElement* element, ParseContext context);
	void parseInclude(TiXmlElement* element, ParseContext context);
	void parseArgument(TiXmlElement* element, ParseContext& context);
	void parseEnv(TiXmlElement* element, ParseContext& context);

	void loadYAMLParams(const YAML::Node& n, const std::string& prefix);

	XmlRpc::XmlRpcValue yamlToXmlRpc(const YAML::Node& n);

	ParseContext m_rootContext;

	std::vector<Node::Ptr> m_nodes;
	typedef std::future<XmlRpc::XmlRpcValue> ParameterFuture;
	std::map<std::string, XmlRpc::XmlRpcValue> m_params;
	std::map<std::string, ParameterFuture> m_paramJobs;

	std::map<std::string, std::string> m_anonNames;
	std::mt19937_64 m_anonGen;

	std::string m_rosmonNodeName;

	std::string m_windowTitle;
};

}
}

#endif


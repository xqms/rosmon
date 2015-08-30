// Aggregates all information needed to start and monitor nodes
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LAUNCH_CONFIG_H
#define LAUNCH_CONFIG_H

#include "node.h"
#include "fd_watcher.h"

#include <map>
#include <vector>
#include <stdexcept>
#include <future>

#include <XmlRpc.h>
#include <tinyxml.h>
#include <yaml-cpp/yaml.h>

#include <ros/node_handle.h>

#include <boost/signals2.hpp>

namespace rosmon
{

class LaunchConfig
{
public:
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

	explicit LaunchConfig(const FDWatcher::Ptr& watcher);
	~LaunchConfig();

	void setArgument(const std::string& name, const std::string& value);

	void parse(const std::string& filename);

	void setParameters();
	void start();
	void shutdown();
	void forceExit();
	bool allShutdown();

	inline const std::vector<Node::Ptr>& nodes() const
	{ return m_nodes; }

	inline bool ok() const
	{ return m_ok; }

	std::string anonName(const std::string& base);

	boost::signals2::signal<void(std::string,std::string)> logMessageSignal;
private:
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

	void parse(TiXmlElement* element, ParseContext context);
	void parseNode(TiXmlElement* element, ParseContext context);
	void parseParam(TiXmlElement* element, ParseContext context);
	void parseROSParam(TiXmlElement* element, ParseContext context);
	void parseInclude(TiXmlElement* element, ParseContext context);
	void parseArgument(TiXmlElement* element, ParseContext& context);
	void parseEnv(TiXmlElement* element, ParseContext& context);

	void loadYAMLParams(const YAML::Node& n, const std::string& prefix);

	bool parseBool(const std::string& value);

	XmlRpc::XmlRpcValue yamlToXmlRpc(const YAML::Node& n);

	void log(const char* fmt, ...) __attribute__((format (printf, 2, 3)));

	void handleRequiredNodeExit(const std::string& name);

	ros::NodeHandle m_nh;
	FDWatcher::Ptr m_fdWatcher;

	ParseContext m_rootContext;

	std::vector<Node::Ptr> m_nodes;
	typedef std::future<XmlRpc::XmlRpcValue> ParameterFuture;
	std::map<std::string, XmlRpc::XmlRpcValue> m_params;
	std::map<std::string, ParameterFuture> m_paramJobs;

	bool m_ok;

	std::map<std::string, std::string> m_anonNames;
	std::mt19937_64 m_anonGen;
};

}

#endif


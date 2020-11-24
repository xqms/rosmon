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

#include <fmt/format.h>

namespace rosmon
{
namespace launch
{

class LaunchConfig;

constexpr double DEFAULT_CPU_LIMIT = 0.9f;
constexpr uint64_t DEFAULT_MEMORY_LIMIT = 500*1024*1024;
constexpr double DEFAULT_STOP_TIMEOUT = 5.0f;

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

	template<typename... Args>
	ParseException format(const char* format, const Args& ... args)
	{
		return ParseException(fmt::format(format, args...));
	}
private:
	std::string m_msg;
};

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

	void setCurrentElement(TiXmlElement* e)
	{
		// NOTE: We should not keep a reference to the TiXmlElement here,
		// since the ParseContext might be around longer than the DOM tree.
		// See evaluateParameters().
		if(e)
			m_currentLine = e->Row();
		else
			m_currentLine = -1;
	}

	ParseContext enterScope(const std::string& prefix);
	void parseScopeAttributes(TiXmlElement* e, ParseContext& attr_ctx);

	std::string evaluate(const std::string& tpl, bool simplifyWhitespace = true);

	bool parseBool(const std::string& value, int line);

	void clearArguments()
	{
		m_args.clear();
	}

	inline const std::map<std::string, std::string>& arguments() const
	{ return m_args; }

	void setArg(const std::string& name, const std::string& value, bool override);

	void setEnvironment(const std::string& name, const std::string& value);

	inline const std::map<std::string, std::string> environment() const
	{ return m_environment; }

	bool shouldSkip(TiXmlElement* e);

	inline LaunchConfig* config()
	{ return m_config; }

	void setRemap(const std::string& from, const std::string& to);
	const std::map<std::string, std::string>& remappings()
	{ return m_remappings; }

	std::string anonName(const std::string& base);

	template<typename... Args>
	ParseException error(const char* fmt, const Args& ... args) const
	{
		std::string msg = fmt::format(fmt, args...);

		if(m_currentLine >= 0)
		{
			return ParseException(fmt::format("{}:{}: {}",
				m_filename, m_currentLine, msg
			));
		}
		else
		{
			return ParseException(fmt::format("{}: {}", m_filename, msg));
		}
	}

	template<typename... Args>
	void warning(const char* fmt, const Args& ... args) const;


	double cpuLimit() const
	{ return m_cpuLimit; }
	void setCPULimit(double limit)
	{ m_cpuLimit = limit; }

	uint64_t memoryLimit() const
	{ return m_memoryLimit; }
	void setMemoryLimit(uint64_t limit)
	{ m_memoryLimit = limit; }

	double stopTimeout() const
	{ return m_stopTimeout; }
	void setStopTimeout(double timeout)
	{ m_stopTimeout = timeout; }

	bool coredumpsEnabled() const
	{ return m_coredumpsEnabled; }
	void setCoredumpsEnabled(bool enabled)
	{ m_coredumpsEnabled = enabled; }

private:
	LaunchConfig* m_config;

	std::string m_prefix;
	std::string m_filename;
	int m_currentLine = -1;
	std::map<std::string, std::string> m_args;
	std::map<std::string, std::string> m_environment;
	std::map<std::string, std::string> m_remappings;
	std::map<std::string, std::string> m_anonNames;

	double m_cpuLimit = DEFAULT_CPU_LIMIT;
	uint64_t m_memoryLimit = DEFAULT_MEMORY_LIMIT;
	double m_stopTimeout = DEFAULT_STOP_TIMEOUT;
	bool m_coredumpsEnabled = true;
};

class LaunchConfig
{
public:
	typedef std::shared_ptr<LaunchConfig> Ptr;
	typedef std::shared_ptr<const LaunchConfig> ConstPtr;

	LaunchConfig();

	enum class OutputAttr
	{
		Obey,
		Ignore
	};

	void setWarningOutput(std::ostream* warningStream);
	std::ostream& warningOutput()
	{ return *m_warningOutput; }

	void setArgument(const std::string& name, const std::string& value);

	void setDefaultStopTimeout(double timeout);
	void setDefaultCPULimit(double CPULimit);
	void setDefaultMemoryLimit(uint64_t memoryLimit);

	void setOutputAttrMode(OutputAttr mode);

	void parse(const std::string& filename, bool onlyArguments = false);
	void parseString(const std::string& input, bool onlyArguments = false);

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

	std::string generateAnonHash();

	bool disableUI() const
	{ return m_disableUI; }
private:
	enum ParamContext
	{
		PARAM_GENERAL, //!< <param> tag inside <node>
		PARAM_IN_NODE, //!< <param> tag everywhere else
	};

	void parseTopLevelAttributes(TiXmlElement* element);

	void parse(TiXmlElement* element, ParseContext* ctx, bool onlyArguments = false);
	void parseNode(TiXmlElement* element, ParseContext& ctx);
	void parseParam(TiXmlElement* element, ParseContext& ctx, ParamContext paramContext = PARAM_GENERAL);
	void parseROSParam(TiXmlElement* element, ParseContext& ctx);
	void parseInclude(TiXmlElement* element, ParseContext ctx);
	void parseArgument(TiXmlElement* element, ParseContext& ctx);
	void parseEnv(TiXmlElement* element, ParseContext& ctx);
	void parseRemap(TiXmlElement* element, ParseContext& ctx);

	void loadYAMLParams(const ParseContext& ctx, const YAML::Node& n, const std::string& prefix);

	XmlRpc::XmlRpcValue paramToXmlRpc(const ParseContext& ctx, const std::string& value, const std::string& type = "");

	ParseContext m_rootContext;

	std::vector<Node::Ptr> m_nodes;

	using ParameterList = std::map<std::string, XmlRpc::XmlRpcValue>;
	using ParameterFuture = std::future<XmlRpc::XmlRpcValue>;

	struct YAMLResult
	{
		std::string name;
		YAML::Node yaml;
	};

	ParameterList m_params;
	std::map<std::string, ParameterFuture> m_paramJobs;
	std::vector<std::future<YAMLResult>> m_yamlParamJobs;

	std::mt19937_64 m_anonGen;

	std::string m_rosmonNodeName;

	std::string m_windowTitle;

	OutputAttr m_outputAttrMode{OutputAttr::Ignore};

	bool m_disableUI = false;

	std::ostream* m_warningOutput = &std::cerr;
};

template<typename... Args>
void ParseContext::warning(const char* fmt, const Args& ... args) const
{
	std::string msg = fmt::format(fmt, args...);

	if(m_currentLine >= 0)
	{
		m_config->warningOutput() << fmt::format(
			"{}:{}: Warning: {}\n", m_filename, m_currentLine, msg
		);
	}
	else
	{
		m_config->warningOutput() << fmt::format(
			"{}: Warning: {}\n", m_filename, msg
		);
	}
}

}
}

#endif


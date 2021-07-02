// Aggregates all information needed to start and monitor nodes
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "launch_config.h"
#include "substitution.h"
#include "yaml_params.h"
#include "bytes_parser.h"
#include "string_utils.h"

#include <ros/package.h>
#include <ros/names.h>

#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <fstream>

#include <sys/wait.h>

#include <boost/regex.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>

#include <yaml-cpp/yaml.h>

namespace rosmon
{
namespace launch
{

const char* UNSET_MARKER = "~~~~~ ROSMON-UNSET ~~~~~";

ParseContext ParseContext::enterScope(const std::string& prefix)
{
	ParseContext ret = *this;
	ret.m_prefix = ros::names::clean(ret.m_prefix + prefix) + "/";

	return ret;
}

void ParseContext::parseScopeAttributes(TiXmlElement* e, ParseContext& attr_ctx)
{
	if(const char* stopTimeout = e->Attribute("rosmon-stop-timeout"))
	{
		double seconds;
		try
		{
			seconds = boost::lexical_cast<double>(attr_ctx.evaluate(stopTimeout));
		}
		catch(boost::bad_lexical_cast&)
		{
			throw error("bad rosmon-stop-timeout value '{}'", stopTimeout);
		}
		if(seconds < 0)
			throw error("negative rosmon-stop-timeout value '{}'", stopTimeout);

		m_stopTimeout = seconds;
	}

	if(const char* memoryLimit = e->Attribute("rosmon-memory-limit"))
	{
		uint64_t memoryLimitByte;
		bool ok;
		std::tie(memoryLimitByte, ok) = parseMemory(static_cast<std::string>(memoryLimit));
		if(!ok)
		{
			throw error("{} cannot be parsed as a memory limit", memoryLimit);
		}

		m_memoryLimit = memoryLimitByte;
	}

	if(const char* cpuLimit = e->Attribute("rosmon-cpu-limit"))
	{
		double cpuLimitPct;
		try
		{
			cpuLimitPct = boost::lexical_cast<double>(attr_ctx.evaluate(cpuLimit));
		}
		catch(boost::bad_lexical_cast&)
		{
			throw error("bad rosmon-cpu-limit value '{}'", cpuLimit);
		}

		if(cpuLimitPct < 0)
			throw error("negative rosmon-cpu-limit value'{}'", cpuLimit);

		m_cpuLimit = cpuLimitPct;
	}

	if(const char* coredumpsEnabled = e->Attribute("enable-coredumps"))
	{
		m_coredumpsEnabled = attr_ctx.parseBool(coredumpsEnabled, e->Row());
	}
}

std::string ParseContext::evaluate(const std::string& tpl, bool simplifyWhitespace)
{
	std::string simplified;
	if(simplifyWhitespace)
		simplified = string_utils::simplifyWhitespace(tpl);
	else
		simplified = tpl;

	try
	{
		return parseSubstitutionArgs(simplified, *this);
	}
	catch(SubstitutionException& e)
	{
		throw error("Substitution error: {}", e.what());
	}
}

bool ParseContext::parseBool(const std::string& value, int line)
{
	std::string expansion = evaluate(value);

	// We are lenient here and accept the pythonic forms "True" and "False"
	// as well, since roslaunch seems to do the same. Even the roslaunch/XML
	// spec mentions True/False in the examples, even though they are not
	// valid options for if/unless and other boolean attributes...
	// http://wiki.ros.org/roslaunch/XML/rosparam

	if(expansion == "1" || expansion == "true" || expansion == "True")
		return true;

	if(expansion == "0" || expansion == "false" || expansion == "False")
		return false;

	throw error("Unknown truth value '%s'", expansion.c_str());
}

bool ParseContext::shouldSkip(TiXmlElement* e)
{
	const char* if_cond = e->Attribute("if");
	const char* unless_cond = e->Attribute("unless");

	if(if_cond && unless_cond)
	{
		throw error("both if= and unless= specified, don't know what to do");
	}

	if(if_cond)
	{
		return !parseBool(if_cond, e->Row());
	}

	if(unless_cond)
	{
		return parseBool(unless_cond, e->Row());
	}

	return false;
}

void ParseContext::setArg(const std::string& name, const std::string& value, bool override)
{
	auto it = m_args.find(name);
	if(it == m_args.end())
		m_args[name] = value;
	else if(override || it->second == UNSET_MARKER)
		m_args[name] = value;
}

void ParseContext::setEnvironment(const std::string& name, const std::string& value)
{
	m_environment[name] = value;
}

void ParseContext::setRemap(const std::string& from, const std::string& to)
{
	m_remappings[from] = to;
}

std::string ParseContext::anonName(const std::string& base)
{
	auto it = m_anonNames.find(base);
	if(it == m_anonNames.end())
	{
		auto name = base + "_" + m_config->generateAnonHash();

		it = m_anonNames.emplace(base, name).first;
	}

	return it->second;
}


LaunchConfig::LaunchConfig()
 : m_rootContext(this)
 , m_anonGen(std::random_device()())
{
	const char* ROS_NAMESPACE = getenv("ROS_NAMESPACE");
	if(ROS_NAMESPACE)
	{
		// Someone set ROS_NAMESPACE, we should respect it.
		// This may happen in nested situations, e.g. rosmon launching rosmon.
		m_rootContext = m_rootContext.enterScope(ROS_NAMESPACE);
	}
}

void LaunchConfig::setArgument(const std::string& name, const std::string& value)
{
	m_rootContext.setArg(name, value, true);
}

void LaunchConfig::setDefaultStopTimeout(double timeout)
{
	m_rootContext.setStopTimeout(timeout);
}

void LaunchConfig::setDefaultCPULimit(double CPULimit)
{
	m_rootContext.setCPULimit(CPULimit);
}

void LaunchConfig::setDefaultMemoryLimit(uint64_t memoryLimit)
{
	m_rootContext.setMemoryLimit(memoryLimit);
}

void LaunchConfig::parse(const std::string& filename, bool onlyArguments)
{
	m_rootContext.setFilename(filename);

	TiXmlDocument document(filename);

	TiXmlBase::SetCondenseWhiteSpace(false);

	if(!document.LoadFile())
	{
		throw m_rootContext.error("Could not load launch file: {}", document.ErrorDesc());
	}

	ros::WallTime start = ros::WallTime::now();
	parse(document.RootElement(), &m_rootContext, onlyArguments);

	// Parse top-level rosmon-specific attributes
	parseTopLevelAttributes(document.RootElement());

	if(!onlyArguments)
		fmt::print("Loaded launch file in {:f}s\n", (ros::WallTime::now() - start).toSec());
}

void LaunchConfig::parseString(const std::string& input, bool onlyArguments)
{
	m_rootContext.setFilename("[string]");

	TiXmlDocument document;

	TiXmlBase::SetCondenseWhiteSpace(false);

	document.Parse(input.c_str());

	if(document.Error())
	{
		throw m_rootContext.error("Could not parse string input: {}", document.ErrorDesc());
	}

	ros::WallTime start = ros::WallTime::now();
	parse(document.RootElement(), &m_rootContext, onlyArguments);

	// Parse top-level rosmon-specific attributes
	parseTopLevelAttributes(document.RootElement());

	if(!onlyArguments)
		fmt::print("Loaded launch file in {:f}s\n", (ros::WallTime::now() - start).toSec());
}

void LaunchConfig::parseTopLevelAttributes(TiXmlElement* element)
{
	const char* name = element->Attribute("rosmon-name");
	if(name)
		m_rosmonNodeName = name;

	const char* windowTitle = element->Attribute("rosmon-window-title");
	if(windowTitle)
		m_windowTitle = windowTitle;

	const char* disableUI = element->Attribute("rosmon-disable-ui");
	if(disableUI)
		m_disableUI = m_rootContext.parseBool(disableUI, element->Row());
}

void LaunchConfig::parse(TiXmlElement* element, ParseContext* ctx, bool onlyArguments)
{
	ctx->parseScopeAttributes(element, *ctx);

	// First pass: Parse arguments
	for(TiXmlNode* n = element->FirstChild(); n; n = n->NextSibling())
	{
		TiXmlElement* e = n->ToElement();
		if(!e)
			continue;

		if(ctx->shouldSkip(e))
			continue;

		ctx->setCurrentElement(e);

		if(e->ValueStr() == "arg")
			parseArgument(e, *ctx);
	}

	if(onlyArguments)
		return;

	// Second pass: everything else
	for(TiXmlNode* n = element->FirstChild(); n; n = n->NextSibling())
	{
		TiXmlElement* e = n->ToElement();
		if(!e)
			continue;

		if(ctx->shouldSkip(e))
			continue;

		ctx->setCurrentElement(e);

		if(e->ValueStr() == "node")
			parseNode(e, *ctx);
		else if(e->ValueStr() == "param")
			parseParam(e, *ctx);
		else if(e->ValueStr() == "rosparam")
			parseROSParam(e, *ctx);
		else if(e->ValueStr() == "group")
		{
			ParseContext cctx = *ctx;

			if(const char* ns = e->Attribute("ns"))
				cctx = cctx.enterScope(ctx->evaluate(ns));

			cctx.parseScopeAttributes(e, *ctx);

			parse(e, &cctx);
		}
		else if(e->ValueStr() == "include")
			parseInclude(e, *ctx);
		else if(e->ValueStr() == "env")
			parseEnv(e, *ctx);
		else if(e->ValueStr() == "remap")
			parseRemap(e, *ctx);
	}
}

void LaunchConfig::parseNode(TiXmlElement* element, ParseContext& attr_ctx)
{
	const char* name = element->Attribute("name");
	const char* pkg = element->Attribute("pkg");
	const char* type = element->Attribute("type");
	const char* args = element->Attribute("args");
	const char* ns = element->Attribute("ns");
	const char* respawn = element->Attribute("respawn");
	const char* respawnDelay = element->Attribute("respawn_delay");
	const char* nRespawnsAllowed = element->Attribute("rosmon-restart-warn-threshold");
	const char* required = element->Attribute("required");
	const char* launchPrefix = element->Attribute("launch-prefix");
	const char* cwd = element->Attribute("cwd");
	const char* clearParams = element->Attribute("clear_params");
	const char* output = element->Attribute("output");

	if(!name || !pkg || !type)
	{
		throw attr_ctx.error("name, pkg, type are mandatory for node elements!");
	}

	// Attributes are evaluated *outside* of the node context, so keep that one
	ParseContext ctx = attr_ctx;

	if(ns)
		ctx = ctx.enterScope(ctx.evaluate(ns));

	std::string fullNamespace = ctx.prefix().substr(0, ctx.prefix().length()-1);

	// Enter scope
	ctx = ctx.enterScope(ctx.evaluate(name));

	// Parse scoped attributes such as rosmon-stop-timeout
	ctx.parseScopeAttributes(element, attr_ctx);

	Node::Ptr node = std::make_shared<Node>(
		attr_ctx.evaluate(name), attr_ctx.evaluate(pkg), attr_ctx.evaluate(type)
	);

	// Check name uniqueness
	{
		auto it = std::find_if(m_nodes.begin(), m_nodes.end(), [&](const Node::Ptr& n) {
			return n->namespaceString() == fullNamespace && n->name() == node->name();
		});

		if(it != m_nodes.end())
		{
			throw ctx.error("node name '{}' is not unique", node->name());
		}
	}

	node->setStopTimeout(ctx.stopTimeout());
	node->setMemoryLimit(ctx.memoryLimit());
	node->setCPULimit(ctx.cpuLimit());
	node->setCoredumpsEnabled(ctx.coredumpsEnabled());

	if(args)
		node->addExtraArguments(ctx.evaluate(args));

	if(!fullNamespace.empty())
		node->setNamespace(fullNamespace);

	if(respawn)
	{
		node->setRespawn(attr_ctx.parseBool(respawn, element->Row()));

		if(respawnDelay)
		{
			double seconds;
			try
			{
				seconds = boost::lexical_cast<double>(attr_ctx.evaluate(respawnDelay));
			}
			catch(boost::bad_lexical_cast&)
			{
				throw ctx.error("bad respawn_delay value '{}'", respawnDelay);
			}

			node->setRespawnDelay(ros::WallDuration(seconds));
		}

		if (nRespawnsAllowed)
		{
			int n_respawns;
			try
			{
				n_respawns = boost::lexical_cast<int>(attr_ctx.evaluate(nRespawnsAllowed));
			}
			catch(boost::bad_lexical_cast&)
			{
				throw ctx.error("bad rosmon-restart-warn-threshold value '{}'", nRespawnsAllowed);
			}

			node->setNumRespawnsAllowed(n_respawns);
		}
	}

	if(required && attr_ctx.parseBool(required, element->Row()))
	{
		node->setRequired(true);
	}

	// We have to parse rosparam tags first, see #118
	for(TiXmlNode* n = element->FirstChild(); n; n = n->NextSibling())
	{
		TiXmlElement* e = n->ToElement();
		if(!e)
			continue;

		if(ctx.shouldSkip(e))
			continue;

		ctx.setCurrentElement(e);

		if(e->ValueStr() == "rosparam")
			parseROSParam(e, ctx);
	}

	// Now we can parse everything else.
	for(TiXmlNode* n = element->FirstChild(); n; n = n->NextSibling())
	{
		TiXmlElement* e = n->ToElement();
		if(!e)
			continue;

		if(ctx.shouldSkip(e))
			continue;

		ctx.setCurrentElement(e);

		if(e->ValueStr() == "param")
			parseParam(e, ctx, PARAM_IN_NODE);
		else if(e->ValueStr() == "remap")
			parseRemap(e, ctx);
		else if(e->ValueStr() == "env")
			parseEnv(e, ctx);
	}

	ctx.setCurrentElement(element);

	// Set environment *after* parsing the node children (may contain env tags)
	node->setExtraEnvironment(ctx.environment());

	if(launchPrefix)
		node->setLaunchPrefix(attr_ctx.evaluate(launchPrefix));

	if(cwd)
		node->setWorkingDirectory(attr_ctx.evaluate(cwd));

	if(clearParams)
		node->setClearParams(attr_ctx.parseBool(clearParams, element->Row()));

	if(m_outputAttrMode == OutputAttr::Obey)
	{
		node->setStdoutDisplayed(false); // output=log is default
	}

	if(output)
	{
		std::string outputStr = attr_ctx.evaluate(output);
		if(outputStr == "screen")
			node->setStdoutDisplayed(true);
		else if(outputStr == "log")
			node->setStdoutDisplayed(false);
		else
			throw ctx.error("Invalid output attribute value: '{}'", outputStr);
	}

	node->setRemappings(ctx.remappings());

	m_nodes.push_back(node);
}

static XmlRpc::XmlRpcValue autoXmlRpcValue(const std::string& fullValue)
{
	std::string fullValueLowercase = boost::algorithm::to_lower_copy(fullValue);
	if(fullValueLowercase == "true")
		return XmlRpc::XmlRpcValue(true);
	else if(fullValueLowercase == "false")
		return XmlRpc::XmlRpcValue(false);
	else
	{
		try { return boost::lexical_cast<int>(fullValue); }
		catch(boost::bad_lexical_cast&) {}

		try { return boost::lexical_cast<double>(fullValue); }
		catch(boost::bad_lexical_cast&) {}

		return fullValue;
	}
}

void LaunchConfig::parseParam(TiXmlElement* element, ParseContext& ctx, ParamContext paramContext)
{
	const char* name = element->Attribute("name");
	const char* value = element->Attribute("value");
	const char* command = element->Attribute("command");
	const char* textfile = element->Attribute("textfile");
	const char* binfile = element->Attribute("binfile");
	const char* type = element->Attribute("type");

	if(!name)
	{
		throw ctx.error("name is mandatory for param elements");
	}

	int numCommands = (value ? 1 : 0) + (command ? 1 : 0) + (textfile ? 1 : 0) + (binfile ? 1 : 0);
	if(numCommands > 1) // == 0 is checked below, don't duplicate.
	{
		throw ctx.error("<param> tags need exactly one of value=, command=, textfile=, binfile= attributes.");
	}

	std::string fullName = ctx.evaluate(name);
	if(fullName.empty())
	{
		throw ctx.error("param name is empty");
	}

	// Expand relative paths. roslaunch ignores leading / when inside a
	// <node> tag - god only knows why.
	if(fullName[0] != '/' || paramContext == PARAM_IN_NODE)
	{
		// Same with "/" (see above)
		if(paramContext == PARAM_IN_NODE && fullName[0] == '/')
		{
			ctx.warning("leading slashes in <param> names are ignored inside <node> contexts for roslaunch compatibility.");
			fullName = fullName.substr(1);
		}
		else if(fullName[0] == '~')
		{
			// We silently ignore "~" at the beginning of the name
			fullName = fullName.substr(1);
		}

		fullName = ctx.prefix() + fullName;
	}

	std::string errorStr;
	if(!ros::names::validate(fullName, errorStr))
	{
		throw ctx.error("Expanded parameter name '{}' is invalid: {}",
			fullName, errorStr
		);
	}

	std::string fullType;
	if(type)
		fullType = ctx.evaluate(type);

	if(value)
	{
		// Simple case - immediate value.
		if(fullType == "yaml")
		{
			std::string fullValue = ctx.evaluate(value, false);

			YAML::Node n;
			try
			{
				n = YAML::Load(fullValue);
			}
			catch(YAML::ParserException& e)
			{
				throw ctx.error("Invalid YAML: {}", e.what());
			}

			loadYAMLParams(ctx, n, fullName);
		}
		else
		{
			// Note: roslaunch strips leading/trailing whitespace of all simple
			// parameters. Furthermore, line feeds and tabs are replaced with
			// space characters.
			m_params[fullName] = paramToXmlRpc(ctx,
				string_utils::convertWhitespace(
					string_utils::strip(ctx.evaluate(value, false))
				),
				fullType
			);

			// A dynamic parameter of the same name gets overwritten now
			m_paramJobs.erase(fullName);
		}

		return;
	}

	if(binfile)
	{
		// Also simple - binary files are always mapped to base64 XmlRpcValue.

		std::string fullFile = ctx.evaluate(binfile);

		m_paramJobs[fullName] = std::async(std::launch::deferred,
			[=]() -> XmlRpc::XmlRpcValue {
				std::ifstream stream(fullFile, std::ios::binary | std::ios::ate);
				if(!stream)
					throw ctx.error("Could not open file '{}'", fullFile);

				std::vector<char> data(stream.tellg(), 0);
				stream.seekg(0, std::ios::beg);

				stream.read(data.data(), data.size());

				// Creates base64 XmlRpcValue
				return {data.data(), static_cast<int>(data.size())};
			}
		);

		m_params.erase(fullName);
		return;
	}

	// Dynamic parameters are more complicated. We split the computation in
	// two parts:
	//  1) Compute the value as std::string
	//  2) Convert to desired type (or dissect into multiple parameters in
	//     case of YAML-typed parameters).

	auto computeString = std::make_shared<std::future<std::string>>();

	// On ROS < Lunar, type="..." is not respected for command= and textfile=
	// attributes. We support that anyway, but print a nice warning for users.
	// See GH issue #138.

#if !ROS_VERSION_MINIMUM(1,13,0)
	if(type)
	{
		ctx.warning(
			"On ROS versions prior to Lunar, roslaunch does not respect the "
			"type attribute on <param> tags with command= or textfile= "
			"actions. However, rosmon does support it and will create the "
			"properly typed parameter {}.",
			fullName
		);
	}
#endif

	if(command)
	{
		// Run a command and retrieve the results.
		std::string fullCommand = ctx.evaluate(command);

		// Commands may take a while - that is why we use std::async here.
		*computeString = std::async(std::launch::deferred,
			[=]() -> std::string {
				std::stringstream buffer;

				int pipe_fd[2];
				if(pipe(pipe_fd) != 0)
					throw ctx.error("Could not create pipe: {}", strerror(errno));

				int pid = fork();
				if(pid < 0)
					throw ctx.error("Could not fork: {}", strerror(errno));

				if(pid == 0)
				{
					// Child
					close(pipe_fd[0]);
					if(pipe_fd[1] != STDOUT_FILENO)
					{
						dup2(pipe_fd[1], STDOUT_FILENO);
						close(pipe_fd[1]);
					}

					char* argp[] = {strdup("sh"), strdup("-c"), strdup(fullCommand.c_str()), nullptr};

					execvp("sh", argp); // should never return
					throw ctx.error("Could not execvp '{}': {}", fullCommand, strerror(errno));
				}

				close(pipe_fd[1]);

				timeval timeout;
				memset(&timeout, 0, sizeof(timeout));
				timeout.tv_sec = 0;
				timeout.tv_usec = 500 * 1000;

				while(true)
				{
					fd_set fds;
					FD_ZERO(&fds);
					FD_SET(pipe_fd[0], &fds);

					int ret = select(pipe_fd[0]+1, &fds, nullptr, nullptr, &timeout);
					if(ret < 0)
						throw ctx.error("Could not select(): {}", strerror(errno));

					if(ret == 0)
					{
						fmt::print("Still loading parameter '{}'...\n", fullName);

						timeout.tv_sec = 3;
						continue;
					}

					char buf[1024];
					ret = read(pipe_fd[0], buf, sizeof(buf)-1);
					if(ret < 0)
						throw ctx.error("Could not read: {}", strerror(errno));
					if(ret == 0)
						break;

					buf[ret] = 0;
					buffer << buf;
				}

				close(pipe_fd[0]);

				int status = 0;
				if(waitpid(pid, &status, 0) < 0)
					throw ctx.error("Could not waitpid(): {}", strerror(errno));

				if(!WIFEXITED(status) || WEXITSTATUS(status) != 0)
				{
					throw ctx.error("<param> command failed (exit status {})",
						WEXITSTATUS(status)
					);
				}

				return buffer.str();
			}
		);

		// A fixed parameter of the same name gets overwritten now
		m_params.erase(fullName);
	}
	else if(textfile)
	{
		std::string fullFile = ctx.evaluate(textfile);

		*computeString = std::async(std::launch::deferred,
			[=]() -> std::string {
				std::ifstream stream(fullFile);
				if(!stream)
					throw ctx.error("Could not open file '{}'", fullFile);

				std::stringstream buffer;
				buffer << stream.rdbuf();

				return buffer.str();
			}
		);
	}
	else
	{
		throw ctx.error("<param> needs either command, value, binfile, or textfile");
	}

	if(fullType == "yaml")
	{
		m_yamlParamJobs.push_back(std::async(std::launch::deferred,
			[=]() -> YAMLResult {
				std::string yamlString = computeString->get();

				YAML::Node n;
				try
				{
					n = YAML::Load(yamlString);
				}
				catch(YAML::ParserException& e)
				{
					throw ctx.error("Read invalid YAML from process or file: {}",
						e.what()
					);
				}

				return {fullName, n};
			}
		));
	}
	else
	{
		m_paramJobs[fullName] = std::async(std::launch::deferred,
			[=]() -> XmlRpc::XmlRpcValue {
				return paramToXmlRpc(ctx, computeString->get(), fullType);
			}
		);

		// A fixed parameter of the same name gets overwritten now
		m_params.erase(fullName);
	}
}

XmlRpc::XmlRpcValue LaunchConfig::paramToXmlRpc(const ParseContext& ctx, const std::string& value, const std::string& type)
{
	if(type.empty())
		return autoXmlRpcValue(value);

	// Fixed type.
	try
	{
		if(type == "int")
			return boost::lexical_cast<int>(string_utils::strip(value));
		else if(type == "double")
			return boost::lexical_cast<double>(string_utils::strip(value));
		else if(type == "bool" || type == "boolean")
		{
			std::string value_lowercase = boost::algorithm::to_lower_copy(
				string_utils::strip(value)
			);

			if(value_lowercase == "true")
				return true;
			else if(value_lowercase == "false")
				return false;
			else
				throw ctx.error("invalid boolean value '{}'", value);
		}
		else if(type == "str" || type == "string")
			return value;
		else
		{
			throw ctx.error("invalid param type '{}'", type);
		}
	}
	catch(boost::bad_lexical_cast& e)
	{
		throw ctx.error("could not convert param value '{}' to type '{}'",
			value, type
		);
	}
}

void LaunchConfig::parseROSParam(TiXmlElement* element, ParseContext& ctx)
{
	const char* command = element->Attribute("command");

	if(!command || strcmp(command, "load") == 0)
	{
		const char* file = element->Attribute("file");
		std::string fullFile;

		std::string contents;
		if(file)
		{
			fullFile = ctx.evaluate(file);
			std::ifstream stream(fullFile);
			if(!stream)
				throw ctx.error("Could not open file '{}'", fullFile);

			std::stringstream buffer;
			buffer << stream.rdbuf();

			contents = buffer.str();
		}
		else
		{
			if(const char* t = element->GetText())
				contents = t;
		}

		// roslaunch silently ignores empty files (which are not valid YAML),
		// so do the same here.
		if(string_utils::isOnlyWhitespace(contents))
			return;

		const char* subst_value = element->Attribute("subst_value");
		if(subst_value && ctx.parseBool(subst_value, element->Row()))
			contents = ctx.evaluate(contents, false);

		YAML::Node n;
		try
		{
			n = YAML::Load(contents);
		}
		catch(YAML::ParserException& e)
		{
			throw ctx.error("Could not parse YAML: {}", e.what());
		}

		ParseContext nameContext = ctx;
		const char* ns = element->Attribute("ns");
		if(ns)
			nameContext = nameContext.enterScope(ctx.evaluate(ns));

		const char* name = element->Attribute("param");
		if(name)
			nameContext = nameContext.enterScope(ctx.evaluate(name));

		// Remove trailing / from prefix to get param name
		try
		{
			loadYAMLParams(ctx, n, nameContext.prefix().substr(0, nameContext.prefix().length()-1));
		}
		catch(ParseException& e)
		{
			if(file)
			{
				throw ctx.error("error while parsing rosparam input file {}: {}",
					fullFile, e.what()
				);
			}
			else
			{
				throw ctx.error("error while parsing YAML input from launch file: {}",
					e.what()
				);
			}
		}
	}
	else
		throw ctx.error("Unsupported rosparam command '{}'", command);
}

void LaunchConfig::loadYAMLParams(const ParseContext& ctx, const YAML::Node& n, const std::string& prefix)
{
	switch(n.Type())
	{
		case YAML::NodeType::Map:
		{
			// Pass 1: Load any anchor references
			for(YAML::const_iterator it = n.begin(); it != n.end(); ++it)
			{
				if(it->first.as<std::string>() == "<<")
				{
					loadYAMLParams(ctx, it->second, prefix);
				}
			}

			// Pass 2: Everything else.
			for(YAML::const_iterator it = n.begin(); it != n.end(); ++it)
			{
				auto key = it->first.as<std::string>();
				if(key != "<<")
				{
					// Load "global" params without prefix (see #130)
					if(!key.empty() && key[0] == '/')
						loadYAMLParams(ctx, it->second, key);
					else
						loadYAMLParams(ctx, it->second, prefix + "/" + it->first.as<std::string>());
				}
			}

			break;
		}
		case YAML::NodeType::Sequence:
		case YAML::NodeType::Scalar:
		{
			m_params[prefix] = yamlToXmlRpc(ctx, n);

			// A dynamic parameter of the same name gets overwritten now
			m_paramJobs.erase(prefix);
			break;
		}
		case YAML::NodeType::Null:
		{
			// Nothing to do, empty node
			break;
		}
		default:
		{
			throw ctx.error("invalid yaml node type");
		}
	}
}

void LaunchConfig::parseInclude(TiXmlElement* element, ParseContext ctx)
{
	const char* file = element->Attribute("file");
	const char* ns = element->Attribute("ns");
	const char* passAllArgs = element->Attribute("pass_all_args");
	const char* clearParams = element->Attribute("clear_params");

	if(!file)
		throw ctx.error("<include> file attribute is mandatory");

	if(clearParams && ctx.parseBool(clearParams, element->Row()))
	{
		throw ctx.error("<include clear_params=\"true\" /> is not supported and probably a bad idea.");
	}

	std::string fullFile = ctx.evaluate(file);

	ParseContext childCtx = ctx;
	if(ns)
		childCtx = childCtx.enterScope(ctx.evaluate(ns));

	// Parse any arguments
	childCtx.parseScopeAttributes(element, ctx);

	// If pass_all_args is not set, delete the current arguments.
	if(!passAllArgs || !ctx.parseBool(passAllArgs, element->Row()))
		childCtx.clearArguments();

	// Now set all explicitly mentioned arguments.
	for(TiXmlNode* n = element->FirstChild(); n; n = n->NextSibling())
	{
		TiXmlElement* e = n->ToElement();
		if(!e)
			continue;

		if(ctx.shouldSkip(e))
			continue;

		if(e->ValueStr() == "arg")
		{
			const char* name = e->Attribute("name");
			const char* value = e->Attribute("value");
			const char* defaultValue = e->Attribute("default");

			if(!name)
				throw ctx.error("<arg> inside include needs a name attribute");

			if(!value && defaultValue)
			{
				// roslaunch allows this - god knows why.
				ctx.warning(
					"You are using <arg> inside an <include> tag with the "
					"default=XY attribute - which is superfluous. "
					"Use value=XY instead for less confusion. "
					"Attribute name: {}",
					name
				);
				value = defaultValue;
			}

			if(!name || !value)
				throw ctx.error("<arg> inside include needs name and value");

			childCtx.setArg(ctx.evaluate(name), ctx.evaluate(value), true);
		}
	}

	TiXmlDocument document(fullFile);
	if(!document.LoadFile())
		throw ctx.error("Could not load launch file '{}': {}", fullFile, document.ErrorDesc());

	childCtx.setFilename(fullFile);

	parse(document.RootElement(), &childCtx);
}

void LaunchConfig::parseArgument(TiXmlElement* element, ParseContext& ctx)
{
	const char* name = element->Attribute("name");
	const char* value = element->Attribute("value");
	const char* def = element->Attribute("default");

	if(!name)
		throw ctx.error("<arg> needs name attribute");

	if(value)
	{
		std::string fullValue = ctx.evaluate(value);
		ctx.setArg(name, fullValue, true);
	}
	else if(def)
	{
		std::string fullValue = ctx.evaluate(def);
		ctx.setArg(name, fullValue, false);
	}
	else
	{
		ctx.setArg(name, UNSET_MARKER, false);
	}
}

void LaunchConfig::parseEnv(TiXmlElement* element, ParseContext& ctx)
{
	const char* name = element->Attribute("name");
	const char* value = element->Attribute("value");

	if(!name || !value)
		throw ctx.error("<env> needs name, value attributes");

	ctx.setEnvironment(ctx.evaluate(name), ctx.evaluate(value));
}

void LaunchConfig::parseRemap(TiXmlElement* element, ParseContext& ctx)
{
	const char* from = element->Attribute("from");
	const char* to = element->Attribute("to");

	if(!from || !to)
		throw ctx.error("remap needs 'from' and 'to' arguments");

	ctx.setRemap(ctx.evaluate(from), ctx.evaluate(to));
}

std::string LaunchConfig::generateAnonHash()
{
	return fmt::format("{:08X}", m_anonGen());
}

template<class Iterator>
void safeAdvance(Iterator& it, const Iterator& end, size_t i)
{
	for(size_t j = 0; j < i; ++j)
	{
		if(it == end)
			return;
		++it;
	}
}

void LaunchConfig::evaluateParameters()
{
	// This function is optimized for speed, since we usually have a lot of
	// parameters and some of those take time (>2s) to compute and upload
	// (e.g. xacro).

	// The optimization is two-fold: we use std::future to avoid useless
	// computations of parameters which are re-set later on anyways, and
	// we use a thread pool to evaluate the futures below.

	const int NUM_THREADS = std::thread::hardware_concurrency();

	std::vector<std::thread> threads(NUM_THREADS);

	std::mutex mutex;

	bool caughtExceptionFlag = false;
	ParseException caughtException("");

	for(int i = 0; i < NUM_THREADS; ++i)
	{
		threads[i] = std::thread([this,i,NUM_THREADS,&mutex,&caughtException,&caughtExceptionFlag]() {
			try
			{
				// Thread number i starts at position i and moves in NUM_THREADS
				// increments.
				auto it = m_paramJobs.begin();
				safeAdvance(it, m_paramJobs.end(), i);

				while(it != m_paramJobs.end())
				{
					XmlRpc::XmlRpcValue val = it->second.get();
					{
						std::lock_guard<std::mutex> guard(mutex);
						m_params[it->first] = val;
					}
					safeAdvance(it, m_paramJobs.end(), NUM_THREADS);
				}

				// YAML parameters have to be handled separately, since we may need
				// to splice them into individual XmlRpcValues.
				auto yamlIt = m_yamlParamJobs.begin();
				safeAdvance(yamlIt, m_yamlParamJobs.end(), i);

				while(yamlIt != m_yamlParamJobs.end())
				{
					YAMLResult yaml = yamlIt->get();
					{
						std::lock_guard<std::mutex> guard(mutex);
						loadYAMLParams(m_rootContext, yaml.yaml, yaml.name);
					}
					safeAdvance(yamlIt, m_yamlParamJobs.end(), NUM_THREADS);
				}
			}
			catch(ParseException& e)
			{
				std::lock_guard<std::mutex> guard(mutex);
				caughtException = e;
				caughtExceptionFlag = true;
			}
		});
	}

	// and wait for completion for the threads.
	for(auto& t : threads)
		t.join();

	if(caughtExceptionFlag)
		throw caughtException;

	m_paramJobs.clear();
}

void LaunchConfig::setOutputAttrMode(OutputAttr mode)
{
	m_outputAttrMode = mode;
}

void LaunchConfig::setWarningOutput(std::ostream* output)
{
	m_warningOutput = output;
}

}
}

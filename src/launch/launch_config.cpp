// Aggregates all information needed to start and monitor nodes
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "launch_config.h"
#include "substitution.h"

#include <ros/package.h>
#include <ros/names.h>

#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <fstream>

#include <sys/wait.h>

#include <boost/regex.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>

#include <yaml-cpp/yaml.h>

namespace rosmon
{
namespace launch
{

const char* UNSET_MARKER = "~~~~~ ROSMON-UNSET ~~~~~";

/**
 * @brief Compress any sequence of whitespace to single spaces.
 *
 * Since we switch of space condensing in TinyXML to be able to properly parse
 * <rosparam> tags, this function can be used for attributes.
 *
 * roslaunch also strips whitespace at begin/end, so we do that as well.
 **/
static std::string simplifyWhitespace(const std::string& input)
{
	std::string output;
	output.reserve(input.size());

	// Skip initial space
	size_t i = 0;
	for(; i < input.size(); ++i)
	{
		if(!std::isspace(static_cast<unsigned char>(input[i])))
			break;
	}

	bool in_space = false;

	for(; i < input.size(); ++i)
	{
		char c = input[i];

		if(std::isspace(static_cast<unsigned char>(c)))
			in_space = true;
		else
		{
			if(in_space)
				output.push_back(' ');

			output.push_back(c);
			in_space = false;
		}
	}

	return output;
}

/**
 * @brief Check if string is whitespace only (includes '\n')
 **/
static bool isOnlyWhitespace(const std::string& input)
{
	for(const char& c: input)
	{
		// see http://en.cppreference.com/w/cpp/string/byte/isspace
		// for reason for casting
		if(!std::isspace(static_cast<unsigned char>(c)))
			return false;
	}

	return true;
}

std::string ParseContext::evaluate(const std::string& tpl, bool simplifyWhitespace)
{
	std::string simplified;
	if(simplifyWhitespace)
		simplified = rosmon::launch::simplifyWhitespace(tpl);
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

LaunchConfig::LaunchConfig()
 : m_rootContext(this)
 , m_anonGen(std::random_device()())
{
}

void LaunchConfig::setArgument(const std::string& name, const std::string& value)
{
	m_rootContext.setArg(name, value, true);
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
}

void LaunchConfig::parse(TiXmlElement* element, ParseContext* ctx, bool onlyArguments)
{
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
			const char* ns = e->Attribute("ns");

			ParseContext cctx = *ctx;

			if(ns)
				cctx = cctx.enterScope(ctx->evaluate(ns));

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

void LaunchConfig::parseNode(TiXmlElement* element, ParseContext ctx)
{
	const char* name = element->Attribute("name");
	const char* pkg = element->Attribute("pkg");
	const char* type = element->Attribute("type");
	const char* args = element->Attribute("args");
	const char* ns = element->Attribute("ns");
	const char* respawn = element->Attribute("respawn");
	const char* respawnDelay = element->Attribute("respawn_delay");
	const char* required = element->Attribute("required");
	const char* launchPrefix = element->Attribute("launch-prefix");
	const char* coredumpsEnabled = element->Attribute("enable-coredumps");
	const char* cwd = element->Attribute("cwd");
	const char* clearParams = element->Attribute("clear_params");

	if(!name || !pkg || !type)
	{
		throw ctx.error("name, pkg, type are mandatory for node elements!");
	}

	if(ns)
		ctx = ctx.enterScope(ctx.evaluate(ns));

	std::string fullNamespace = ctx.prefix().substr(0, ctx.prefix().length()-1);

	// Enter scope
	ctx = ctx.enterScope(ctx.evaluate(name));

	Node::Ptr node = std::make_shared<Node>(
		ctx.evaluate(name), ctx.evaluate(pkg), ctx.evaluate(type)
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

	if(args)
		node->addExtraArguments(ctx.evaluate(args));

	if(!fullNamespace.empty())
		node->setNamespace(fullNamespace);

	if(respawn)
	{
		node->setRespawn(ctx.parseBool(respawn, element->Row()));

		if(respawnDelay)
		{
			double seconds;
			try
			{
				seconds = boost::lexical_cast<double>(ctx.evaluate(respawnDelay));
			}
			catch(boost::bad_lexical_cast&)
			{
				throw ctx.error("bad respawn_delay value '{}'", respawnDelay);
			}

			node->setRespawnDelay(ros::WallDuration(seconds));
		}
	}

	if(required && ctx.parseBool(required, element->Row()))
	{
		node->setRequired(true);
	}

	for(TiXmlNode* n = element->FirstChild(); n; n = n->NextSibling())
	{
		TiXmlElement* e = n->ToElement();
		if(!e)
			continue;

		if(ctx.shouldSkip(e))
			continue;

		ctx.setCurrentElement(e);

		if(e->ValueStr() == "param")
			parseParam(e, ctx);
		else if(e->ValueStr() == "rosparam")
			parseROSParam(e, ctx);
		else if(e->ValueStr() == "remap")
			parseRemap(e, ctx);
		else if(e->ValueStr() == "env")
			parseEnv(e, ctx);
	}

	ctx.setCurrentElement(element);

	// Set environment *after* parsing the node children (may contain env tags)
	node->setExtraEnvironment(ctx.environment());

	if(launchPrefix)
		node->setLaunchPrefix(ctx.evaluate(launchPrefix));

	if(coredumpsEnabled)
		node->setCoredumpsEnabled(ctx.parseBool(coredumpsEnabled, element->Row()));

	if(cwd)
		node->setWorkingDirectory(ctx.evaluate(cwd));

	if(clearParams)
		node->setClearParams(ctx.parseBool(clearParams, element->Row()));

	node->setRemappings(ctx.remappings());

	m_nodes.push_back(node);
}

static XmlRpc::XmlRpcValue autoXmlRpcValue(const std::string& fullValue)
{
	if(fullValue == "true")
		return XmlRpc::XmlRpcValue(true);
	else if(fullValue == "false")
		return XmlRpc::XmlRpcValue(false);
	else
	{
		try { return boost::lexical_cast<int>(fullValue); }
		catch(boost::bad_lexical_cast&) {}

		try { return boost::lexical_cast<float>(fullValue); }
		catch(boost::bad_lexical_cast&) {}

		return fullValue;
	}
}

void LaunchConfig::parseParam(TiXmlElement* element, ParseContext ctx)
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

	// Expand relative paths
	if(fullName[0] != '/')
	{
		// We silently ignore "~" at the beginning of the name
		if(fullName[0] == '~')
			fullName = fullName.substr(1);

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
			m_params[fullName] = paramToXmlRpc(ctx, ctx.evaluate(value), fullType);

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
				if(stream.bad())
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
				if(stream.bad())
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
			return boost::lexical_cast<int>(value);
		else if(type == "double")
			return boost::lexical_cast<double>(value);
		else if(type == "bool" || type == "boolean")
		{
			if(value == "true")
				return true;
			else if(value == "false")
				return false;
			else
			{
				throw ctx.error("invalid boolean value '{}'", value);
			}
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

void LaunchConfig::parseROSParam(TiXmlElement* element, ParseContext ctx)
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
			if(stream.bad())
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
		if(isOnlyWhitespace(contents))
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

		const char* ns = element->Attribute("ns");
		if(ns)
			ctx = ctx.enterScope(ctx.evaluate(ns));

		const char* name = element->Attribute("param");
		if(name)
			ctx = ctx.enterScope(ctx.evaluate(name));

		// Remove trailing / from prefix to get param name
		try
		{
			loadYAMLParams(ctx, n, ctx.prefix().substr(0, ctx.prefix().length()-1));
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


XmlRpc::XmlRpcValue LaunchConfig::yamlToXmlRpc(const ParseContext& ctx, const YAML::Node& n)
{
	if(n.Type() != YAML::NodeType::Scalar)
	{
		switch(n.Type())
		{
			case YAML::NodeType::Map:
			{
				std::map<std::string, XmlRpc::XmlRpcValue> members;
				for(YAML::const_iterator it = n.begin(); it != n.end(); ++it)
				{
					members[it->first.as<std::string>()] = yamlToXmlRpc(ctx, it->second);
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
	if(n.Tag() == "!!int")
		return XmlRpc::XmlRpcValue(n.as<int>());
	if(n.Tag() == "!!float")
		return XmlRpc::XmlRpcValue(n.as<double>());
	if(n.Tag() == "!!bool")
		return XmlRpc::XmlRpcValue(n.as<bool>());

	// Otherwise, we simply have to try things one by one...
	try { return XmlRpc::XmlRpcValue(n.as<bool>()); }
	catch(YAML::Exception&) {}

	try { return XmlRpc::XmlRpcValue(n.as<int>()); }
	catch(YAML::Exception&) {}

	try { return XmlRpc::XmlRpcValue(n.as<double>()); }
	catch(YAML::Exception&) {}

	try { return XmlRpc::XmlRpcValue(n.as<std::string>()); }
	catch(YAML::Exception&) {}

	throw ctx.error("Could not convert YAML value");
}

void LaunchConfig::loadYAMLParams(const ParseContext& ctx, const YAML::Node& n, const std::string& prefix)
{
	switch(n.Type())
	{
		case YAML::NodeType::Map:
		{
			for(YAML::const_iterator it = n.begin(); it != n.end(); ++it)
			{
				loadYAMLParams(ctx, it->second, prefix + "/" + it->first.as<std::string>());
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

std::string LaunchConfig::anonName(const std::string& base)
{
	auto it = m_anonNames.find(base);
	if(it == m_anonNames.end())
	{
		uint32_t r = m_anonGen();

		char buf[20];
		snprintf(buf, sizeof(buf), "%08X", r);

		auto name = base + "_" + buf;

		it = m_anonNames.emplace(base, name).first;
	}

	return it->second;
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

}
}

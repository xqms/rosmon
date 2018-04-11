// Aggregates all information needed to start and monitor nodes
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "launch_config.h"
#include "substitution.h"

#include <ros/package.h>

#include <fstream>

#include <stdarg.h>
#include <stdio.h>

#include <boost/regex.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>

#include <yaml-cpp/yaml.h>

namespace rosmon
{
namespace launch
{

const char* UNSET_MARKER = "~~~~~ ROSMON-UNSET ~~~~~";

static LaunchConfig::ParseException error(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	char str[1024];

	vsnprintf(str, sizeof(str), fmt, args);

	va_end(args);

	return LaunchConfig::ParseException(str);
}

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
		if(!isspace(i))
			break;
	}

	bool in_space = false;

	for(; i < input.size(); ++i)
	{
		char c = input[i];

		if(isspace(c))
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

std::string ParseContext::evaluate(const std::string& tpl)
{
	try
	{
		return parseSubstitutionArgs(tpl, *this);
	}
	catch(SubstitutionException& e)
	{
		throw error("%s: %s", filename().c_str(), e.what());
	}
}

bool ParseContext::parseBool(const std::string& value, int line)
{
	std::string expansion = evaluate(value);

	if(expansion == "1" || expansion == "true")
		return true;
	else if(expansion == "0" || expansion == "false")
		return false;
	else
		throw error("%s:%d: Unknown truth value '%s'", filename().c_str(), line, expansion.c_str());
}

bool ParseContext::shouldSkip(TiXmlElement* e)
{
	const char* if_cond = e->Attribute("if");
	if(if_cond)
	{
		if(parseBool(if_cond, e->Row()))
			return false;
		else
			return true;
	}

	const char* unless_cond = e->Attribute("unless");
	if(unless_cond)
	{
		if(parseBool(unless_cond, e->Row()))
			return true;
		else
			return false;
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

LaunchConfig::LaunchConfig()
 : m_rootContext(this)
 , m_anonGen(std::random_device()())
{
}

LaunchConfig::~LaunchConfig()
{
}

void LaunchConfig::setArgument(const std::string& name, const std::string& value)
{
	m_rootContext.setArg(name, value, true);
}

void LaunchConfig::parse(const std::string& filename, bool onlyArguments)
{
	TiXmlDocument document(filename);

	TiXmlBase::SetCondenseWhiteSpace(false);

	if(!document.LoadFile())
	{
		throw error("Could not load launch file '%s'. Exiting...\n", filename.c_str());
	}

	ros::WallTime start = ros::WallTime::now();
	m_rootContext.setFilename(filename);
	parse(document.RootElement(), &m_rootContext, onlyArguments);

	// Parse top-level rosmon-specific attributes
	const char* name = document.RootElement()->Attribute("rosmon-name");
	if(name)
		m_rosmonNodeName = name;

	const char* windowTitle = document.RootElement()->Attribute("rosmon-window-title");
	if(windowTitle)
		m_windowTitle = windowTitle;

	if(!onlyArguments)
		printf("Loaded launch file in %fs\n", (ros::WallTime::now() - start).toSec());
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

	if(!name || !pkg || !type)
	{
		throw error("File %s:%d: name, pkg, type are mandatory for node elements!\n",
			ctx.filename().c_str(), element->Row()
		);
	}

	if(ns)
		ctx = ctx.enterScope(ctx.evaluate(ns));

	std::string fullNamespace = ctx.prefix().substr(0, ctx.prefix().length()-1);

	// Enter scope
	ctx = ctx.enterScope(ctx.evaluate(name));

	Node::Ptr node = std::make_shared<Node>(
		ctx.evaluate(name), ctx.evaluate(pkg), ctx.evaluate(type)
	);

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
				throw error("%s:%d: bad respawn_delay value", ctx.filename().c_str(), element->Row());
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

		if(e->ValueStr() == "param")
			parseParam(e, ctx);
		else if(e->ValueStr() == "rosparam")
			parseROSParam(e, ctx);
		else if(e->ValueStr() == "remap")
		{
			const char* from = e->Attribute("from");
			const char* to = e->Attribute("to");

			if(!from || !to)
				throw error("%s:%d: remap needs 'from' and 'to' arguments", ctx.filename().c_str(), e->Row());

			node->addRemapping(ctx.evaluate(from), ctx.evaluate(to));
		}
		else if(e->ValueStr() == "env")
			parseEnv(e, ctx);
	}

	// Set environment *after* parsing the node children (may contain env tags)
	node->setExtraEnvironment(ctx.environment());

	if(launchPrefix)
		node->setLaunchPrefix(ctx.evaluate(launchPrefix));

	if(coredumpsEnabled)
		node->setCoredumpsEnabled(ctx.parseBool(coredumpsEnabled, element->Row()));

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

	if(!name)
	{
		throw error("File %s:%d: name and value are mandatory for param elements\n",
			ctx.filename().c_str(), element->Row()
		);
	}

	std::string fullName;
	if(name[0] == '/')
		fullName = ctx.evaluate(name);
	else
		fullName = ctx.prefix() + ctx.evaluate(name);

	if(value)
	{
		const char* type = element->Attribute("type");
		std::string fullValue = ctx.evaluate(simplifyWhitespace(value));

		XmlRpc::XmlRpcValue result;

		if(type)
		{
			// Fixed type.

			try
			{
				if(strcmp(type, "int") == 0)
					result = boost::lexical_cast<int>(fullValue);
				else if(strcmp(type, "double") == 0)
					result = boost::lexical_cast<double>(fullValue);
				else if(strcmp(type, "bool") == 0 || strcmp(type, "boolean") == 0)
				{
					if(fullValue == "true")
					{
						// tricky: there is no operator= for bool, see #3
						result = XmlRpc::XmlRpcValue(true);
					}
					else if(fullValue == "false")
						result = XmlRpc::XmlRpcValue(false);
					else
					{
						throw error("%s:%d: invalid boolean value '%s'",
							ctx.filename().c_str(), element->Row(), fullValue.c_str()
						);
					}
				}
				else if(strcmp(type, "str") == 0 || strcmp(type, "string") == 0)
					result = fullValue;
				else
				{
					throw error("%s:%d: invalid param type '%s'",
						ctx.filename().c_str(), element->Row(), type
					);
				}
			}
			catch(boost::bad_lexical_cast& e)
			{
				throw error("%s:%d: could not convert param value '%s' to type '%s'",
					ctx.filename().c_str(), element->Row(), fullValue.c_str(), type
				);
			}
		}
		else
		{
			// Try to determine the type automatically.
			result = autoXmlRpcValue(fullValue);
		}

		m_params[fullName] = result;

		// A dynamic parameter of the same name gets overwritten now
		m_paramJobs.erase(fullName);
	}
	else if(command)
	{
		std::string fullCommand = ctx.evaluate(command);

		// Commands may take a while - that is why we use std::async here.
		m_paramJobs[fullName] = std::async(std::launch::deferred,
			[=]() -> XmlRpc::XmlRpcValue {
				std::stringstream buffer;

				int pipe_fd[2];
				if(pipe(pipe_fd) != 0)
					throw error("Could not create pipe: %s", strerror(errno));

				int pid = fork();
				if(pid < 0)
					throw error("Could not fork: %s", strerror(errno));

				if(pid == 0)
				{
					// Child
					close(pipe_fd[0]);
					if(pipe_fd[1] != STDOUT_FILENO)
					{
						dup2(pipe_fd[1], STDOUT_FILENO);
						close(pipe_fd[1]);
					}

					char* argp[] = {strdup("sh"), strdup("-c"), strdup(fullCommand.c_str()), NULL};

					execvp("sh", argp); // should never return
					throw error("Could not execvp '%s': %s", fullCommand.c_str(), strerror(errno));
				}

				close(pipe_fd[1]);

				timeval timeout;
				timeout.tv_sec = 0;
				timeout.tv_usec = 500 * 1000;

				while(true)
				{
					fd_set fds;
					FD_ZERO(&fds);
					FD_SET(pipe_fd[0], &fds);

					int ret = select(pipe_fd[0]+1, &fds, 0, 0, &timeout);
					if(ret < 0)
						throw error("Could not select(): %s", strerror(errno));

					if(ret == 0)
					{
						printf("Still loading parameter '%s'...\n", fullName.c_str());

						timeout.tv_sec = 3;
						continue;
					}

					char buf[1024];
					ret = read(pipe_fd[0], buf, sizeof(buf)-1);
					if(ret < 0)
						throw error("Could not read: %s", strerror(errno));
					if(ret == 0)
						break;

					buf[ret] = 0;
					buffer << buf;
				}

				close(pipe_fd[0]);

				return buffer.str();
			}
		);

		// A fixed parameter of the same name gets overwritten now
		m_params.erase(fullName);
	}
	else if(textfile)
	{
		std::string fullFile = ctx.evaluate(textfile);

		m_paramJobs[fullName] = std::async(std::launch::deferred,
			[=]() -> XmlRpc::XmlRpcValue {
				std::ifstream stream(fullFile);
				if(stream.bad())
					throw error("%s:%d: Could not open file '%s'", ctx.filename().c_str(), element->Row(), fullFile.c_str());

				std::stringstream buffer;
				buffer << stream.rdbuf();

				return buffer.str();
			}
		);

		// A fixed parameter of the same name gets overwritten now
		m_params.erase(fullName);
	}
	else
	{
		throw error("%s:%d: <param> needs either command, value or textfile",
			ctx.filename().c_str(), element->Row()
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
				throw error("%s:%d: Could not open file '%s'", ctx.filename().c_str(), element->Row(), fullFile.c_str());

			std::stringstream buffer;
			buffer << stream.rdbuf();

			contents = buffer.str();
		}
		else
			contents = element->GetText();

		const char* subst_value = element->Attribute("subst_value");
		if(subst_value && strcmp(subst_value, "true") == 0)
			contents = ctx.evaluate(contents);

		YAML::Node n = YAML::Load(contents);

		const char* ns = element->Attribute("ns");
		if(ns)
			ctx = ctx.enterScope(ctx.evaluate(ns));

		const char* name = element->Attribute("param");
		if(name)
			ctx = ctx.enterScope(ctx.evaluate(name));

		// Remove trailing / from prefix to get param name
		try
		{
			loadYAMLParams(n, ctx.prefix().substr(0, ctx.prefix().length()-1));
		}
		catch(ParseException& e)
		{
			std::stringstream ss;
			ss << ctx.filename() << ": ";
			if(file)
				ss << "Error while parsing rosparam input file '" << fullFile << "': ";
			else
				ss << "Error while parsing YAML input from launch file: ";

			ss << e.what();
			throw ParseException(ss.str());
		}
	}
	else
		throw error("Unsupported rosparam command '%s'", command);
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


XmlRpc::XmlRpcValue LaunchConfig::yamlToXmlRpc(const YAML::Node& n)
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
					members[it->first.as<std::string>()] = yamlToXmlRpc(it->second);
				}
				return XmlRpcValueCreator::createStruct(members);
			}
			case YAML::NodeType::Sequence:
			{
				std::vector<XmlRpc::XmlRpcValue> values;
				for(YAML::const_iterator it = n.begin(); it != n.end(); ++it)
				{
					values.push_back(yamlToXmlRpc(*it));
				}
				return XmlRpcValueCreator::createArray(values);
			}
			default:
				throw error("Invalid YAML node type");
		}
	}

	// Scalars are tricky, as XmlRpcValue is strongly typed. So we need to
	// figure out the data type...

	// Check if a YAML tag is present
	if(n.Tag() == "!!int")
		return XmlRpc::XmlRpcValue(n.as<int>());
	else if(n.Tag() == "!!float")
		return XmlRpc::XmlRpcValue(n.as<double>());
	else if(n.Tag() == "!!bool")
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

	throw error("Could not convert YAML value");
}

void LaunchConfig::loadYAMLParams(const YAML::Node& n, const std::string& prefix)
{
	switch(n.Type())
	{
		case YAML::NodeType::Map:
		{
			for(YAML::const_iterator it = n.begin(); it != n.end(); ++it)
			{
				loadYAMLParams(it->second, prefix + "/" + it->first.as<std::string>());
			}
			break;
		}
		case YAML::NodeType::Sequence:
		case YAML::NodeType::Scalar:
		{
			m_params[prefix] = yamlToXmlRpc(n);

			// A dynamic parameter of the same name gets overwritten now
			m_paramJobs.erase(prefix);
			break;
		}
		default:
		{
			throw error("invalid yaml node type");
		}
	}
}

void LaunchConfig::parseInclude(TiXmlElement* element, ParseContext ctx)
{
	const char* file = element->Attribute("file");
	const char* ns = element->Attribute("ns");

	if(!file)
		throw error("%s:%d: file attribute is mandatory", ctx.filename().c_str(), element->Row());

	std::string fullFile = ctx.evaluate(file);

	ParseContext childCtx = ctx;
	if(ns)
		childCtx = childCtx.enterScope(ctx.evaluate(ns));

	// Parse any arguments
	childCtx.clearArguments();

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
				throw error("%s:%d: <arg> inside include needs name and value", ctx.filename().c_str(), element->Row());

			childCtx.setArg(ctx.evaluate(name), ctx.evaluate(value), true);
		}
	}

	TiXmlDocument document(fullFile);
	if(!document.LoadFile())
		throw error("Could not load launch file '%s'. Exiting...\n", fullFile.c_str());

	childCtx.setFilename(fullFile);

	parse(document.RootElement(), &childCtx);
}

void LaunchConfig::parseArgument(TiXmlElement* element, ParseContext& ctx)
{
	const char* name = element->Attribute("name");
	const char* value = element->Attribute("value");
	const char* def = element->Attribute("default");

	if(!name)
		throw error("%s:%d: <arg> needs name attribute", ctx.filename().c_str(), element->Row());

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
		throw error("%s:%d: <env> needs name, value attributes", ctx.filename().c_str(), element->Row());

	ctx.setEnvironment(ctx.evaluate(name), ctx.evaluate(value));
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

	for(int i = 0; i < NUM_THREADS; ++i)
	{
		threads[i] = std::thread([this,i,NUM_THREADS,&mutex]() {
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
		});
	}

	// and wait for completion for the threads.
	for(auto& t : threads)
		t.join();

	m_paramJobs.clear();
}

}
}

// Aggregates all information needed to start and monitor nodes
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "launch_config.h"
#include "package_registry.h"

#include <ros/package.h>
#include <ros/node_handle.h>

#include <fstream>

#include <stdarg.h>
#include <stdio.h>

#include <boost/regex.hpp>

#include <yaml-cpp/yaml.h>

static const char* UNSET_MARKER = "~~~~~ ROSMON-UNSET ~~~~~";

namespace rosmon
{

static LaunchConfig::ParseException error(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	char str[1024];

	vsnprintf(str, sizeof(str), fmt, args);

	va_end(args);

	return LaunchConfig::ParseException(str);
}

std::string LaunchConfig::ParseContext::evaluate(const std::string& tpl)
{
	static const boost::regex regex(R"(\$\((\w+) ([^\(\)]+)\))");

	boost::smatch res;

	if(boost::regex_search(tpl, res, regex))
	{
		std::string cmd = res[1].str();
		std::string args = res[2].str();
		std::string value = "";

		if(cmd == "find")
		{
			// This command is actually very hard to resolve. It dates back
			// to the rosbuild days, where all products from a package where
			// contained in the package directory.
			// Under catkin, this is not the case anymore. So we actually
			// have to look at the package directory in src, devel, install
			// spaces.

			// => FIXME: This is not enough!
			value = PackageRegistry::getPath(args);
			if(value.empty())
				throw error("%s: $(find %s): Could not find package\n", filename().c_str(), args.c_str());
		}
		else if(cmd == "arg")
		{
			auto it = m_args.find(args);
			if(it == m_args.end())
				throw error("%s: $(arg %s): Unknown arg", filename().c_str(), args.c_str());

			value = it->second;

			if(value == UNSET_MARKER)
				throw error("%s: $(arg %s): Accessing unset argument", filename().c_str(), args.c_str());
		}
		else
			throw error("Unsupported subst command '%s'", cmd.c_str());

		std::stringstream ss;
		ss << tpl.substr(0, res.position()) << value << tpl.substr(res.position() + res.length());

		return evaluate(ss.str());
	}
	else
		return tpl;
}

bool LaunchConfig::ParseContext::shouldSkip(TiXmlElement* e)
{
	const char* if_cond = e->Attribute("if");
	if(if_cond)
	{
		std::string expansion = evaluate(if_cond);
		if(expansion == "0" || expansion == "false")
			return true;
		else if(expansion == "1" || expansion == "true")
			return false;
		else
			throw error("%s:%d: Unknown truth value '%s'", filename().c_str(), e->Row(), expansion.c_str());
	}

	const char* unless_cond = e->Attribute("unless");
	if(unless_cond)
	{
		std::string expansion = evaluate(unless_cond);
		if(expansion == "1" || expansion == "true")
			return true;
		else if(expansion == "0" || expansion == "false")
			return false;
		else
			throw error("%s:%d: Unknown truth value '%s'", filename().c_str(), e->Row(), expansion.c_str());
	}

	return false;
}

void LaunchConfig::ParseContext::setArg(const std::string& name, const std::string& value, bool override)
{
	auto it = m_args.find(name);
	if(it == m_args.end())
		m_args[name] = value;
	else if(override || it->second == UNSET_MARKER)
		m_args[name] = value;
}

LaunchConfig::LaunchConfig(const FDWatcher::Ptr& watcher)
 : m_fdWatcher(watcher)
{
}

LaunchConfig::~LaunchConfig()
{
}

void LaunchConfig::setArgument(const std::string& name, const std::string& value)
{
	m_rootContext.setArg(name, value, true);
}

void LaunchConfig::parse(const std::string& filename)
{
	TiXmlDocument document(filename);

	TiXmlBase::SetCondenseWhiteSpace(false);

	if(!document.LoadFile())
	{
		throw error("Could not load launch file '%s'. Exiting...\n", filename.c_str());
	}

	ros::WallTime start = ros::WallTime::now();
	m_rootContext.setFilename(filename);
	parse(document.RootElement(), m_rootContext);
	printf("Loaded launch file in %fs\n", (ros::WallTime::now() - start).toSec());

	for(auto it : m_params)
	{
		log("parameter: %s\n", it.first.c_str());
	}

	log("\n==================================\nNodes:\n");
	for(auto node : m_nodes)
	{
		std::vector<std::string> cmd = node->composeCommand();
		std::stringstream ss;
		for(auto part : cmd)
			ss << part << " ";

		log(" - %s\n", ss.str().c_str());
	}
}

void LaunchConfig::setParameters()
{
	ros::NodeHandle nh;
	for(auto it : m_params)
	{
		nh.setParam(it.first, it.second);
	}
}

void LaunchConfig::parse(TiXmlElement* element, ParseContext ctx)
{
	// First pass: Parse arguments
	for(TiXmlNode* n = element->FirstChild(); n; n = n->NextSibling())
	{
		TiXmlElement* e = n->ToElement();
		if(!e)
			continue;

		if(ctx.shouldSkip(e))
			continue;

		if(e->ValueStr() == "arg")
			parseArgument(e, ctx);
	}

	// Second pass: everything else
	for(TiXmlNode* n = element->FirstChild(); n; n = n->NextSibling())
	{
		TiXmlElement* e = n->ToElement();
		if(!e)
			continue;

		if(ctx.shouldSkip(e))
			continue;

		if(e->ValueStr() == "node")
			parseNode(e, ctx);
		else if(e->ValueStr() == "param")
			parseParam(e, ctx);
		else if(e->ValueStr() == "rosparam")
			parseROSParam(e, ctx);
		else if(e->ValueStr() == "group")
		{
			const char* ns = e->Attribute("ns");

			ParseContext cctx = ctx;

			if(ns)
				cctx = cctx.enterScope(ctx.evaluate(ns));

			parse(e, cctx);
		}
		else if(e->ValueStr() == "include")
			parseInclude(e, ctx);
	}
}

void LaunchConfig::parseNode(TiXmlElement* element, ParseContext ctx)
{
	const char* name = element->Attribute("name");
	const char* pkg = element->Attribute("pkg");
	const char* type = element->Attribute("type");
	const char* args = element->Attribute("args");
	const char* ns = element->Attribute("ns");

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

	Node::Ptr node = boost::make_shared<Node>(m_fdWatcher, m_nh, ctx.evaluate(name), ctx.evaluate(pkg), ctx.evaluate(type));

	if(args)
		node->addExtraArguments(ctx.evaluate(args));

	if(!fullNamespace.empty())
		node->setNamespace(fullNamespace);

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
	}

	node->logMessageSignal.connect(logMessageSignal);

	m_nodes.push_back(node);
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
		std::string fullValue = ctx.evaluate(value);

		if(type)
		{
			// Fixed type.

			try
			{
				if(strcmp(type, "int") == 0)
					m_params[fullName] = boost::lexical_cast<int>(fullValue);
				else if(strcmp(type, "double") == 0)
					m_params[fullName] = boost::lexical_cast<double>(fullValue);
				else if(strcmp(type, "bool") == 0)
				{
					if(fullValue == "true")
						m_params[fullName] = true;
					else if(fullValue == "false")
						m_params[fullName] = false;
					else
					{
						throw error("%s:%d: invalid boolean value '%s'",
							ctx.filename().c_str(), element->Row(), fullValue.c_str()
						);
					}
				}
				else if(strcmp(type, "str") == 0)
					m_params[fullName] = fullValue;
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

			if(fullValue == "true")
				m_params[fullName] = XmlRpc::XmlRpcValue(true);
			else if(fullValue == "false")
				m_params[fullName] = XmlRpc::XmlRpcValue(false);
			else
			{
				try { m_params[fullName] = boost::lexical_cast<int>(fullValue); return; }
				catch(boost::bad_lexical_cast&) {}

				try { m_params[fullName] = boost::lexical_cast<float>(fullValue); return; }
				catch(boost::bad_lexical_cast&) {}

				m_params[fullName] = fullValue;
			}
		}
	}
	else if(command)
	{
		std::string fullCommand = ctx.evaluate(command);

		std::stringstream buffer;

		FILE* f = popen(fullCommand.c_str(), "r");
		while(!feof(f))
		{
			char buf[1024];
			int ret = fread(buf, 1, sizeof(buf)-1, f);
			if(ret <= 0)
				throw error("Could not read: %s", strerror(errno));

			buf[ret] = 0;
			buffer << buf;
		}
		pclose(f);

		m_params[fullName] = buffer.str();
	}
	else if(textfile)
	{
		std::string fullFile = ctx.evaluate(textfile);
		std::ifstream stream(fullFile);
		if(stream.bad())
			throw error("%s:%d: Could not open file '%s'", ctx.filename().c_str(), element->Row(), fullFile.c_str());

		std::stringstream buffer;
		buffer << stream.rdbuf();

		m_params[fullName] = buffer.str();
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

		std::string contents;
		if(file)
		{
			std::string fullFile = ctx.evaluate(file);
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

		const char* name = element->Attribute("name");
		if(name)
			ctx = ctx.enterScope(ctx.evaluate(name));

		// Remove trailing / from prefix to get param name
		loadYAMLParams(n, ctx.prefix().substr(0, ctx.prefix().length()-1));
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
			break;
		}
		default:
			throw error("invalid yaml node type");
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
		childCtx = childCtx.enterScope(ns);

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

	parse(document.RootElement(), childCtx);
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

void LaunchConfig::start()
{
	for(auto node : m_nodes)
	{
		node->start();
	}
}

void LaunchConfig::shutdown()
{
	for(auto node : m_nodes)
		node->shutdown();
}

void LaunchConfig::forceExit()
{
	log("Killing the following nodes, which are refusing to exit:\n");
	for(auto node : m_nodes)
	{
		if(node->running())
		{
			log(" - %s\n", node->name().c_str());
			node->forceExit();
		}
	}
}

bool LaunchConfig::allShutdown()
{
	bool allShutdown = true;
	for(auto node : m_nodes)
	{
		if(node->running())
			allShutdown = false;
	}

	return allShutdown;
}

void LaunchConfig::log(const char* fmt, ...)
{
	static char buf[512];

	va_list v;
	va_start(v, fmt);

	vsnprintf(buf, sizeof(buf), fmt, v);

	va_end(v);

	logMessageSignal("[rosmon]", buf);
}

}

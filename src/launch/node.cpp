// Launch configuration for a single Node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "node.h"

#include "../package_registry.h"

#include <wordexp.h>
#include <glob.h>

#include <cstdarg>

static std::runtime_error error(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	char str[1024];

	vsnprintf(str, sizeof(str), fmt, args);

	va_end(args);

	return std::runtime_error(str);
}

namespace rosmon
{

namespace launch
{

Node::Node(std::string name, std::string package, std::string type)
 : m_name(std::move(name))
 , m_package(std::move(package))
 , m_type(std::move(type))

 // NOTE: roslaunch documentation seems to suggest that this is true by default,
 //  however, the source tells a different story...
 , m_respawn(false)
 , m_respawnDelay(1.0)

 , m_required(false)
 , m_coredumpsEnabled(true)
{
	m_executable = PackageRegistry::getExecutable(m_package, m_type);
}

void Node::addRemapping(const std::string& from, const std::string& to)
{
	m_remappings[from] = to;
}

void Node::addExtraArguments(const std::string& argString)
{
	wordexp_t tokens;

	// Get rid of newlines since this confuses wordexp
	std::string clean = argString;
	for(auto& c : clean)
	{
		if(c == '\n' || c == '\r')
			c = ' ';
	}

	// NOTE: This also does full shell expansion (things like $PATH)
	//   But since we trust the user here (and modifying PATH etc dooms us in
	//   any case), I think we can use wordexp here.
	int ret = wordexp(clean.c_str(), &tokens, WRDE_NOCMD);
	if(ret != 0)
		throw error("You're supplying something strange in 'args': '%s' (wordexp ret %d)", clean.c_str(), ret);

	for(unsigned int i = 0; i < tokens.we_wordc; ++i)
		m_extraArgs.emplace_back(tokens.we_wordv[i]);

	wordfree(&tokens);
}

void Node::setNamespace(const std::string& ns)
{
	m_namespace = ns;
}

void Node::setExtraEnvironment(const std::map<std::string, std::string>& env)
{
	m_extraEnvironment = env;
}

void Node::setRequired(bool required)
{
	m_required = required;
}

void Node::setRespawn(bool respawn)
{
	m_respawn = respawn;
}

void Node::setRespawnDelay(const ros::WallDuration& respawnDelay)
{
	m_respawnDelay = respawnDelay;
}

void Node::setLaunchPrefix(const std::string& launchPrefix)
{
	wordexp_t tokens;

	// Get rid of newlines since this confuses wordexp
	std::string clean = launchPrefix;
	for(auto& c : clean)
	{
		if(c == '\n' || c == '\r')
			c = ' ';
	}

	// NOTE: This also does full shell expansion (things like $PATH)
	//   But since we trust the user here (and modifying PATH etc dooms us in
	//   any case), I think we can use wordexp here.
	int ret = wordexp(clean.c_str(), &tokens, WRDE_NOCMD);
	if(ret != 0)
		throw error("You're supplying something strange in 'launch-prefix': '%s' (wordexp ret %d)", clean.c_str(), ret);

	for(unsigned int i = 0; i < tokens.we_wordc; ++i)
		m_launchPrefix.emplace_back(tokens.we_wordv[i]);

	wordfree(&tokens);
}

void Node::setCoredumpsEnabled(bool on)
{
	m_coredumpsEnabled = on;
}

}

}

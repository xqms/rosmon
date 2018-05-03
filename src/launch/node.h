// Launch configuration for a single Node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSMON_LAUNCH_NODE_H
#define ROSMON_LAUNCH_NODE_H

#include <string>
#include <map>
#include <memory>
#include <vector>

#include <ros/time.h>

namespace rosmon
{

namespace launch
{

class Node
{
public:
	typedef std::shared_ptr<Node> Ptr;
	typedef std::shared_ptr<const Node> ConstPtr;

	Node(std::string name, std::string package, std::string type);

	void addRemapping(const std::string& from, const std::string& to);
	void addExtraArguments(const std::string& argString);
	void setNamespace(const std::string& ns);
	void setExtraEnvironment(const std::map<std::string, std::string>& env);
	void setCoredumpsEnabled(bool on);

	void setRespawn(bool respawn);
	void setRespawnDelay(const ros::WallDuration& respawnDelay);

	void setLaunchPrefix(const std::string& launchPrefix);

	std::string name() const
	{ return m_name; }

	std::string package() const
	{ return m_package; }

	std::string type() const
	{ return m_type; }

	std::string executable() const
	{ return m_executable; }

	std::string namespaceString() const
	{ return m_namespace; }

	std::map<std::string, std::string> remappings() const
	{ return m_remappings; }

	std::vector<std::string> extraArguments() const
	{ return m_extraArgs; }

	std::map<std::string, std::string> extraEnvironment() const
	{ return m_extraEnvironment; }

	bool respawn() const
	{ return m_respawn; }

	ros::WallDuration respawnDelay() const
	{ return m_respawnDelay; }

	void setRequired(bool required);

	bool required() const
	{ return m_required; }

	std::vector<std::string> launchPrefix() const
	{ return m_launchPrefix; }

	bool coredumpsEnabled() const
	{ return m_coredumpsEnabled; }
private:
	std::string m_name;
	std::string m_package;
	std::string m_type;

	std::string m_executable;

	std::string m_namespace;

	std::map<std::string, std::string> m_remappings;
	std::vector<std::string> m_extraArgs;

	std::map<std::string, std::string> m_extraEnvironment;

	bool m_respawn;
	ros::WallDuration m_respawnDelay;

	bool m_required;

	std::vector<std::string> m_launchPrefix;

	bool m_coredumpsEnabled;
};

}

}

#endif

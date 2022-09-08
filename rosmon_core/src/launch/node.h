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

	void setRemappings(const std::map<std::string, std::string>& remappings);
	void addExtraArguments(const std::string& argString);
	void setNamespace(const std::string& ns);
	void setExtraEnvironment(const std::map<std::string, std::string>& env);
	void setCoredumpsEnabled(bool on);

	void setRespawn(bool respawn);
	void setRespawnDelay(const ros::WallDuration& respawnDelay);
	void setNumRespawnsAllowed(int numRespawnsAllowed);
	void setSpawnDelay(const ros::WallDuration& spawnDelay);

	void setLaunchPrefix(const std::string& launchPrefix);

	void setWorkingDirectory(const std::string& workingDirectory);

	void setClearParams(bool on);

	void setStopTimeout(double timeout);

	void setMemoryLimit(uint64_t memoryLimitByte);

	void setCPULimit(double cpuLimit);

	void setMuted(bool muted);
	void setStdoutDisplayed(bool showStdout);

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

	int numRespawnsAllowed() const
	{ return m_numRespawnsAllowed; }

	ros::WallDuration spawnDelay() const
	{ return m_spawnDelay; }

	void setRequired(bool required);

	bool required() const
	{ return m_required; }

	std::vector<std::string> launchPrefix() const
	{ return m_launchPrefix; }

	bool coredumpsEnabled() const
	{ return m_coredumpsEnabled; }

	std::string workingDirectory() const
	{ return m_workingDirectory; }

	bool clearParams() const
	{ return m_clearParams; }

	double stopTimeout() const
	{ return m_stopTimeout; }

	uint64_t memoryLimitByte() const
	{ return m_memoryLimitByte;}

	double cpuLimit() const
	{ return m_cpuLimit; }

	bool isMuted() const
	{ return m_muted; }

	bool stdoutDisplayed() const
	{ return m_stdoutDisplayed; }
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
	int m_numRespawnsAllowed = 0;  // Will warn on any respawn by default
	ros::WallDuration m_spawnDelay;

	bool m_required;

	std::vector<std::string> m_launchPrefix;

	bool m_coredumpsEnabled;

	std::string m_workingDirectory;

	bool m_clearParams;

	double m_stopTimeout;

	uint64_t m_memoryLimitByte;
	double m_cpuLimit;

	bool m_muted;
	bool m_stdoutDisplayed;
};

}

}

#endif

// Provides cached ros::package::getPath() lookups
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "package_registry.h"

#include <ros/package.h>

namespace rosmon
{

static std::map<std::string, std::string> g_cache;

std::string PackageRegistry::getPath(const std::string& package)
{
	auto it = g_cache.find(package);
	if(it == g_cache.end())
	{
		std::string path = ros::package::getPath(package);
		g_cache[package] = path;
		return path;
	}
	else
		return it->second;
}

}

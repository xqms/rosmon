// Provides cached ros::package::getPath() lookups
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "package_registry.h"

#include <ros/package.h>
#include <ros/time.h>

#include <rospack/rospack.h>

namespace rosmon
{

static std::map<std::string, std::string> g_cache;
static rospack::Rospack g_pack;

void PackageRegistry::init()
{
	std::vector<std::string> sp;
	g_pack.getSearchPathFromEnv(sp);
	g_pack.crawl(sp, false);
}

std::string PackageRegistry::getPath(const std::string& package)
{
	auto it = g_cache.find(package);
	if(it == g_cache.end())
	{
		ros::WallTime t1 = ros::WallTime::now();

		std::string path;
		if(!g_pack.find(package, path))
			path.clear();

		g_cache[package] = path;
		return path;
	}
	else
		return it->second;
}

}

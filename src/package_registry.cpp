// Provides cached ros::package::getPath() lookups
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "package_registry.h"

#include <ros/package.h>
#include <ros/time.h>

#include <rospack/rospack.h>

#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>

#include <sys/types.h>
#include <sys/wait.h>

namespace rosmon
{

static std::map<std::string, std::string> g_cache;
static rospack::Rospack g_pack;
static std::vector<std::string> g_catkin_workspaces;
static std::map<std::pair<std::string, std::string>, std::string> g_executableCache;
static bool g_initialized = false;

namespace fs = boost::filesystem;

static void init()
{
	if(g_initialized)
		return;

	std::vector<std::string> sp;
	g_pack.getSearchPathFromEnv(sp);
	g_pack.crawl(sp, false);

	// Determine stack of catkin workspaces
	char* env_cmake = getenv("CMAKE_PREFIX_PATH");
	if(env_cmake)
	{
		std::string cmakePath = env_cmake;
		boost::char_separator<char> sep(":");
		boost::tokenizer<boost::char_separator<char>> tok(cmakePath, sep);

		for(auto token : tok)
		{
			fs::path path(token);
			if(!fs::exists(path / ".catkin"))
				continue;

// 			printf("Found catkin workspace: '%s'\n", path.string().c_str());
			g_catkin_workspaces.push_back(path.string());
		}
	}
}

std::string PackageRegistry::getPath(const std::string& package)
{
	if(!g_initialized)
		init();

	auto it = g_cache.find(package);
	if(it == g_cache.end())
	{
		std::string path;
		if(!g_pack.find(package, path))
			path.clear();

		g_cache[package] = path;
		return path;
	}

	return it->second;
}

static std::string getExecutableInPath(const fs::path& path, const std::string& name)
{
	if(!fs::exists(path))
		return std::string();

	for(fs::recursive_directory_iterator it(path); it != fs::recursive_directory_iterator(); ++it)
	{
		if(it->path().filename() == name && access(it->path().c_str(), X_OK) == 0)
		{
			return it->path().string();
		}
	}

	return std::string();
}

static std::string _getExecutable(const std::string& package, const std::string& name)
{
	if(!g_initialized)
		init();

	// Try catkin libexec & catkin share first
	for(const auto& workspace : g_catkin_workspaces)
	{
		fs::path workspacePath(workspace);

		fs::path execPath = workspacePath / "lib" / package / name;
		if(fs::exists(execPath) && access(execPath.c_str(), X_OK) == 0)
			return execPath.string();

		std::string sharePath = getExecutableInPath(workspacePath / "share" / package, name);
		if(!sharePath.empty())
			return sharePath;
	}

	// Crawl package directory for an appropriate executable
	std::string packageDir = PackageRegistry::getPath(package);
	if(!packageDir.empty())
		return getExecutableInPath(packageDir, name);

	// Nothing found :-(
	return std::string();
}

std::string PackageRegistry::getExecutable(const std::string& package, const std::string& name)
{
	std::pair<std::string, std::string> key(package, name);

	auto it = g_executableCache.find(key);
	if(it != g_executableCache.end())
		return it->second;

	std::string result = _getExecutable(package, name);
	g_executableCache[key] = result;

	return result;
}

std::string PackageRegistry::findPathToFile(const std::string& package, const std::string& name)
{
	if(!g_initialized)
		init();

	// Try catkin libexec & catkin share first
	for(const auto& workspace : g_catkin_workspaces)
	{
		fs::path workspacePath(workspace);

		fs::path execPath = workspacePath / "lib" / package;
		fs::path filePath = execPath / name;
		if(fs::exists(filePath) && access(filePath.c_str(), X_OK) == 0)
			return execPath.string();

		fs::path sharePath = workspacePath / "share" / package;
		filePath = sharePath / name;
		if(fs::exists(filePath) && access(filePath.c_str(), X_OK) == 0)
			return sharePath.string();
	}

	// Try package directory (src)
	fs::path packageDir = PackageRegistry::getPath(package);
	fs::path filePath = packageDir / name;
	if(fs::exists(filePath) && access(filePath.c_str(), X_OK) == 0)
		return packageDir.string();

	// Nothing found :-(
	return std::string();
}

}

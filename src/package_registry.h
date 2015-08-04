// Provides cached ros::package::getPath() lookups
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PACKAGE_REGISTRY_H
#define PACKAGE_REGISTRY_H

#include <string>

namespace rosmon
{

class PackageRegistry
{
public:
	static void init();
	static std::string getPath(const std::string& package);

	static std::string getExecutable(const std::string& package, const std::string& name);
};

}

#endif


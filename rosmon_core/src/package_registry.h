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
	static std::string getPath(const std::string& package);

	static std::string getExecutable(const std::string& package, const std::string& name);

	/**
	 * @brief Find path to a package file
	 *
	 * This function tries to find the package path (src, libexec, share) that
	 * contains the mentioned file/directory. This is used for $(find ...)/...
	 * lookups.
	 *
	 * @param package ROS package name
	 * @param name relative path inside the package
	 **/
	static std::string findPathToFile(const std::string& package, const std::string& name);
};

}

#endif


#include "resource_checker.h"
#include <ros/ros.h>
#include "octets_parse.h"

using namespace rosmon_diagnostics;

ResourceChecker::ResourceChecker()
{
    ros::NodeHandle nh("~");
    defaultMemoryLimit_byte =
        nh.param("defaultMemoryLimit_byte", 10000000);           // 10 Mbyte by default.
    defaultUserCPULimit = nh.param("defaultUserCPULimit", 0.05); // 5%

    std::map<std::string, std::string> memoryWrtNodeNameLiteral;
    nh.getParam("memory_limits", memoryWrtNodeNameLiteral);
    nh.getParam("user_cpu_limits", userCPUWrtNodeName);

    fillMemoryMapFromLitteral(memoryWrtNodeNameLiteral);
}

uint64_t ResourceChecker::getMaxAllowedMemory(const std::string& nodeName) const
{
    auto it = memoryWrtNodeName.find(nodeName);
    if(it == memoryWrtNodeName.end())
    {
        return defaultMemoryLimit_byte;
    }
    return it->second;
}

float ResourceChecker::getMaxUserAllowedCPU(const std::string& nodeName) const
{
    auto it = userCPUWrtNodeName.find(nodeName);
    if(it == userCPUWrtNodeName.end())
    {
        return defaultUserCPULimit;
    }
    return it->second;
}

void ResourceChecker::fillMemoryMapFromLitteral(
    const std::map<std::string, std::string> literal)
{
    OctetsParser parser;
    uint64_t memory;
    for(const auto& memWrtNodeName : literal)
    {
        if(parser.parseMemory(memWrtNodeName.second, memory))
        {
            memoryWrtNodeName.insert(std::make_pair(memWrtNodeName.first, memory));
        }
    }
}

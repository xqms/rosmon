#include "resource_checker.h"
#include "octets_parse.h"
#include <ros/ros.h>

using namespace rosmon_diagnostics;

ResourceChecker::ResourceChecker()
{
    ros::NodeHandle nh("~");
    std::string strMemoryLimit =
        nh.param("defaultMemoryLimit_byte", std::string("15 MB"));
    parser.parseMemory(strMemoryLimit, defaultMemoryLimit_byte);

    defaultCPULimit = nh.param("defaultCPULimit", 0.05); // 5%

    std::map<std::string, std::string> memoryWrtNodeNameLiteral;
    nh.getParam("memory_limits", memoryWrtNodeNameLiteral);
    nh.getParam("cpu_limits", CPUWrtNodeName);

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

float ResourceChecker::getMaxAllowedCPU(const std::string& nodeName) const
{
    auto it = CPUWrtNodeName.find(nodeName);
    if(it == CPUWrtNodeName.end())
    {
        return defaultCPULimit;
    }
    return it->second;
}

void ResourceChecker::fillMemoryMapFromLitteral(
    const std::map<std::string, std::string> literal)
{
    uint64_t memory;
    for(const auto& memWrtNodeName : literal)
    {
        if(parser.parseMemory(memWrtNodeName.second, memory))
        {
            memoryWrtNodeName.insert(std::make_pair(memWrtNodeName.first, memory));
        }
    }
}

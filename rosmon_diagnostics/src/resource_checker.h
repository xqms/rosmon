#pragma once
#include <map>
#include <stdint.h>
#include <string>

namespace rosmon_diagnostics
{

class ResourceChecker
{
public:
    ResourceChecker();
    uint64_t getMaxAllowedMemory(const std::string& nodeName) const;
    float getMaxUserAllowedCPU(const std::string& nodeName) const;



protected:
    std::map<std::string, uint64_t> memoryWrtNodeName;
    std::map<std::string, float> userCPUWrtNodeName;
    uint64_t defaultMemoryLimit_byte;
    float defaultUserCPULimit; // normalized between 0 and 1.
    void fillMemoryMapFromLitteral(const std::map<std::string, std::string> literal);
};

} // namespace rosmon_diagnostics

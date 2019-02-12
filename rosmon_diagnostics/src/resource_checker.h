#pragma once
#include <map>
#include <stdint.h>
#include <string>
#include "octets_parse.h"

namespace rosmon_diagnostics
{

class ResourceChecker
{
public:
    ResourceChecker();
    uint64_t getMaxAllowedMemory(const std::string& nodeName) const;
    float getMaxAllowedCPU(const std::string& nodeName) const;

protected:
    std::map<std::string, uint64_t> memoryWrtNodeName;
    std::map<std::string, float> CPUWrtNodeName;
    uint64_t defaultMemoryLimit_byte;
    float defaultCPULimit;      // normalized between 0 and <core count>.
    void fillMemoryMapFromLitteral(const std::map<std::string, std::string> literal);
    OctetsParser parser;
};

} // namespace rosmon_diagnostics

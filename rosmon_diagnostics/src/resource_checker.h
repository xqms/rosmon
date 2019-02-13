#pragma once
#include "bytes_parser.h"
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
    float getMaxAllowedCPU(const std::string& nodeName) const;

protected:
    std::map<std::string, uint64_t> memoryWrtNodeName;
    std::map<std::string, float> CPUWrtNodeName;
    uint64_t defaultMemoryLimit_byte;
    float defaultCPULimit; // normalized between 0 and <core count>.
    void fillMemoryMapFromLitteral(const std::map<std::string, std::string>& literal);
    ByteParser parser;
};

} // namespace rosmon_diagnostics

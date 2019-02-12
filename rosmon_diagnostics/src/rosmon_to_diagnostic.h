#pragma once

#include "resource_checker.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <rosmon_msgs/State.h>

namespace rosmon_diagnostics
{
class RosmonToDiagnostic
{
public:
    RosmonToDiagnostic();
    void onNewStateMessage(const rosmon_msgs::State& state);

    const diagnostic_msgs::DiagnosticArray& getCurrentDiagnosticArray() const
    {
        return currentDiagnosticArray;
    }
    static std::string memoryToString(uint64_t memory);
protected:
    ResourceChecker resourceChecker;
    diagnostic_msgs::DiagnosticArray currentDiagnosticArray;
    std::string diagnosticNamePrefix = "processes";
};

} // namespace rosmon_diagnostics

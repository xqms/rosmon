// Evaluates $(eval ...) substitutions
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSMON_LAUNCH_SUBSTITUTION_PYTHON_H
#define ROSMON_LAUNCH_SUBSTITUTION_PYTHON_H

#include <string>

namespace rosmon
{
namespace launch
{

class ParseContext;

std::string evaluatePython(const std::string& input, ParseContext& context);

}
}

#endif

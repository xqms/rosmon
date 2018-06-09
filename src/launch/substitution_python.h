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

//! Evaluate a $(eval ...) python expression
std::string evaluatePython(const std::string& input, ParseContext& context);

//! Evaluate a deg(...) rosparam expression
double evaluateROSParamPython(const std::string& input);

}
}

#endif

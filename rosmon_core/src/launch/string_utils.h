// String utilities
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSMON_LAUNCH_STRING_UTILS_H
#define ROSMON_LAUNCH_STRING_UTILS_H

#include <string>

namespace rosmon
{
namespace launch
{
namespace string_utils
{

/**
 * @brief Compress any sequence of whitespace to single spaces.
 *
 * Since we switch of space condensing in TinyXML to be able to properly parse
 * <rosparam> tags, this function can be used for attributes.
 *
 * roslaunch also strips whitespace at begin/end, so we do that as well.
 **/
std::string simplifyWhitespace(const std::string& input);

/**
 * @brief Check if string is whitespace only (includes '\n')
 **/
bool isOnlyWhitespace(const std::string& input);

}
}
}

#endif

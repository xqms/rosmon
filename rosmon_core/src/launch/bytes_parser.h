// Parse memory specifications like "100 MB"
// Authors: Adrien Barral, Max Schwarz

#ifndef ROSMON_LAUNCH_BYTES_PARSER_H
#define ROSMON_LAUNCH_BYTES_PARSER_H

#include <tuple>

namespace rosmon
{
namespace launch
{

/**
 * Note: this uses 1 KB == 1000 Byte.
 *
 * @return Tuple memory in bytes and success
 */
std::tuple<uint64_t, bool> parseMemory(const std::string& memory);

}
}

#endif

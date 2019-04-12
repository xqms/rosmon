#pragma once

#include <tuple>

namespace rosmon {

namespace launch {
class ByteParser
{
public:
    std::tuple<uint64_t, bool> parseMemory(const std::string& memory);

private:
};
}

}

#pragma once

#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/qi.hpp>
#include <fmt/format.h>
#include <tuple>

namespace rosmon {

namespace launch {
class ByteParser
{
public:
    std::tuple<uint64_t, bool> parseMemory(const std::string& memory);

private:
    struct bytes_decades_ : boost::spirit::qi::symbols<char, uint64_t>
    {
        bytes_decades_();
    } byte_decades;
};

class ByteStringGenerator {
public :
    static std::string memoryToString(uint64_t memory)
    {
        if(memory < 1000)
        {
            return fmt::format("{} Bytes", memory);
        }
        else if(memory < 1e6)
        {
            return fmt::format("{:.2f} KB", memory / 1e3);
        }
        else if(memory < 1e9)
        {
            return fmt::format("{:.2f} MB", memory / 1e6);
        }
        else if(memory < 1e12)
        {
            return fmt::format("{:.2f} GB", memory / 1e9);
        }
        return fmt::format("{:.2f} TB", memory / 1e12);
    }

};

}

}

#pragma once

#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/qi.hpp>
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
}

}

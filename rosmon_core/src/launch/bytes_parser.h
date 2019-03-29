#pragma once

#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/qi.hpp>
#include <fmt/format.h>
#include <ros/console.h>

namespace rosmon {

namespace launch {
class ByteParser
{
public:
    bool parseMemory(const std::string& memory, uint64_t& outMemory)
    {
        using boost::phoenix::ref;
        using boost::spirit::qi::_1;
        using boost::spirit::qi::double_;
        using boost::spirit::qi::no_case;
        using boost::spirit::qi::phrase_parse;

        auto it = memory.begin();
        double res = 0.0;
        bool ok = phrase_parse(
            it, memory.end(),
            (double_[ref(res) = _1] >> -(byte_decades[ref(res) *= _1])),
            boost::spirit::ascii::space);
        if(!ok)
        {
            ROS_WARN_STREAM("can't parse memory value : " << memory);
            return false;
        }
        outMemory = static_cast<uint64_t>(res);
        return true;
    }

private:
    struct bytes_decades_ : boost::spirit::qi::symbols<char, uint64_t>
    {
        bytes_decades_()
        {
            add("kB", 1e3)("mB", 1e6)("gB", 1e9)("tB", 1e12)("KB", 1e3)("MB", 1e6)(
                "GB", 1e9)("TB", 1e12)("B", 1);
        }
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

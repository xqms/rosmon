#pragma once

#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/qi.hpp>
#include <ros/console.h>

class OctetsParser
{
public:
    bool parseMemory(const std::string& memory, uint64_t& outMemory)
    {
        using boost::phoenix::ref;
        using boost::spirit::qi::_1;
        using boost::spirit::qi::double_;
        using boost::spirit::qi::phrase_parse;
        using boost::spirit::qi::no_case;

        auto it = memory.begin();
        double res = 0.0;
        bool ok =
            phrase_parse(it, memory.end(),
                         (double_[ref(res) = _1] >> -no_case[(octets_decades[ref(res) *= _1])]),
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
    struct octets_decades_ : boost::spirit::qi::symbols<char, uint64_t>
    {
        octets_decades_() { add("ko", 1e3)("mo", 1e6)("go", 1e9)("to", 1e12)("o", 1); }
    } octets_decades;
};


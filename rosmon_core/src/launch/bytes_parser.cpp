// Parse memory specifications like "100 MB" orr "100MiB"
// Authors: Adrien Barral, Max Schwarz

#include "bytes_parser.h"

#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/qi.hpp>

#include <ros/console.h>

namespace rosmon
{
namespace launch
{

std::tuple<uint64_t, bool> parseMemory(const std::string& memory)
{
	using boost::phoenix::ref;
	using boost::spirit::qi::_1;
	using boost::spirit::qi::double_;
	using boost::spirit::qi::no_case;
	using boost::spirit::qi::phrase_parse;

	struct bytes_decades_ : boost::spirit::qi::symbols<char, uint64_t>
	{
		bytes_decades_(){
			add("kB", 1e3)("mB", 1e6)("gB", 1e9)("tB", 1e12)("KB", 1e3)("MB", 1e6)("GB", 1e9)(
				"TB", 1e12)("B", 1)("KiB", static_cast<uint64_t>(1ull<<10))("MiB",
				static_cast<uint64_t>(1ull<<20))("GiB", static_cast<uint64_t>(1ull<<30))("TiB",
				static_cast<uint64_t>(1ull<<40));
		}
	} byte_decades;

	auto it = memory.begin();
	double res = 0.0;
	bool ok = phrase_parse(it, memory.end(),
		(double_[ref(res) = _1] >> -(byte_decades[ref(res) *= _1])),
		boost::spirit::ascii::space
	);

	return std::make_tuple(static_cast<uint64_t>(res), ok);
}

}
}

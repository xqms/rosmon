// String utilities
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "string_utils.h"

namespace rosmon
{
namespace launch
{
namespace string_utils
{

std::string simplifyWhitespace(const std::string& input)
{
	std::string output;
	output.reserve(input.size());

	// Skip initial space
	size_t i = 0;
	for(; i < input.size(); ++i)
	{
		if(!std::isspace(static_cast<unsigned char>(input[i])))
			break;
	}

	bool in_space = false;

	for(; i < input.size(); ++i)
	{
		char c = input[i];

		if(std::isspace(static_cast<unsigned char>(c)))
			in_space = true;
		else
		{
			if(in_space)
				output.push_back(' ');

			output.push_back(c);
			in_space = false;
		}
	}

	return output;
}

bool isOnlyWhitespace(const std::string& input)
{
	for(const char& c: input)
	{
		// see http://en.cppreference.com/w/cpp/string/byte/isspace
		// for reason for casting
		if(!std::isspace(static_cast<unsigned char>(c)))
			return false;
	}

	return true;
}

}
}
}

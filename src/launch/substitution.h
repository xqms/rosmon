// Parse the $(...) substitution args
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSMON_LAUNCH_SUBSTITUTION_H
#define ROSMON_LAUNCH_SUBSTITUTION_H

#include <string>
#include <stdexcept>

namespace rosmon
{
namespace launch
{

class ParseContext;

class SubstitutionException : public std::exception
{
public:
	explicit SubstitutionException(const std::string& msg)
	 : m_msg(msg)
	{}

	virtual ~SubstitutionException() throw()
	{}

	virtual const char* what() const noexcept
	{ return m_msg.c_str(); }
private:
	std::string m_msg;
};

namespace substitutions
{
	std::string anon(const std::string& name, ParseContext& context);
	std::string arg(const std::string& name, const ParseContext& context);
	std::string dirname(const ParseContext& context);
	std::string env(const std::string& name);
	std::string optenv(const std::string& name, const std::string& defaultValue);

	//! $(find ...) which always gives `rospack find` results
	std::string find_stupid(const std::string& name);
}

std::string parseSubstitutionArgs(const std::string& input, ParseContext& context);

}
}

#endif

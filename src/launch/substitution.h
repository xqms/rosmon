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

std::string parseSubstitutionArgs(const std::string& string, ParseContext& context);

}
}

#endif

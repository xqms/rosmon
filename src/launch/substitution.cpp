// Parse the $(...) substitution args
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "substitution.h"

#include "launch_config.h"
#include "../package_registry.h"

#include <boost/algorithm/string/trim.hpp>

#include <stdarg.h>

namespace rosmon
{
namespace launch
{

typedef std::function<std::string(const std::string&, const std::string&)> Handler;
typedef std::map<std::string, Handler> HandlerMap;

enum ParserState
{
	PARSER_IDLE,
	PARSER_DOLLAR,
	PARSER_INSIDE,
};

static SubstitutionException error(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	char str[1024];

	vsnprintf(str, sizeof(str), fmt, args);

	va_end(args);

	return SubstitutionException(str);
}

static std::string parseOneElement(const std::string& input, const HandlerMap& handlers, bool strict, bool* found)
{
	ParserState state = PARSER_IDLE;
	std::size_t begin = 0;

	for(std::size_t i = 0; i < input.size(); ++i)
	{
		char c = input[i];

		switch(state)
		{
			case PARSER_IDLE:
				if(c == '$')
					state = PARSER_DOLLAR;
				break;
			case PARSER_DOLLAR:
				if(c == '(')
				{
					state = PARSER_INSIDE;
					begin = i+1;
				}
				break;
			case PARSER_INSIDE:
				if(c == '$')
					state = PARSER_DOLLAR;
				else if(c == ')')
				{
					// We found something!
					std::string contents = input.substr(begin, i - begin);
					std::string after = input.substr(i+1);

					// Split into name and args
					auto pos = contents.find(' ');
					std::string name = contents.substr(0, pos);
					std::string args;
					if(pos != std::string::npos)
						args = contents.substr(pos+1);

					auto it = handlers.find(name);
					if(it != handlers.end())
					{
						std::string replacement = it->second(args, after);

						std::stringstream ss;
						ss << input.substr(0, begin-2);
						ss << replacement;
						ss << after;

						*found = true;
						return ss.str();
					}
					else if(strict)
					{
						throw error("Unknown substitution arg '%s'", name.c_str());
					}

					state = PARSER_IDLE;
				}
				break;
		}
	}

	*found = false;
	return input;
}

std::string parseSubstitutionArgs(const std::string& input, ParseContext& context)
{
	bool found = false;
	std::string buffer = input;

	HandlerMap simpleHandlers = {
		{"arg", [&context](const std::string& args, const std::string&) -> std::string{
			auto it = context.arguments().find(args);
			if(it == context.arguments().end())
				throw error("$(arg %s): Unknown arg", args.c_str());

			std::string value = it->second;

			if(value == UNSET_MARKER)
			{
				throw error(
					"$(arg %s): Accessing unset argument '%s', please specify as parameter.", args.c_str(), args.c_str()
				);
			}

			return value;
		}},
		{"env", [](const std::string& args, const std::string&) -> std::string{
			const char* envval = getenv(args.c_str());
			if(!envval)
				throw error("$(env %s): Environment variable not set!", args.c_str());

			return envval;
		}},
		{"optenv", [](const std::string& args, const std::string&) -> std::string{
			auto pos = args.find(" ");
			std::string defaultValue;
			std::string name = args;
			if(pos != std::string::npos)
			{
				defaultValue = args.substr(pos + 1);
				name = args.substr(0, pos);
			}

			const char* envval = getenv(name.c_str());
			if(envval)
				return envval;
			else
				return defaultValue;
		}},
		{"env", [&context](const std::string& args, const std::string&) -> std::string{
			std::string base = args;
			boost::trim(base);

			return context.config()->anonName(base);
		}},
	};

	do
	{
		buffer = parseOneElement(buffer, simpleHandlers, false, &found);
	}
	while(found);

	HandlerMap findHandlers = {
		{"find", [](const std::string& args, const std::string& after) -> std::string{
			// Extract path after $(find ...)
			auto pos = after.find(' ');
			std::string filename = after.substr(0, pos);
			boost::trim(filename);

			if(!filename.empty())
			{
				std::string path = PackageRegistry::findPathToFile(args, filename);
				if(!path.empty())
					return path;
			}

			// Fallback to "rospack find"
			std::string path = PackageRegistry::getPath(args);
			if(!path.empty())
				return path;

			throw error("$(find %s): Could not find package", args.c_str());
		}},
	};

	do
	{
		buffer = parseOneElement(buffer, findHandlers, true, &found);
	}
	while(found);

	return buffer;
}

}
}

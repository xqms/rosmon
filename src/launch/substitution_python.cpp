// Evaluates $(eval ...) substitutions
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "substitution_python.h"
#include "substitution.h"

#include "launch_config.h"

#include <boost/python/exec.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/dict.hpp>
#include <boost/python/make_function.hpp>
#include <boost/python/import.hpp>

#include <boost/lexical_cast.hpp>

namespace py = boost::python;

namespace rosmon
{
namespace launch
{

static SubstitutionException error(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	char str[1024];

	vsnprintf(str, sizeof(str), fmt, args);

	va_end(args);

	return SubstitutionException(str);
}

#if HAVE_PYTHON

template<class F>
py::object make_handler(F f)
{
	return py::make_function(
		f,
		py::default_call_policies(),
		boost::mpl::vector<std::string, const std::string&>()
	);
}

template<class F>
py::object make_handler0(F f)
{
	return py::make_function(
		f,
		py::default_call_policies(),
		boost::mpl::vector<std::string>()
	);
}

template<class F>
py::object make_handler2(F f)
{
	return py::make_function(
		f,
		py::default_call_policies(),
		boost::mpl::vector<std::string, const std::string&, const std::string&>()
	);
}

static py::object toPython(const std::string& value)
{
	if(value == "true" || value == "True")
		return py::object(true);

	if(value == "false" || value == "False")
		return py::object(false);

	try { return py::object(boost::lexical_cast<int>(value)); }
	catch(boost::bad_lexical_cast&) {}

	try { return py::object(boost::lexical_cast<float>(value)); }
	catch(boost::bad_lexical_cast&) {}

	return py::object(value);
}

static py::object pythonArg(const std::string& name, ParseContext& context)
{
	return toPython(substitutions::arg(name, context));
}

static bool g_initialized = false;

std::string evaluatePython(const std::string& input, ParseContext& context)
{
	if(!g_initialized)
	{
		Py_Initialize();
		g_initialized = true;
	}

	py::object main_module = py::import("__main__");
	py::dict global = py::dict(main_module.attr("__dict__"));
	py::dict local;

	// Add arguments
	{
		for(auto& arg : context.arguments())
		{
			local[arg.first] = toPython(arg.second);
		}
	}

	// Add substitution functions
	{
		local["anon"] = make_handler([&context](const std::string& name){
			return substitutions::anon(name, context);
		});
		local["arg"] = py::make_function([&context](const std::string& name) -> py::object{
			return pythonArg(name, context);
		}, py::default_call_policies(), boost::mpl::vector<py::object, const std::string&>());
		local["dirname"] = make_handler0([&context](){
			return substitutions::dirname(context);
		});
		local["env"] = make_handler(substitutions::env);
		local["optenv"] = make_handler2(substitutions::optenv);

		local["find"] = make_handler(substitutions::find_stupid);
	}

	// Import math
	{
		py::object math = py::import("math");
		py::object mathDict = math.attr("__dict__");
		global.update(mathDict);
	}

	py::object result;
	try
	{
		result = py::eval(input.c_str(), global, local);
	}
	catch(py::error_already_set&)
	{
		std::stringstream ss;
		ss << "Caught Python exception while evaluating $(eval " << input << "):\n";

		PyObject *e, *v, *t;
		PyErr_Fetch(&e, &v, &t);

		// A NULL e means that there is not available Python
		// exception
		if(v)
		{
			std::string strErrorMessage = py::extract<std::string>(v);
			ss << strErrorMessage;
		}
		throw error(ss.str().c_str());
	}

	py::extract<std::string> asString(result);
	if(asString.check())
	{
		return asString();
	}

	py::extract<bool> asBool(result);
	if(asBool.check())
	{
		if(asBool())
			return "true";
		else
			return "false";
	}

	py::extract<int> asInt(result);
	if(asInt.check())
		return std::to_string(asInt());

	py::extract<float> asFloat(result);
	if(asFloat.check())
		return boost::lexical_cast<std::string>(asFloat()); // to_string has low precision

	throw error("$(eval '%s'): Got unknown python return type", input.c_str());
}

#else // HAVE_PYTHON

std::string evaluatePython(const std::string& input, ParseContext& context)
{
	throw SubstitutionException(
		"rosmon was built without python support, so I cannot parse $(eval ...) substitution args."
	);
}

#endif

}
}

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

		try
		{
			py::object t = py::extract<py::object>(e);
			py::object t_name = t.attr("__name__");
			std::string typestr = py::extract<std::string>(t_name);

			ss << typestr << ": ";
		}
		catch(py::error_already_set const &)
		{}

		try
		{
			py::object vo = py::extract<py::object>(v);
			std::string valuestr = py::extract<std::string>(vo.attr("__str__")());

			ss << valuestr;
		}
		catch(py::error_already_set const &)
		{
			ss << "<no str() handler>";
		}

		throw SubstitutionException(ss.str());
	}

#if PY_MAJOR_VERSION >= 3
	if(PyUnicode_Check(result.ptr()))
	{
		if(PyUnicode_READY(result.ptr()) != 0)
			throw SubstitutionException("Could not read unicode object");

		return std::string(PyUnicode_AsUTF8(result.ptr()));
	}
#else
	if(PyString_Check(result.ptr()))
		return py::extract<std::string>(result);
#endif

	if(PyBool_Check(result.ptr()))
	{
		if(py::extract<bool>(result))
			return "true";
		else
			return "false";
	}

#if PY_MAJOR_VERSION >= 3
	if(PyLong_Check(result.ptr()))
		return std::to_string(py::extract<int64_t>(result));
#else
	if(PyInt_Check(result.ptr()) || PyLong_Check(result.ptr()))
	{
		return std::to_string(py::extract<int64_t>(result));
	}
#endif

	if(PyFloat_Check(result.ptr()))
	{
		// std::to_string has low precision here, so use boost::lexical_cast
		return boost::lexical_cast<std::string>(
			py::extract<float>(result)()
		);
	}

	throw SubstitutionException::format("$(eval '{}'): Got unknown python return type", input);
}

double evaluateROSParamPython(const std::string& input)
{
	if(!g_initialized)
	{
		Py_Initialize();
		g_initialized = true;
	}

	py::object main_module = py::import("__main__");
	py::dict global = py::dict(main_module.attr("__dict__"));
	py::dict local;

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
		ss << "Caught Python exception while evaluating rosparam expression '" << input << "'):\n";

		PyObject *e, *v, *t;
		PyErr_Fetch(&e, &v, &t);

		try
		{
			py::object t = py::extract<py::object>(e);
			py::object t_name = t.attr("__name__");
			std::string typestr = py::extract<std::string>(t_name);

			ss << typestr << ": ";
		}
		catch(py::error_already_set const &)
		{}

		try
		{
			py::object vo = py::extract<py::object>(v);
			std::string valuestr = py::extract<std::string>(vo.attr("__str__")());

			ss << valuestr;
		}
		catch(py::error_already_set const &)
		{
			ss << "<no str() handler>";
		}

		throw SubstitutionException(ss.str());
	}

	try
	{
		return py::extract<double>(result);
	}
	catch(py::error_already_set)
	{
		throw SubstitutionException::format("got strange python type from rosparam python expression (should be numeric): '{}'", input);
	}
}

#else // HAVE_PYTHON

std::string evaluatePython(const std::string& input, ParseContext& context)
{
	throw SubstitutionException(
		"rosmon was built without python support, so I cannot parse $(eval ...) substitution args."
	);
}

double evaluateROSParamPython(const std::string& input)
{
	try
	{
		return boost::lexical_cast<double>(input);
	}
	catch(boost::bad_lexical_cast)
	{
		throw SubstitutionException::format(
			"rosmon was built without python support, so I cannot parse complex rosparam expressions (expression: {})",
			input
		);
	}
}

#endif

}
}

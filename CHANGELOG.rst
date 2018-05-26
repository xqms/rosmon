^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosmon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.6 (2018-05-26)
------------------
* test/xml: replace more multiline string Catch captures
  Sorry, somehow these slipped through - and I didn't have a good way of
  testing these locally without the buildfarm. I'm testing with gcc 4.8
  on trusty now, which seems to have matching behavior.
* Contributors: Max Schwarz

1.0.5 (2018-05-25)
------------------
* test/xml: avoid multiline Catch captures in exception tests
  These trigger some weird bug between g++ 5.4 and ccache, which is used
  in the ROS buildfarm. [...]
  This should fix compilation on the build farm.
* Contributors: Max Schwarz

1.0.4 (2018-05-24)
------------------
* Merge pull request `#30 <https://github.com/xqms/rosmon/issues/30>`_ from xqms/feature/global_remap
  Support <remap> in other scopes than <node>. Fixes `#28 <https://github.com/xqms/rosmon/issues/28>`_.
* Merge pull request `#29 <https://github.com/xqms/rosmon/issues/29>`_ from xqms/feature/fmt
  Port all string formatting to fmt
* update README.md, refer to ROS wiki
  Otherwise we duplicate the information.
* launch: keep pointer to current element in ParseContext
  This is in preparation for a refactoring of the error handling code. This
  way, we don't have to explicitly pass line number information around - we
  can instead pull it from the ParseContext when the error is generated.
* Merge pull request `#27 <https://github.com/xqms/rosmon/issues/27>`_ from xqms/feature/spec_tests
  roslaunch/XML spec unit tests
* launch: launch_config: error on <include clear_params="true" />
  Even the roslaunch/XML spec says this is "extremely dangerous". We will
  explicitly *not* support that one for now.
* launch: substitution_python: fix type deduction
  py::extract actually includes automatic conversion, so it is not
  appropriate for checking the returned object type. Use Python API instead.
* launch: handle <node clear_params="true"> attribute
* launch: launch_config: add support for <node cwd="..." />
* launch: launch_config: node uniqueness check should consider namespaces
* launch: substitution_python: report python exceptions more completely
* launch: launch_config: error if node name is not unique
* launch: launch_config: accept True/False as boolean values as well
  We are lenient here and accept the pythonic forms "True" and "False"
  as well, since roslaunch seems to do the same. Even the roslaunch/XML
  spec mentions True/False in the examples, even though they are not
  valid options for if/unless and other boolean attributes...
  http://wiki.ros.org/roslaunch/XML/rosparam
* launch_config: add check for invalid <param> combinations
* xml: param: test robustness against malformed tags
* launch: launch_config: propagate exceptions from lazy param threads
  .. to main thread.
* launch: launch_config: check if <param> commands exit normally
* launch_config: handle binfile
* launch: handle type "yaml" parameters (new in roslaunch since Lunar)
  This is actually a bit complicated, since this breaks a previous assumption
  we made: Our lazy evaluation of parameters depend on a 1:1 mapping of
  parameter names to jobs - this is not the case with YAML parameters, since
  one YAML file can turn into multiple params on the parameter server.
  So we handle YAML parameters separately from "ordinary" parameters, i.e.
  here our lazy evaluation does not prevent multiple loadings of the same
  parameters.
* cmake: basic rostest depends on rosmon target
  This makes sure that "make run_tests" also (re-)builds rosmon.
* launch: larger refactoring of param parsing
  Simplifies the forced type logic and applies it to "command" and "textfile"
  results as well.
* launch: split off as shared library and offer string parsing interface
  Preparation for more specific unit tests on roslaunch XML loading.
* CMakeLists.txt: option to create clang source-based coverage builds
* Contributors: Max Schwarz, Matthias Nieuwenhuisen

1.0.3 (2018-05-05)
------------------
* launch_config: ignore empty YAML data in <rosparam> tags
  See `#12 <https://github.com/xqms/rosmon/issues/12>`_ for discussion as to why this is necessary.
  TL;DR: roslaunch does it.
  Co-authored-by: Lucas Coelho Figueiredo <lucascoelhof@gmail.com>
* launch_config: simplify whitespace inside ParseContext::evaluate()
  This should fix problems with whitespace such as `#1 <https://github.com/xqms/rosmon/issues/1>`_,
  `#16 <https://github.com/xqms/rosmon/issues/16>`_, `#22 <https://github.com/xqms/rosmon/issues/22>`_.
* ui: calculate node name padding correctly on 32 bit architectures
  Fixes `#19 <https://github.com/xqms/rosmon/issues/19>`_.
* add LICENSE file
* address clang-tidy warnings
* launch_config: handle relative params with tilde + validate names
  This also prints a more informative error message on malformed parameter
  names.
* launch_config: support pass_all_args
* Contributors: Max Schwarz

1.0.2 (2018-04-24)
------------------
* node_monitor: don't collect core dumps from launch-prefixed nodes
* node_monitor: fix error message on failed execvp()
  Previously, the error message was not printed to the screen, as log() is
  not useful in the child process. Rather, use the intended communication
  channel (stdout/stderr) to print log messages.
* Contributors: Max Schwarz

1.0.1 (2018-04-13)
------------------
* Fix compilation issues on Ubuntu Artful and Debian Stretch.
* Contributors: Max Schwarz

1.0.0 (2018-04-13)
------------------
* Initial release
* Contributors: David Schwarz, Gabriel Arjones, Kartik Mohta, Max Schwarz, Philipp Allgeuer

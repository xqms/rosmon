^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosmon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.1 (2023-09-29)
------------------

2.5.0 (2023-07-10)
------------------

2.4.0 (2021-08-02)
------------------
* Increase cmake minimum version to 3.4
* Contributors: Max Schwarz

2.3.2 (2020-05-29)
------------------

2.3.1 (2020-05-28)
------------------

2.3.0 (2020-05-28)
------------------

2.2.1 (2019-11-08)
------------------

2.2.0 (2019-10-29)
------------------

2.1.1 (2019-07-09)
------------------

2.1.0 (2019-06-25)
------------------
* Handle namespaces in service calls correctly, PR #80
* Add namespaces to status topics, PR #78
* Contributors: Artur Miller, David Walsh, Max Schwarz

2.0.2 (2019-03-12)
------------------

2.0.1 (2019-03-12)
------------------

2.0.0 (2019-03-11)
------------------
* split into separate packages for core, GUI, messages, PR #72
* Contributors: Max Schwarz

1.0.10 (2018-10-29)
-------------------
* main: exit gracefully on SIGTERM and SIGHUP as well (issue #59, PR #60)
* launch: allow <arg default="XY"> in <include> tags (issue #57, PR #58)
  roslaunch allows this, so we should as well. Since it seems a bit
  confusing, we issue a warning when this happens.
  In these cases, <arg value="XY"> should be applicable and is much clearer.
* Contributors: Max Schwarz

1.0.9 (2018-09-20)
------------------
* Fix race condition in integration test (issue #42, PR #54)
* Clean namespace names properly to fix double slashes (issue #49, PR #53)
* Respect ROS_NAMESPACE for nested launches (issue #46, PR #51)
* gui: Sort Memory column correctly (issue #48, PR #50)
* Try to find an executable *file* for nodes (issue #45, PR #47)
* Add --stop-timeout option and launch file attribute (PR #37)
* Handle params with leading slashes inside nodes (PR #40)
* Add --no-start option (PR #39)
* gui: Fix index bug in showContextMenu (PR #38)
* Contributors: Max Schwarz, Nikos Skalkotos, Romain Reignier

1.0.8 (2018-08-07)
------------------
* main: Add option for flushing the logfile
  Add --flush option that will flush the logfile after each entry.
* Merge pull request `#35 <https://github.com/xqms/rosmon/issues/35>`_
  More complete support for rosparam features
* Merge pull request `#34 <https://github.com/xqms/rosmon/issues/34>`_
  YAML quoted strings
* launch: rosparam: support binary data
* launch: rosparam: correctly handle explicit YAML type tags
* launch: support rosparam angle computations
* launch: always map YAML quoted values to string params
  These always get mapped to str by python's yaml.load, which is used by
  roslaunch, so we do the same here.
* Contributors: Max Schwarz, Nikos Skalkotos

1.0.7 (2018-05-27)
------------------
* Support Python 3 & select appropriate Python version.
  This fixes a mismatch on Debian Jessie, where rospack is linked against
  Python 2.7 and we would link against Python 3.4.
* launch: substitution_python: support Python 3
* Contributors: Max Schwarz

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

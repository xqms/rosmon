^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosmon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

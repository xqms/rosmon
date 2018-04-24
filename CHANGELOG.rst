^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosmon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

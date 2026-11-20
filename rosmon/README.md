![attachment:screenshot.svg](doc/screenshot.svg)

## Why rosmon?

rosmon is a drop-in replacement for the venerable
[roslaunch](https://index.ros.org/p/roslaunch) tool. It offers some
interesting features both for monitoring and debugging:

- rosmon has a clean, modern console UI. With one glance at the status
  bar, you can see that all of your nodes are still running as they
  should. Console output from the nodes is intercepted and augmented
  with the node name - allowing you to immediately see *which* node is
  printing that message. This also works for plain output without
  rosconsole macros.
- rosmon has a ROS interface itself. Query node status or start/stop
  nodes using service calls. This allows you to use ROS network
  transparency for remote monitoring.
- rosmon is developer friendly. With two keystrokes, you can attach gdb
  to a running node. If a node crashes without gdb attached, you can run
  gdb against an automatically collected coredump, providing post-mortem
  analysis.
- rosmon is *fast*. Thanks to lazy evaluation of parameters, costly
  calls to [xacro](https://index.ros.org/p/xacro) and friends are
  reduced as much as possible.

## Differences to roslaunch

The [XML](https://index.ros.org/p/roslaunch) specification is complex
and a bit vague, so you can expect some differences. Bug reports are
welcome! Known differences include:

- rosmon will not launch [roscore](roscore) automatically. We believe
  that it is important to make a conscious decision how and where to run
  the roscore. Often, restarting roslaunch unknowingly restarts the
  contained roscore, leading to hard-to-debug problems with other
  connected nodes.
- The `<machine>` tag is not supported.

For additional information and rosmon extensions not supported by
roslaunch see [XML](#xml).

rosmon was developed for the DARPA Robotics Challenge, which influenced
our decision to develop it in a statically typed language (C++) for more
runtime robustness. However, this meant that
[roslaunch](https://index.ros.org/p/roslaunch) XML parsing code could
not be used.

## Installation

Binary packages are available:



```
sudo apt install ros-${ROS_DISTRO}-rosmon

source /opt/ros/${ROS_DISTRO}/setup.bash # Needed to use the 'mon launch' shortcut
```



If you just need the rosmon CLI and don't require the `rqt_rosmon` rqt
plugin, you can install the smaller package
`ros-${ROS_DISTRO}-rosmon-core` instead.

Recommendation: To benefit from compressed core dump storage and
explicit core dump storage limits, install systemd-coredump.

## Usage

When installed, rosmon drops a handy shell wrapper into your shell
context (currently supported in bash and zsh). You may have to re-source
your workspace environment after building. Usage (similar to roslaunch):



```
mon launch <package> <launch file> [arguments]
```



or



```
mon launch <path to launch file> [arguments]
```



Full usage information:



```
Usage:
  rosmon [actions] [options] some_package test.launch [arg1:=value1 ...]
  rosmon [actions] [options] path/to/test.launch [arg1:=value1 ...]

Actions (default is to launch the launch file):
  --benchmark    Exit after loading the launch file
  --list-args    List launch file arguments

Options:
  --disable-ui   Disable fancy terminal UI
  --help         This help screen
  --log=FILE     Write log file to FILE
  --name=NAME    Use NAME as ROS node name. By default, an anonymous
                 name is chosen.

rosmon also obeys some environment variables:
  ROSMON_COLOR_MODE   Can be set to 'truecolor', '256colors', 'ansi'
                      to force a specific color mode
                      If unset, rosmon tries to detect the best
                      available color mode.
```



## Console interface

rosmon features a basic console interface. Each node is assigned a
dedicated key, which can be used to open the action menu for that node.
Available actions are:

- Starting/stopping the node (keys 's' and 'k')
- Starting a debugger, either attached to the running process or
  post-mortem against a collected coredump (key 'd')

## GUI interface

rosmon can also be controlled remotely using an
[rqt](https://index.ros.org/p/rqt) plugin, which uses rosmon's ROS
interface. The context menu offers starting and stopping control.
Additionally collected statistics (CPU and memory usage) are displayed.

## Nodes

### rosmon

Main rosmon node.

#### Published Topics

- `~state`
  ([rosmon/msg/State](https://docs.ros.org/en/rolling/p/rosmon/msg/State.html)):
  Current state of all controlled nodes. Updated every second.

#### Services

- `~start_stop`
  ([rosmon/srv/StartStop](https://docs.ros.org/en/rolling/p/rosmon/srv/StartStop.html)):
  Start or stop a controlled node.

## XML

This page documents additional XML tags and attributes supported by
rosmon. For the base launch file XML definition,
see [XML](https://index.ros.org/p/roslaunch).

### Root attributes

rosmon supports additional attributes on the root `<launch>` tag:

. `rosmon-name` . Sets the ROS name of the rosmon instance to a fixed
string, instead of an anonymous name. . `rosmon-window-title` . rosmon
will try to set the "title" of the terminal window to this value. In
`Konsole` and `tmux`, this sets the tab title. .
`rosmon-disable-ui` . Same effect as the `\--disable-ui` CLI flag:
Disables all interactive elements of the UI.

### Node attributes

Additional attributes on `<node>` tags (these can also be specified
on `<launch>`, `<include>`, `<group>` and then apply as
defaults to all nodes contained within):

. `enable-coredumps` . Set this to `false` to disable coredumps for
this node. This may be necessary if the process uses large amounts of
virtual memory (e.g. CUDA software), which takes a long time to dump to
disk. . `rosmon-cpu-limit` . If the node process consumes more CPU
than this limit, rosmon will send out diagnostic messages. This is
specified as ratio relative to one core, i.e. 1.5 means usage of
one-and-a-half CPU cores. The default is 0.9. . `rosmon-memory-limit`
. If the node process consumes more memory than this limit (specified in
bytes), it will send out diagnostic messages. The default is 500 MB. .
`rosmon-stop-timeout` . rosmon will wait this amount of seconds before
finally killing the process. If this is not set, rosmon will use the
value of the command line argument `\--stop-timeout=N` or the default
value of five seconds. . `rosmon-restart-warn-threshold` . When
`respawn="true"`, rosmon will automatically restart a node that
exited. By default, it will generate a diagnostic message at this point.
This attribute allows to configure a number of respawns that are
expected. If `rosmon-restart-warn-threshold` < 0, all restarts are
done silently without diagnostic messages.

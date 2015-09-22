# rosmon

![screenshot](https://github.com/xqms/rosmon/raw/master/doc/screenshot.png)

`rosmon` is a drop-in replacement for the standard `roslaunch` tool. Rather
unlike `roslaunch`, `rosmon` is focussed on (remote) process monitoring.

## Quick & effective development

The first thing to note is that `rosmon` sports a modern console UI with colored
indicators for each node. With one glance you can see that everything is still
running as it should.

How often have you stared at the screen, waiting for `roslaunch` to decide that
it is time to kill that one misbehaving node you have no control over (hello,
gazebo) that does not react to `SIGINT`? No more. `rosmon` gives nodes five
seconds to terminate, then they are killed.
This timeout will be configurable in the future.

Another nice feature is that `rosmon` intercepts all `stdout/stderr` output from
your nodes and is able to tag it, e.g. with the node name. That way, you don't
have to wonder anymore who is spamming the console with plain `printf` calls...

## Interaction

`rosmon` includes a simple keyboard user interface: Each node has an assigned
key. If you press that key, you get a menu in the status line which allows you
to start, stop, or debug a coredump of that very node.

## Remote control

`rosmon` offers a simple but effective ROS interface to start, stop or restart
nodes and monitor node states. Restarting a single stuck node out of the launch
file is no longer a problem. An `rqt` plugin for controlling `rosmon` is
included.

## Status

That said, `rosmon` is very much work-in-progress right now.

**WARNING**: `rosmon` is not yet `roslaunch` compliant. For example, the
`$(find PACKAGE)` substitution performs a simple search using `rospack` and
always returns the path to the package in the source tree. `roslaunch` may
choose to return the corresponding devel or install path if the file is found
there. Other inconsistencies and unsupported features may exist. Use at your
own risk.

## Building

Simple include this repository in your catkin workspace. After a build
(tested with `catkin_tools`) and re-sourcing of the `devel/setup.bash` you will
have the `mon` command in your environment.

`mon` supports the same argument style as `roslaunch` (either
`mon launch path/to/file.launch` or `mon launch package file.launch`).

## License

`rosmon` is licensed under BSD-3.

## Author

Max Schwarz <max.schwarz@uni-bonn.de>

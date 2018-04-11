# rosmon

![screenshot](https://github.com/xqms/rosmon/raw/master/doc/screenshot.png)

`rosmon` is a drop-in replacement for the standard `roslaunch` tool. Rather
unlike `roslaunch`, `rosmon` is focused on (remote) process monitoring.

## Quick & effective development

 * Modern console UI with colored indicators for each node. With one glance
   you can see that everything is still running as it should.
 * Debugger integration:
   * Automatic coredump collection for dying processes
   * Launch `gdb` against your running process or post-mortem against the
     collected coredump
 * `rosmon` intercepts all `stdout/stderr` from your nodes and tags it with
   a timestamp and the node name. No more wondering which of your nodes the
   mysterious warning came from.

## Interaction

`rosmon` includes a simple keyboard user interface: Each node has an assigned
key. If you press that key, you get a menu in the status line which allows you
to start, stop, or debug a coredump of that very node.

## Remote control

`rosmon` offers a simple but effective ROS interface to start, stop or restart
nodes and monitor node states. Restarting a single stuck node out of the launch
file is no longer a problem. An `rqt` plugin for controlling `rosmon` is
included.

## Security

`rosmon` disables the "Yama" `ptrace()` protection from its child processes, so
that the user can attach with `gdb` without needing root privileges.
This may be seen as a security risk.

## Building

Simple include this repository in your catkin workspace. After a build
(tested with `catkin_tools`) and re-sourcing of the `devel/setup.bash` you will
have the `mon` command in your environment.

`mon` supports the same argument style as `roslaunch` (either
`mon launch path/to/file.launch` or `mon launch package file.launch`).

## Robust core dump collection

rosmon will try to collect coredumps automatically on node crashes. However,
the default setting for `/proc/sys/kernel/core_pattern` is `core`, which means
that the coredump will be created in the working directory of the process.
rosmon therefore creates separate temporary working directories for the nodes.

If this is problematic (e.g. your nodes expect a certain working directory
or change working directory at runtime), you can setup a fixed `core_pattern`
on your system.
Add the following lines to `/etc/rc.local`:

    mkdir /tmp/cores
    chmod a+rw /tmp/cores
    echo "/tmp/cores/core.%e.%p.%t" > /proc/sys/kernel/core_pattern

and reboot (or execute `/etc/rc.local` once).

## License

`rosmon` is licensed under BSD-3.

## Author

Max Schwarz <max.schwarz@uni-bonn.de>

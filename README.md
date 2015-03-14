# rosmon

`rosmon` is a drop-in replacement for the standard `roslaunch` tool. It has two
main goals, which may differ from the goals of `roslaunch`:

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

## Remote control

`rosmon` offers a simple but effective ROS interface to start, stop or restart
nodes and monitor node states. Restarting a single stuck node out of the launch
file is no longer a problem.

## Status

That said, `rosmon` is very much work-in-progress right now. The sketched
features are all present, but have not been tested thoroughly.

## License

`rosmon` is licensed under BSD-3.

## Author

Max Schwarz <max.schwarz@uni-bonn.de>

# rosmon

![screenshot](https://github.com/xqms/rosmon/raw/master/doc/screenshot.png)

`rosmon` is a drop-in replacement for the standard `roslaunch` tool. Rather
unlike `roslaunch`, `rosmon` is focused on (remote) process monitoring.

Please see the [ROS wiki page](https://wiki.ros.org/rosmon) for further
information, the rest of this README contains information for `rosmon`
developers.

## Building

Simple include this repository in your catkin workspace. After a build
(tested with `catkin_tools`) and re-sourcing of the `devel/setup.bash` you will
have the `mon` command in your environment.

## License

`rosmon` is licensed under BSD-3. The `rosmon` repository includes the
[fmt](http://fmtlib.net/latest/index.html) library, which is also licensed
under BSD-2.

## Author

Max Schwarz <max.schwarz@ais.uni-bonn.de>

# rosmon

![screenshot](./rosmon_core/doc/screenshot.svg)

`rosmon` is a drop-in replacement for the standard `roslaunch` tool. Rather
unlike `roslaunch`, `rosmon` is focused on (remote) process monitoring.

Please see the [ROS wiki page](https://wiki.ros.org/rosmon) for further
information, the rest of this README contains information for `rosmon`
developers.

Looking for a similar ROS2 solution? Take a look at
[better_launch](https://github.com/dfki-ric/better_launch)!

## Installation & Quick start

Official `rosmon` packages are available. On ROS Kinetic, Melodic, and Noetic (Ubuntu) simply do:

    sudo apt install ros-${ROS_DISTRO}-rosmon

In case you don't need the rqt GUI, you can install the core package only:

    sudo apt install ros-${ROS_DISTRO}-rosmon-core

Afterwards, try

    # launch one of the rosmon test cases
    mon launch rosmon_core basic.launch

## Building from source

Simple include this repository in your catkin workspace. `rosmon` depends on
[rosfmt], so make sure you either have `rosfmt` installed using the ROS packages
or you compile it from source in your catkin workspace. After a build
(tested with `catkin_tools`) and re-sourcing of the `devel/setup.bash` you will
have the `mon` command in your environment.

[rosfmt]: https://github.com/xqms/rosfmt

## License

`rosmon` is licensed under BSD-3.

## Author

Max Schwarz <max.schwarz@ais.uni-bonn.de>

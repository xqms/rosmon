
# If you change this file, please increment the version number in
# CMakeLists.txt to trigger a CMake update.

# cache rosmon executable so that we don't need a rosrun invocation each
# time we do completion
_ROSMON_EXECUTABLE=$(rosrun --prefix echo rosmon rosmon)

function mon() {
	case $1 in
		launch)
			shift
			rosrun rosmon rosmon $*
			;;
	esac
}

function _packages() {
	packages=("${(@f)$(rospack list-names 2> /dev/null)}")
	_values 'ROS packages' $packages
}

function _file_or_package() {
	_alternative \
		'launch:launch file:{_files -g "*.launch"}' \
		"package:ROS package:_packages"
}

function _launch_arg() {
	if [[ -f ${line[2]} ]]; then
		args=("${(@f)$($_ROSMON_EXECUTABLE --list-args ${line[2]} 2> /dev/null)}")
	else
		args=("${(@f)$($_ROSMON_EXECUTABLE --list-args ${line[2]} ${line[3]} 2> /dev/null)}")
	fi
	args=(${^args}"\\::argument:()")
	if [[ $#args -gt 1 ]]; then
		_values -S '=' 'launch args' $args
	fi
}

function _launch_file() {
	if [[ -f ${line[2]} ]]; then
		_launch_arg
	else
		package=${line[2]}
		files=("${(@f)$(find $(rospack find ${package}) -name '*.launch' -type f -printf "%f\n")}")
		if [[ $#files -gt 0 ]]; then
			_values 'launch files' $files
		fi
	fi
}

function _mon() {
	_arguments \
		'1:cmd:{_values "commands" launch}' \
		'--disable-ui[Disable UI]' \
		'--benchmark[Benchmark]' \
		'--help[Help]' \
		'--list-args[List launch file arguments]' \
		'--log[Log output file]:filename:_files' \
		'--name[ROS node name]:node_name:()' \
		':launch file or package:_file_or_package' \
		'::launch file:_launch_file' \
		'*:launch arg:_launch_arg'
}
compdef _mon mon


# If you change this file, please increment the version number in
# CMakeLists.txt to trigger a CMake update.

# cache rosmon executable so that we don't need a rosrun invocation each
# time we do completion
_ROSMON_EXECUTABLE=$(rosrun --prefix echo rosmon_core rosmon)

function mon() {
	case $1 in
		launch)
			shift
			rosrun rosmon_core rosmon "$@"
			;;
	esac
}

function _mon() {
	local cur="${COMP_WORDS[COMP_CWORD]}"
	local cmd="${COMP_WORDS[1]}"

	local package_name=""
	local launch_name=""

	local FLAGS=( --disable-ui --benchmark --help --list-args )
	local OPTS=( --log --name )
	local i=2

	# Find package name and launch file name if they have been specified already
	while [[ $i -lt $COMP_CWORD ]]; do
		local arg="${COMP_WORDS[$i]}"

		if [[ $arg == --* ]]; then
			i=$(( i + 2 ));
		else
			if [[ -z $package_name ]]; then
				package_name="$arg"
			else
				launch_name="$arg"
				break
			fi
			i=$(( i + 1 ));
		fi
	done

	case "${COMP_CWORD}" in
		1)
			COMPREPLY=( $(compgen -W "launch" -- $cur) )
			;;
		*)
			# Are we currently inside an option?
			if [[ " ${OPTS[@]} " =~ " ${COMP_WORDS[COMP_CWORD-1]} " ]]; then
				case "${COMP_WORDS[COMP_CWORD-1]}" in
					--log)
						COMPREPLY=( $(compgen -f -- $cur) )
						;;
					*)
						COMPREPLY=()
						;;
				esac

			# If we have no package name yet, offer package names
			elif [[ -z $package_name ]]; then
				local packages=$(rospack list-names 2>/dev/null)
				COMPREPLY=( $(compgen -W "${packages} ${FLAGS[*]} ${OPTS[*]}" -- $cur) )
				compopt -o default

			# If we have no launch file yet, offer launch files
			elif [[ ( ! -f $package_name ) && -z $launch_name ]]; then
				local package_dir="$(rospack find ${COMP_WORDS[2]})"
				local launchfiles=$(find -L "$package_dir" -name '*.launch' -type f -printf "%f\n")
				COMPREPLY=( $(compgen -W "${launchfiles} ${FLAGS[*]} ${OPTS[*]}" -- $cur) )

			# Only arguments now
			else
				if [[ -f $package_name ]]; then
					local launch_arguments=$($_ROSMON_EXECUTABLE \
						--list-args "${package_name}" 2> /dev/null)
				else
					local launch_arguments=$($_ROSMON_EXECUTABLE \
						--list-args ${package_name} ${launch_name} 2> /dev/null)
				fi

				COMPREPLY=( $(compgen -o nospace -S ":=" -W "${launch_arguments}" -- $cur) )
				compopt -o nospace
			fi
			;;
	esac
}
complete -F _mon mon


# If you change this file, please increment the version number in
# CMakeLists.txt to trigger a CMake update.

function mon() {
	local workspace="$(roscd && pwd)"
	[[ "$(basename "$workspace")" == "devel" ]] && workspace="$(dirname "$workspace")"
	
	case $1 in
		launch)
			shift
			rosrun rosmon rosmon $*
			;;
	esac
}

function _mon() {
	local cur="${COMP_WORDS[COMP_CWORD]}"
	local cmd="${COMP_WORDS[1]}"

	case "${COMP_CWORD}" in
		1)
			COMPREPLY=( $(compgen -W "launch" -- $cur) )
			;;
		2)
			local workspace="$(roscd && pwd)"
			[[ "$(basename "$workspace")" == "devel" ]] && workspace="$(dirname "$workspace")"

			case "${cmd}" in
				launch)
					local packages=$(rospack list-names 2>/dev/null)
					local files=$(ls *.launch 2>/dev/null)
					COMPREPLY=( $(compgen -W "${packages} ${files}" -- $cur) )
					;;
			esac
			;;
		3)
			case "${cmd}" in
				launch)
					local launchfiles=$(find $(rospack find ${COMP_WORDS[2]}) -name '*.launch' -type f -printf "%f\n")
					COMPREPLY=( $(compgen -W "${launchfiles}" -- $cur) )
					;;
			esac
			;;
		*)
			COMPREPLY=()
			;;
	esac
}
complete -F _mon mon

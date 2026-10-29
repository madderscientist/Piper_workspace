#!/usr/bin/env bash
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    printf 'Use: source ./setup_env.bash\n' >&2
    exit 1
fi
_piper_x_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || return
if [[ ! -f "${_piper_x_root}/.venv/bin/activate" ]]; then
    printf 'Create the Python environment first; see README.md.\n' >&2
    unset _piper_x_root
    return 1
fi
source /opt/ros/humble/setup.bash || return
source "${_piper_x_root}/.venv/bin/activate" || return
if [[ -f "${_piper_x_root}/install/local_setup.bash" ]]; then
    source "${_piper_x_root}/install/local_setup.bash" || return
fi
unset _piper_x_root

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
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source "${_piper_x_root}/.venv/bin/activate" || return
if [[ -f "${_piper_x_root}/install/local_setup.bash" ]]; then
    source "${_piper_x_root}/install/local_setup.bash" || return
fi
export PIPER_CONFIG_DIR="${_piper_x_root}/config"
# First run creates the machine-local (git-ignored) skeleton; existing files are never overwritten.
mkdir -p \
    "${PIPER_CONFIG_DIR}/arms" \
    "${PIPER_CONFIG_DIR}/calibration" \
    "${PIPER_CONFIG_DIR}/paxini_driver" \
    "${PIPER_CONFIG_DIR}/wrist_camera" \
    "${PIPER_CONFIG_DIR}/head_camera" || return
if [[ ! -f "${PIPER_CONFIG_DIR}/env.bash" ]]; then
    cp "${PIPER_CONFIG_DIR}/examples/env.bash" "${PIPER_CONFIG_DIR}/env.bash" || return
fi
if ! compgen -G "${PIPER_CONFIG_DIR}/arms/*.yaml" > /dev/null; then
    printf 'config/arms/ is empty: copy this machine'"'"'s arms from config/examples/arms/ and set can_port.\n' >&2
fi
source "${PIPER_CONFIG_DIR}/env.bash" || return
unset _piper_x_root

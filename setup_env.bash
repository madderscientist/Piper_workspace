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
if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><FragmentSize>64500 B</FragmentSize><MaxMessageSize>65500 B</MaxMessageSize><MaxRexmitMessageSize>64500 B</MaxRexmitMessageSize></General><Internal><Watermarks><WhcLow>64 KiB</WhcLow><WhcHigh>8 MiB</WhcHigh><WhcHighInit>4 MiB</WhcHighInit><WhcAdaptive>false</WhcAdaptive></Watermarks><BurstSize><MaxInitTransmit>131072</MaxInitTransmit><MaxRexmit>128 KiB</MaxRexmit></BurstSize><DefragReliableMaxSamples>32</DefragReliableMaxSamples><NackDelay>2 ms</NackDelay><PreEmptiveAckDelay>1 ms</PreEmptiveAckDelay><AutoReschedNackDelay>10 ms</AutoReschedNackDelay><HeartbeatInterval min="2 ms" minsched="2 ms">5 ms</HeartbeatInterval></Internal></Domain></CycloneDDS>'
fi
source "${_piper_x_root}/.venv/bin/activate" || return
if [[ -f "${_piper_x_root}/install/local_setup.bash" ]]; then
    source "${_piper_x_root}/install/local_setup.bash" || return
fi
unset _piper_x_root

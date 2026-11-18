# Task-space Cartesian impedance target executed at driver inner loop rate (200 Hz) in MIT mode.
# tool_offset is expressed in link6 frame relative to the J6 (last joint) output flange center, in meters.
# Task-space and 20 kHz motor FOC damping are derived automatically from live q, tool_offset, inertia, and kp.
float64[3] position
float64[4] quat_wxyz
float64[3] tool_offset
float64[6] kp

# Task-space Cartesian impedance target at tool_offset in link6, executed at driver inner loop rate (200 Hz) in MIT mode.
# Joint kp is zeroed on wire; negative joint_kd entries auto-derive motor critical damping from live q, tool_offset, and kp.
float64[3] position
float64[4] quat_wxyz
float64[3] tool_offset
float64[6] kp
float64[6] kd
float64[6] joint_kd
float64 max_force
float64 max_torque

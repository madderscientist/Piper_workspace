# Joint-side units: rad, rad/s, Nm. Every command contains all six joints.
# Configured gravity is folded into target position (position + gravity / kp); torque is passed through as-is.
float64[6] position
float64[6] velocity
float64[6] kp
float64[6] kd
float64[6] torque

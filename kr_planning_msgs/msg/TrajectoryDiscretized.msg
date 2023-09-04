Header header
# Path is consist of multiple ordered waypoints
# this is mpc style points, N is number of controls
# pos, vel, acc, yaw are all N+1 length
# thrust, moment are N length
geometry_msgs/Point[] pos
geometry_msgs/Point[] vel
geometry_msgs/Point[] acc
float64[] yaw
float64[] thrust
geometry_msgs/Point[] moment
float64 dt
int32 N_ctrl
geometry_msgs/Quaternion inital_attitude
geometry_msgs/Point initial_omega


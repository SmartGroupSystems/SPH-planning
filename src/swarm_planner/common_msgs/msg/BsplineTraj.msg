# BsplineTraj.msg
std_msgs/Header header
int32           traj_id

geometry_msgs/Point[] position
geometry_msgs/Point[] velocity
geometry_msgs/Point[] acceleration
geometry_msgs/Point[] jerk

float32 yaw
float32 yaw_rate
uint32 current_seq

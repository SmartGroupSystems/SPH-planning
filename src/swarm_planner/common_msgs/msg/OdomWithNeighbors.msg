# OdomWithNeighbors.msg
std_msgs/Header header  # 时间戳和帧编号
std_msgs/String drone_name  # 无人机的名称
Odom myOdom            # 自定义的Odom类型
std_msgs/String[] drone_names  # 每个无人机的名称数组
Odom[] neighborsOdom   # 一个Odom数组，用于存储邻居的Odom信息

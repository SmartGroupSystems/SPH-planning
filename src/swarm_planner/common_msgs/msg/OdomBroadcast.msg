# OdomBroadcast.msg
std_msgs/Header header  # 时间戳和帧编号
std_msgs/String[] drone_names  # 每个无人机的名称数组
Odom[] OdomBroadcast  # 一个Odom数组，用于全部Odom信息
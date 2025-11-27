# Status of the MAVLink time synchroniser
# 时间同步状态消息
# 发布 ROS 与飞控的时间同步状态（同步误差、同步频率、是否锁定），确保多传感器数据的时间戳一致性

std_msgs/Header header
uint64 remote_timestamp_ns		# remote system timestamp (nanoseconds)
int64 observed_offset_ns		# raw time offset directly observed from this timesync packet (nanoseconds)
int64 estimated_offset_ns		# smoothed time offset between companion system and Mavros (nanoseconds)
float32 round_trip_time_ms		# round trip time of this timesync packet (milliseconds)
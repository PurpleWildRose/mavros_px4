# 系统状态消息
# 飞控硬件系统的核心健康状态（传感器在线状态、CPU 负载、内存使用、通信链路状态），是系统级故障诊断的核心

std_msgs/Header header

uint32 sensors_present
uint32 sensors_enabled
uint32 sensors_health
uint16 load
uint16 voltage_battery
int16 current_battery
int8 battery_remaining
uint16 drop_rate_comm
uint16 errors_comm
uint16 errors_count1
uint16 errors_count2
uint16 errors_count3
uint16 errors_count4
# Represent battery status from SYSTEM_STATUS
# 电池状态消息
# 发布飞控电池的实时状态（电压、电流、剩余电量百分比、电池循环次数），是低电量报警的核心数据来源
#
# To be replaced when sensor_msgs/BatteryState PR will be merged
# https://github.com/ros/common_msgs/pull/74

std_msgs/Header header
float32 voltage # [V]
float32 current # [A]
float32 remaining # 0..1

# APM ESC Telemetry as returned by BLHeli
# 电调遥测消息（单个）
# 高精度电调数据（如电机相电流、母线电压波动），用于电机控制算法优化或故障排查
#
# See:
# https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_1_TO_4
# https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_5_TO_8
# https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_9_TO_12

std_msgs/Header header

float32 temperature     # deg C
float32 voltage         # V
float32 current         # A
float32 totalcurrent    # Ah
int32 rpm               # 1/min
uint16 count            # count of telemetry packets

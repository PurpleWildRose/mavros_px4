# ESCStatus.msg
# 电调状态消息（集合）
# 实时发布电调工作状态（当前转速、输出功率、温度、错误码），用于监控电机驱动健康度（如电调过热报警）
#
# See mavlink message documentation here:
# https://mavlink.io/en/messages/common.html#ESC_STATUS

std_msgs/Header header

mavros_msgs/ESCStatusItem[] esc_status

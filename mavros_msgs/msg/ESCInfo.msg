# ESCInfo.msg
# 电调信息消息（集合）
# ESCInfo 是多个电调信息的集合, 详细参数（型号、固件版本、最大电流），用于电调硬件识别
#
# See mavlink message documentation here:
# https://mavlink.io/en/messages/common.html#ESC_INFO

std_msgs/Header header

uint16 counter
uint8 count
uint8 connection_type
uint8 info
mavros_msgs/ESCInfoItem[] esc_info


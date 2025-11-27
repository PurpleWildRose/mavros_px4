# ESCInfoItem.msg
# 电调信息消息（单个）
# ESCInfoItem 是单个电调的详细参数（型号、固件版本、最大电流），用于电调硬件识别
#
# See mavlink message documentation here:
# https://mavlink.io/en/messages/common.html#ESC_INFO

std_msgs/Header header

uint16 failure_flags
uint32 error_count
int32 temperature

